#include <Servo.h>
//camera servos
Servo servo1,servo2;

//motors as servos
Servo front_left,back_left,front_right,back_right;  // create servo object to control the PWM signal

//Servo Pins
int servoPin1= 2; //Top 
int servoPin2= 3; //Bottom 
//LED PINs
#define RED 50
#define GREEN 52 
#define BLUE 53 
#define Button 51 
//Sonar PINs
//looking at it from left to right (you being in front of robot)
#define T1trig 28 
#define T1echo 29 

#define T2trig 30 
#define T2echo 31  

#define T3trig 32  
#define T3echo 33 

#define T4trig 34 
#define T4echo 35 

#define T5trig 36 
#define T5echo 37 

#define T6trig 38 
#define T6echo 39 

//Bottom sonars
#define bottomtrig1 22 
#define bottomecho1 23 

#define bottomtrig2 24
#define bottomecho2 25 


//bumpers
#define Bumper1 48 
#define Bumper2 49 
#define Bumper3 46
#define Bumper4 47
#define Bumper5 44
#define Bumper6 45

//Global Variables
float follow_distance = 7; //Unit in inches
float turn_range = 13;
float LeftSensor1,LeftSensor2;
float percent_difference=LeftSensor1*.3;
float T1,T2,T3,T4,T5,T6;
float duration, distance;
char MODE;
int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button
volatile bool MotorState;
// bumper state variables
int Bumper1State,Bumper2State,Bumper3State,Bumper4State,Bumper5State,Bumper6State; 

//robot speed
int speed=20;

void setup(){
  Serial.begin (9600);
  //LED Setup
  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(BLUE,OUTPUT);

  pinMode (Button, INPUT_PULLUP);
  currentButtonState = digitalRead(Button);
  attachInterrupt(digitalPinToInterrupt(Button),ButtonRead,RISING);

  //Servo setup
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  //Sonar Setup
  pinMode(T1trig, OUTPUT);
  pinMode(T1echo, INPUT);
  pinMode(T2trig, OUTPUT);
  pinMode(T2echo, INPUT);
  pinMode(T3trig, OUTPUT);
  pinMode(T3echo, INPUT);
  pinMode(T4trig, OUTPUT);
  pinMode(T4echo, INPUT);
  pinMode(T5trig, OUTPUT);
  pinMode(T5echo, INPUT);
  pinMode(T6trig, OUTPUT);
  pinMode(T6echo, INPUT);
  pinMode(bottomtrig1, OUTPUT);
  pinMode(bottomecho1, INPUT);
  pinMode(bottomtrig2, OUTPUT);
  pinMode(bottomecho2, INPUT);

  //Motor Setup
  front_left.attach(13);  // make sure to use a PWM capable pin 
  back_left.attach(12);  // make sure to use a PWM capable pin 
  front_right.attach(11);  // make sure to use a PWM capable pin 
  back_right.attach(10);  // make sure to use a PWM capable pin  

  //start off
  MotorState = false;


  //bumper switch setup
  pinMode (Bumper1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Bumper1),BumperRead,RISING);
  pinMode (Bumper2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Bumper2),BumperRead,RISING);
  pinMode (Bumper3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Bumper3),BumperRead,RISING);
  pinMode (Bumper3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Bumper4),BumperRead,RISING);
  pinMode (Bumper4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Bumper5),BumperRead,RISING);
  pinMode (Bumper5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Bumper6),BumperRead,RISING);
  pinMode (Bumper6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Bumper6),BumperRead,RISING);

    //adjust servos(camera)
  servo1.write(140);
  servo2.write(90); 
}

void loop() {
  //sonar readings
  SonarSensor(T1trig, T1echo);
  T1 = distance;
  SonarSensor(T2trig, T2echo);
  T2 = distance;
  SonarSensor(T3trig, T3echo);
  T3 = distance;
  SonarSensor(T4trig, T4echo);
  T4 = distance;
  SonarSensor(T5trig, T5echo);
  T5 = distance;
  SonarSensor(T6trig, T6echo);
  T6 = distance;
  SonarSensor(bottomtrig1, bottomecho1);
  LeftSensor1 = distance;
  SonarSensor(bottomtrig2, bottomecho2);
  LeftSensor2 = distance;
  

  //used to stop the robot when qr code is scanned(DONE is sent by Pi)
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if(data=="DONE"){
      //stop robot
      MotorState=false;
    }
  }
  
  //check button state
  ButtonRead();
  
  //print sonar data
  Sonar_Debug();
  
  //move robot
  Movement();
  
  //check bumper state
  BumperRead();

}

void SonarSensor(int trigPin,int echoPin)
{
  delay(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.034 / 2)/2.54;
}

void Movement(){
  //corner right
  if((T1 <=11) ||(T2 <=11)||(T3 <=11)||(T4 <=11)||(T5 <=5)||(T6 <=5)){
    MODE = 'B';
    LEDControl();
    if(MotorState==true){
      set_esc_power(front_left, 40+6);
      set_esc_power(back_left, 40+6);
      set_esc_power(front_right, 40);
      set_esc_power(back_right, 40);
    }
    else{
      set_esc_power(front_left, 0);
      set_esc_power(back_left, 0);
      set_esc_power(front_right, 0);
      set_esc_power(back_right, 0);
    }
  }
    //corner left
  else if((LeftSensor1 > 13)){
    MODE = 'Y';
    LEDControl();
    if(MotorState==true){
      set_esc_power(front_left, 0);
      set_esc_power(back_left, 0);
      set_esc_power(front_right, -speed);
      set_esc_power(back_right, -speed);
    }
    else{
      set_esc_power(front_left, 0);
      set_esc_power(back_left, 0);
      set_esc_power(front_right, 0);
      set_esc_power(back_right, 0);
    }    
  }

  
  //backend adjustment
  //regular right
  else if((LeftSensor1 < 7)){
    //change to another color after
    MODE = 'R';
    LEDControl();
    if(MotorState==true){  
      set_esc_power(front_left, speed+6);
      set_esc_power(back_left, -speed+5);
      set_esc_power(front_right, speed);
      set_esc_power(back_right, -speed);
    }
    else{
      set_esc_power(front_left, 0);
      set_esc_power(back_left, 0);
      set_esc_power(front_right, 0);
      set_esc_power(back_right, 0);
    }
  }
  //regular left
  else if((LeftSensor1 > 11)){
    MODE = 'G';
    LEDControl();
    if(MotorState==true){
      set_esc_power(front_left, -speed+5);
      set_esc_power(back_left, speed+6);
      set_esc_power(front_right, -speed);
      set_esc_power(back_right, speed);
    }
    else{
      set_esc_power(front_left, 0);
      set_esc_power(back_left, 0);
      set_esc_power(front_right, 0);
      set_esc_power(back_right, 0);
    }
  }
  //straight
  else{
    MODE = 'W';
    LEDControl();  
    if(MotorState==true){
      //back too close wall
      if((LeftSensor2<LeftSensor1) && (LeftSensor2>7 && LeftSensor2<9)){         
          //set_esc_power(front_left, -speed+5);
          set_esc_power(front_left, speed+6);
          set_esc_power(back_left, speed+6);
          set_esc_power(front_right, -30);
          set_esc_power(back_right, -speed);
      }
      //back too far from wall
       else if((LeftSensor2>LeftSensor1)&& (LeftSensor2>10 && LeftSensor2<11)){          
          set_esc_power(front_left, speed+6);
          set_esc_power(back_left, 40);
          set_esc_power(front_right, -speed);
          set_esc_power(back_right, -speed);
      }
      //forward
      else{
          set_esc_power(front_left, speed+6);
          set_esc_power(back_left, speed+6);
          set_esc_power(front_right, -speed);
          set_esc_power(back_right, -speed);
      }
    }
    else{
      set_esc_power(front_left, 0);
      set_esc_power(back_left, 0);
      set_esc_power(front_right, 0);
      set_esc_power(back_right, 0);
    }
  }
}

void Sonar_Debug(){

  Serial.print("Left1: ");
  Serial.print(LeftSensor1);
  Serial.print(" in."); 
  Serial.print(" -        ");

  Serial.print("Left2: ");
  Serial.print(LeftSensor2);
  Serial.print(" in."); 
  Serial.print(" -        ");


  Serial.print("T1: ");
  Serial.print(T1);
  Serial.print(" in.");
  Serial.print(" -        ");

  Serial.print("T2: ");
  Serial.print(T2);
  Serial.print(" in.");
    Serial.print(" -        ");

  
  Serial.print("T3: ");
  Serial.print(T3);
  Serial.print(" in.");
  Serial.print(" -        ");

  Serial.print("T4: ");
  Serial.print(T4);
  Serial.print(" in.");
  Serial.print(" -        ");

  Serial.print("T5: ");
  Serial.print(T5);
  Serial.print(" in.");
  Serial.print(" -        ");

  Serial.print("T6: ");
  Serial.print(T6);
  Serial.println(" in.");
  

}

void LEDControl(){
  switch (MODE){
    case 'W':
      digitalWrite(RED,HIGH);
      digitalWrite(GREEN,HIGH);
      digitalWrite(BLUE,HIGH);
      break;
    case 'R':
      digitalWrite(RED,HIGH);
      digitalWrite(GREEN,LOW);
      digitalWrite(BLUE,LOW);
      break;
    case 'G':
      digitalWrite(RED,LOW);
      digitalWrite(GREEN,HIGH);
      digitalWrite(BLUE,LOW);
      break;
    case 'B':
      digitalWrite(RED,LOW);
      digitalWrite(GREEN,LOW);
      digitalWrite(BLUE,HIGH);
      break;
    case 'Y':
      digitalWrite(RED,HIGH);
      digitalWrite(GREEN,HIGH);
      digitalWrite(BLUE,LOW);
      break;
    case 'P':
      digitalWrite(RED,HIGH);
      digitalWrite(GREEN,HIGH);
      digitalWrite(BLUE,HIGH);
      break;
  }
}

void ButtonRead(){
  lastButtonState    = currentButtonState;      // save the last state
  currentButtonState = digitalRead(Button); // read new state

  if(lastButtonState == HIGH && currentButtonState == LOW) {
    if(MotorState == false){
      MotorState = true;
      }
    else{
      MotorState = false;
      set_esc_power(front_left, 0);  
      set_esc_power(back_left, 0);
      set_esc_power(front_right, 0);
      set_esc_power(back_right, 0);
    }
  }
}

void BumperRead(){
  //checking bumper state
  Bumper1State = digitalRead(Bumper1);
  Bumper2State = digitalRead(Bumper2);
  Bumper3State = digitalRead(Bumper3);
  Bumper4State = digitalRead(Bumper4);
  Bumper5State = digitalRead(Bumper5);
  Bumper6State = digitalRead(Bumper6);
  
  //hit so stop robot
  if((Bumper1State == LOW) ||(Bumper2State == LOW)||(Bumper3State == LOW)||(Bumper4State == LOW)||(Bumper5State == LOW)||(Bumper6State == LOW)){
    MotorState=false;
  }
}

//motor control function
void set_esc_power (Servo esc, int power){
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
}
