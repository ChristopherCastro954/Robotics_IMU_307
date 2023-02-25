#include <Servo.h>
Servo servo1,servo2;
//Servo Pins
int servoPin1= 6; //Top
int servoPin2= 7; //Bottom

//LED PINs
#define RED 48
#define GREEN 50
#define BLUE 52

#define Button 18

//Sonar PINs

//looking at it from left to right (you being in front of robot)
#define T1trig 46
#define T1echo 47

#define T2trig 44
#define T2echo 45

#define T3trig 34
#define T3echo 35

#define T4trig 32
#define T4echo 33

#define T5trig 42
#define T5echo 43

#define T6trig 30
#define T6echo 31

//Bottom sonar
#define bottomtrig 11
#define bottomecho 12

//Motor Driver Pins Left
#define PWML 2 
#define DIRL 3

//Motor Driver Pins Right
#define PWMR 4
#define DIRR 5

//bumpers
#define Bumper1 40
#define Bumper2 41
#define Bumper3 38
#define Bumper4 39
#define Bumper5 36
#define Bumper6 37


//Global Variables
float follow_distance = 7; //Unit in inches
float turn_range = 13;
float FrontSensor,LeftSensor;
float T1,T2,T3,T4,T5,T6;
float duration, distance;
char MODE;
int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button
volatile bool MotorState;
// bumper state variables
int Bumper1State,Bumper2State,Bumper3State,Bumper4State,Bumper5State,Bumper6State; 

//PWM ints
int R_val = 240;
int L_val = 240;

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
  pinMode(bottomtrig, OUTPUT);
  pinMode(bottomecho, INPUT);

  //Motor Setup
  pinMode(PWML,OUTPUT);
  pinMode(PWMR,OUTPUT);
  pinMode(DIRL,OUTPUT);
  pinMode(DIRR,OUTPUT);

  //DIR Setup
  digitalWrite(DIRL,LOW);
  digitalWrite(DIRR,LOW);
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
}

void loop() {
  //delay(50);
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
  SonarSensor(bottomtrig, bottomecho);
  LeftSensor = distance;

  Average();

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if(data=="DONE"){
      MotorState=false;
    }
  }
  servo1.write(130);
  servo2.write(90); 
  ButtonRead();
  Sonar_Debug();
  Movement();

  BumperRead();

}

void SonarSensor(int trigPin,int echoPin)
{
  delay(1);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.034 / 2)/2.54;
}

void Movement(){
  //For Clock wise motion , I1 = HIGH , I2 = LOW
    //corner turn right
  if((T1 <=14) ||(T2 <=14)||(T3 <=14)||(T4 <=14)||(T5 <=14)||(T6 <=14)){ //Left /////&& (FrontSensor > 6)
    MODE = 'B';
    LEDControl();
    if(MotorState==true){
      analogWrite(PWMR,0);
      analogWrite(PWML,L_val);
      
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    //Serial.print(" Corner Right    ");
    //delay(100);
  }
  
  //regular right
  else if((LeftSensor < 7)){
    MODE = 'R';
    LEDControl();
    if(MotorState==true){
      analogWrite(PWMR,40);
      analogWrite(PWML,L_val);
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    //Serial.print("Right1    ");
  }
  //corner left
  else if((LeftSensor > 13)){
    MODE = 'Y';
    LEDControl();
    if(MotorState==true){
      analogWrite(PWMR,R_val);
      analogWrite(PWML,L_val*.39);
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    //Serial.print("Left      ");
    
  }
  //regular left
  else if((LeftSensor > 9)){
    MODE = 'G';
    LEDControl();
    if(MotorState==true){
      analogWrite(PWMR,R_val);
      analogWrite(PWML,(L_val*.60));
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    //Serial.print("Left      ");
  }

  //go straight
  else{ //Straight
    MODE = 'W';
    LEDControl();  
    if(MotorState==true){
      analogWrite(PWMR,R_val);
      analogWrite(PWML,L_val);
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    //Serial.print("Striaght  ");
  }
  
}

void Sonar_Debug(){

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
  Serial.print(" in.");
  Serial.print(" -        ");
  
  Serial.print("Left Side: ");
  Serial.print(LeftSensor);
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
      //delay(2000);
      break;
    case 'Y':
      digitalWrite(RED,HIGH);
      digitalWrite(GREEN,HIGH);
      digitalWrite(BLUE,LOW);
      break;
  }
}

void ButtonRead(){
  lastButtonState    = currentButtonState;      // save the last state
  currentButtonState = digitalRead(Button); // read new state

  if(lastButtonState == HIGH && currentButtonState == LOW) {
    if(MotorState == false){
      MotorState = true;
      //delay(500);
      }
    else{
      MotorState = false;
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
      //delay(1000);
    }
  }
}

void Average(){
  FrontSensor=(T1+T2+T3+T4+T5+T6)/3;
}

void BumperRead(){
  //checking bumper state
  Bumper1State = digitalRead(Bumper1);
  Bumper2State = digitalRead(Bumper2);
  Bumper3State = digitalRead(Bumper3);
  Bumper4State = digitalRead(Bumper4);
  Bumper5State = digitalRead(Bumper5);
  Bumper6State = digitalRead(Bumper6);

      if((Bumper1State == LOW) ||(Bumper2State == LOW)||(Bumper3State == LOW)||(Bumper4State == LOW)||(Bumper5State == LOW)||(Bumper6State == LOW)){
        MotorState=false;
      }
}
