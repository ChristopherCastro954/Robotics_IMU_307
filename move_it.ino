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
//Front
#define trigPin1 9
#define echoPin1 10

//Left
#define trigPin2 11
#define echoPin2 12

//Motor Driver Pins Left
#define PWML 2 
#define DIRL 3

//Motor Driver Pins Right
#define PWMR 4
#define DIRR 5


//Global Variables
float follow_distance = 7; //Unit in inches
float turn_range = 13;
float FrontSensor,LeftSensor;
float duration, distance;
char MODE;
int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button
volatile bool MotorState;

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
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  //Motor Setup
  pinMode(PWML,OUTPUT);
  pinMode(PWMR,OUTPUT);
  pinMode(DIRL,OUTPUT);
  pinMode(DIRR,OUTPUT);

  //DIR Setup
  digitalWrite(DIRL,LOW);
  digitalWrite(DIRR,LOW);
  MotorState = false;
}

void loop() {
  SonarSensor(trigPin1, echoPin1);
  FrontSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  LeftSensor = distance;

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

}

void SonarSensor(int trigPin,int echoPin)
{
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
  if((FrontSensor <=14)){ //Left /////&& (FrontSensor > 6)
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
    delay(1900);
  }
  
  //regular right
  else if((LeftSensor < 7)){
    MODE = 'R';
    LEDControl();
    if(MotorState==true){
      analogWrite(PWMR,0);
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
    if(MotorState==true){/*
      analogWrite(PWMR,R_val);
      analogWrite(PWML,L_val);
      delay(100);
      analogWrite(PWMR,R_val);
      analogWrite(PWML,0);
      delay(200);*/
      analogWrite(PWMR,R_val);
      analogWrite(PWML,L_val/2);
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
      analogWrite(PWML,0);
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
  /*Serial.print("Front: ");
  Serial.print(FrontSensor);
  Serial.print(" in.");
  Serial.print(" - ");
  Serial.print("Left: ");
  Serial.print(LeftSensor);
  Serial.println(" in."); */
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
/*
void Movement(){
  //For Clock wise motion , I1 = HIGH , I2 = LOW
  if((LeftSensor > 6) && (LeftSensor < 9) && (FrontSensor > 13)){ //Straight
    if(MotorState==true){
      analogWrite(PWMR,R_val);
      analogWrite(PWML,L_val);
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    Serial.print("Striaght  ");
    MODE = 'W';    
  }
  else if((FrontSensor <= 13)){ //Right ////(LeftSensor > 6) && (LeftSensor < 9) && 
    if(MotorState==true){
      analogWrite(PWMR,0);
      analogWrite(PWML,L_val);
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    Serial.print("Right1    ");
    MODE = 'R';
  }
  else if((LeftSensor >= 9) && (FrontSensor > 6) && (FrontSensor <13)){ //Left /////&& (FrontSensor > 6)
    if(MotorState==true){
      analogWrite(PWMR,R_val);
      analogWrite(PWML,0);
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    Serial.print("Left      ");
    MODE = 'G';
  }
  else{ //Also Right
    if(MotorState==true){
      analogWrite(PWMR,0);
      analogWrite(PWML,L_val);
    }
    else{
      analogWrite(PWMR,0);
      analogWrite(PWML,0);
    }
    Serial.print("Right2    ");
    MODE = 'B';
  }
}*/
