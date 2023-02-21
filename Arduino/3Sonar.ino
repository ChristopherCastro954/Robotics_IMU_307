//Sonar PINs
//TLeft
#define trigPin1 2
#define echoPin1 3

//TMiddle
#define trigPin2 4
#define echoPin2 5

//TRight
#define trigPin3 6
#define echoPin3 7

//Global Variables
float TRightSensor,TMiddleSensor,TLeftSensor;
float duration, distance;

void setup() {
  Serial.begin (9600);

  //Sonar Setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
}

void loop() {
  SonarSensor(trigPin1, echoPin1);
  TLeftSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  TMiddleSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  TRightSensor = distance;
  Sonar_Debug();
  delay(1000);
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

void Sonar_Debug(){
  Serial.print("Right: ");
  Serial.print(TRightSensor);
  Serial.print(" in.");
  Serial.print(" - ");
  Serial.print("Middle: ");
  Serial.print(TMiddleSensor);
  Serial.print(" in.");
  Serial.print(" - ");
  Serial.print("Left: ");
  Serial.print(TLeftSensor);
  Serial.println(" in.");
}