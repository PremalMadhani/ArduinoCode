#include <Servo.h>

#define PIN_Motor_STBY 3
#define PIN_Motor_AIN1 7
#define PIN_Motor_PWMA 5 // Left motors
#define PIN_Motor_BIN1 8
#define PIN_Motor_PWMB 6 // Right motors

#define TRIG_PIN 13 //A5
#define ECHO_PIN 12 //A4

Servo myServo;
int distanceThresh = 25; // Stop if object is closer than 25cm
unsigned int tempda_x = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_AIN1, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_BIN1, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  
  digitalWrite(PIN_Motor_STBY, HIGH); // Must be HIGH to enable motors
  
  myservo.attach(10); // Servo pin on V4.0 is usually 10
  myservo.write(90);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  int distance = getDistance();
  
  if (distance > distanceThresh) {
    moveForward(150);
  } else {
    stopCar();
    delay(200);
    moveBackward(150); // Move back to give turning room
    delay(500);
    stopCar();
    
    // Choose the best direction
    chooseBestPath();
  }
}

void chooseBestPath() {
  myServo.write(140); // Look Left
  delay(500);
  int leftDist = getDistance();
  
  myServo.write(40);  // Look Right
  delay(500);
  int rightDist = getDistance();
  
  myServo.write(90);  // Reset servo to center
  delay(200);

  if (leftDist > rightDist) {
    turnLeft(180);
    delay(600); // Adjust delay for a ~90 degree turn
  } else {
    turnRight(180);
    delay(600);
  }
  stopCar();
}

// Support Functions
long getDistance() {
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = (unsigned int)pulseIn(ECHO_PIN, HIGH)/58;
  return tempda_x; // Distance in cm
}

void moveForward(int speed) {
  digitalWrite(PIN_Motor_AIN1, HIGH);
  analogWrite(PIN_Motor_PWMA, speed);
  digitalWrite(PIN_Motor_BIN1, HIGH);
  analogWrite(PIN_Motor_PWMB, speed);
}

void turnLeft(int speed) {
  digitalWrite(PIN_Motor_AIN1, HIGH); digitalWrite(PIN_Motor_BIN1, LOW);
  analogWrite(PWMA, speed); analogWrite(PWMB, speed);
}

void turnRight(int speed) {
  digitalWrite(PIN_Motor_AIN1, LOW); digitalWrite(PIN_Motor_BIN1, HIGH);
  analogWrite(PWMA, speed); analogWrite(PWMB, speed);
}

void stopCar() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}
