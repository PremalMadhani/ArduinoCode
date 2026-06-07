#include <Servo.h>

// Correct V4.0 Pin Mapping for TB6612
#define PIN_Motor_STBY 3
#define PIN_Motor_AIN1 7
#define PIN_Motor_PWMA 5 // Left motors
#define PIN_Motor_BIN1 8
#define PIN_Motor_PWMB 6 // Right motors

#define TRIG_PIN 13 //A5
#define ECHO_PIN 12 //A4

Servo myservo;

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

void moveForward(int speed) {
  digitalWrite(PIN_Motor_AIN1, HIGH);
  analogWrite(PIN_Motor_PWMA, speed);
  digitalWrite(PIN_Motor_BIN1, HIGH);
  analogWrite(PIN_Motor_PWMB, speed);
}

void stopCar() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}

long readDistance() {
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = (unsigned int)pulseIn(ECHO_PIN, HIGH)/58;
  return tempda_x; // Distance in cm
}

void loop() {
  int distance = readDistance();
  Serial.println(distance);
  if (distance > 30 || distance == 0) 
  { // 0 often means "out of range"
    moveForward(150);
  } 
  else 
  {
    stopCar();
    delay(500);
    // Simple turn to avoid
    digitalWrite(PIN_Motor_AIN1, LOW); 
    analogWrite(PIN_Motor_PWMA, 150);
    delay(500);
  }
  delay(50);
}