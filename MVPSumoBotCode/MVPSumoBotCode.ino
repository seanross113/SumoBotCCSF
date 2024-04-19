#include <SparkFun_TB6612.h>

// Define pins for reflectance sensors
const int leftReflectancePin = A0;  // Left reflectance sensor pin
const int rightReflectancePin = A1; // Right reflectance sensor pin

// Define pins for motor control
// Pinout: https://docs.arduino.cc/tutorials/nano-esp32/pin-setup/
// int ledChannelLeft = 0;
// int ledChannelRight = 1;

// const int leftMotorPWM = 5;      // PWM pin for left motor speed control
// const int leftMotorDir1 = 6;    // Direction pin 1 for left motor
// const int leftMotorDir2 = 7;    // Direction pin 2 for left motor

const int leftMotorPWM = 3;  // PWM pin for left motor speed control
const int leftMotorDir1 = 4; // Direction pin 1 for left motor
const int leftMotorDir2 = 2; // Direction pin 2 for left motor

// const int rightMotorPWM = 17;     // PWM pin for right motor speed control
// const int rightMotorDir1 = 9;    // Direction pin 1 for right motor
// const int rightMotorDir2 = 10;    // Direction pin 2 for right motor

const int rightMotorPWM = 9;  // PWM pin for right motor speed control
const int rightMotorDir1 = 6; // Direction pin 1 for right motor
const int rightMotorDir2 = 7; // Direction pin 2 for right motor

// const int standbyPin = 8;       // Standby pin for motor controller
const int standbyPin = 5; // Standby pin for motor controller

// Define motor speed
const int moderateSpeed = 200; // Moderate motor speed
const int maxSpeed = 255;      // Maximum motor speed
// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(leftMotorDir1, leftMotorDir2, leftMotorPWM, offsetA, standbyPin);
Motor motor2 = Motor(rightMotorDir1, rightMotorDir2, rightMotorPWM, offsetB, standbyPin);

//Define sensor thresholds
const int lineThreshold = 1000;      // Reflectance sensor threshold

void setup()
{
  // Initialize pins
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  pinMode(standbyPin, OUTPUT);

  pinMode(leftReflectancePin, INPUT);
  pinMode(rightReflectancePin, INPUT);

  //  pinMode(D7, OUTPUT);
  //  pinMode(D6, OUTPUT);

  // Setup PWM channels
  //  ledcSetup(ledChannelLeft, 5000, 8);    // 5000 Hz frequency, 8-bit resolution
  //  ledcSetup(ledChannelRight, 5000, 8);   // 5000 Hz frequency, 8-bit resolution

  // Attach PWM channels to GPIO pins
  //  ledcAttachPin(ledChannelLeft, leftMotorPWM);
  //  ledcAttachPin(ledChannelRight, rightMotorPWM);

  // Enable motor controller
  digitalWrite(standbyPin, HIGH);

  Serial.begin(115200); // Initialize serial communication
}

void loop()
{
  Serial.println("yo3");
  int leftReflectance = analogRead(leftReflectancePin);
  int rightReflectance = analogRead(rightReflectancePin);
  Serial.println("Left Reflector value:");
  Serial.println(leftReflectance);
  Serial.println("Right Reflector value:");
  Serial.println(rightReflectance);
  /*
  motor1.drive(255, 1000);
  motor1.drive(-255, 1000);
  motor1.brake();
  motor2.drive(255, 1000);
  motor2.drive(-255, 1000);
  motor2.brake();
  // digitalWrite(leftMotorPWM, HIGH);
  //  digitalWrite(10, HIGH);
  //  digitalWrite(9, HIGH);
  */
  delay(1000);
}

/*
// Function to measure distance using ultrasonic sensor
//int getDistance() {
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  long duration = pulseIn(echoPin, HIGH);
//  int distance = duration * 0.034 / 2;  // Convert time to distance (cm)
//  return distance;
//}

// Function to move the robot forward
void moveForward(int speed) {
  analogWrite(leftMotorPWM, speed);
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(rightMotorPWM, speed);
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
}

// Function to turn the robot left
void moveLeft(int speed) {
  analogWrite(leftMotorPWM, 0);
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(rightMotorPWM, speed);
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
}

// Function to turn the robot right
void moveRight(int speed) {
  analogWrite(leftMotorPWM, speed);
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, LOW);
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(leftMotorPWM, 0);
  digitalWrite(rightMotorPWM, 0);
}
*/
