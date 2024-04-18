// Define pins for IR sensors
const int leftIRSensorPin = 6;   // Connect left IR sensor to digital pin 2
const int rightIRSensorPin = 7;  // Connect right IR sensor to digital pin 3

// Define pins for distance sensor (HC-SR04)
const int trigPin = 2;              // Trigger pin
const int echoPin = 4;              // Echo pin

// Define pins for reflectance sensors
const int leftReflectancePin = A1;  // Left reflectance sensor pin
const int rightReflectancePin = A0; // Right reflectance sensor pin

// Define pins for motor control
const int leftMotorPWM = 3;      // PWM pin for left motor speed control
const int leftMotorDir1 = 8;    // Direction pin 1 for left motor
const int leftMotorDir2 = 9;    // Direction pin 2 for left motor
const int rightMotorPWM = 5;     // PWM pin for right motor speed control
const int rightMotorDir1 = 11;    // Direction pin 1 for right motor
const int rightMotorDir2 = 12;    // Direction pin 2 for right motor
const int standbyPin = 10;       // Standby pin for motor controller


// Define motor speed
const int moderateSpeed = 200;      // Moderate motor speed
const int maxSpeed = 255;           // Maximum motor speed

// Define sensor thresholds
const int obstacleThreshold = 500;  // IR sensor threshold
const int obstacleDistanceThreshold = 20; // Minimum distance detected by ultrasonic sensor (adjust as needed)
const int lineThreshold = 1000;      // Reflectance sensor threshold

void setup() {
  // Initialize pins
  pinMode(leftIRSensorPin, INPUT);
  pinMode(rightIRSensorPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(leftReflectancePin, INPUT);
  pinMode(rightReflectancePin, INPUT);

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  pinMode(standbyPin, OUTPUT);

  // Enable motor controller
  digitalWrite(standbyPin, HIGH);

  Serial.begin(9600);
}

void loop() {
  // Read sensor states
  int leftIRSensorState = digitalRead(leftIRSensorPin);
  int rightIRSensorState = digitalRead(rightIRSensorPin);

  // Read distance from ultrasonic sensor
  int distance = getDistance();

  // Read reflectance sensor values
  int leftReflectance = analogRead(leftReflectancePin);
  Serial.println("Left Reflector value:");
  Serial.println(leftReflectance);
  int rightReflectance = analogRead(rightReflectancePin);
  Serial.println("Right Reflector value:");
  Serial.println(rightReflectance);

  // If both IR sensors detect an obstacle, move forward to push it
  if (leftIRSensorState == HIGH && rightIRSensorState == HIGH) {
    Serial.println("both IR sensors high");
    moveForward(maxSpeed);
  } 
  // If only left sensor detects an obstacle, turn right
  else if (leftIRSensorState == HIGH) {
    Serial.println("left IR sensor high");
    moveRight(moderateSpeed);
  } 
  // If only right sensor detects an obstacle, turn left
  else if (rightIRSensorState == HIGH) {
    Serial.println("right IR sensor high");
    moveLeft(moderateSpeed);
  } 
  // If obstacle detected by ultrasonic sensor, move forward to push it
  else if (distance <= obstacleDistanceThreshold) {
    Serial.println("obstacle detected");
    moveForward(maxSpeed);
  } 
  // If left reflectance sensor detects a light-colored line, turn left
  else if (leftReflectance >= lineThreshold) {
    Serial.println("left reflector triggered");
    moveLeft(moderateSpeed);
  } 
  // If right reflectance sensor detects a dark-colored line, turn right
  else if (rightReflectance >= lineThreshold) {
    Serial.println("right reflector triggered");
    moveRight(moderateSpeed);
  } 
  // If no obstacle or line detected, stop
  else {
    Serial.println("No obstacle detected");
    stopMotors();
  }
}

// Function to measure distance using ultrasonic sensor
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;  // Convert time to distance (cm)
  return distance;
}

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