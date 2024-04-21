#include <SparkFun_TB6612.h>
#include <TFMPlus.h>
#include <Wire.h>
#include <TFLI2C.h>

TFLI2C tflI2C;
int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;

// Define pins for reflectance sensors
const int leftReflectancePin = A0;  // Left reflectance sensor pin
const int rightReflectancePin = A1; // Right reflectance sensor pin

// Define pins for motor control
const int leftMotorPWM = 3;  // PWM pin for left motor speed control
const int leftMotorDir1 = 4; // Direction pin 1 for left motor
const int leftMotorDir2 = 2; // Direction pin 2 for left motor

const int rightMotorPWM = 9;  // PWM pin for right motor speed control
const int rightMotorDir1 = 6; // Direction pin 1 for right motor
const int rightMotorDir2 = 7; // Direction pin 2 for right motor

const int standbyPin = 5; // Standby pin for motor controller

// Define motor speed
const int moderateSpeed = 200; // Moderate motor speed
const int maxSpeed = 255;      // Maximum motor speed
// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;
Motor motor_left = Motor(leftMotorDir1, leftMotorDir2, leftMotorPWM, offsetA, standbyPin);
Motor motor_right = Motor(rightMotorDir1, rightMotorDir2, rightMotorPWM, offsetB, standbyPin);

//Define sensor thresholds
const int leftLineThreshold = 200;      // Reflectance sensor threshold
const int rightLineThreshold = 200;      // Reflectance sensor threshold

const int distThresh = 30;

void setup()
{
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  pinMode(standbyPin, OUTPUT);

  pinMode(leftReflectancePin, INPUT);
  pinMode(rightReflectancePin, INPUT);

  Wire.begin();

  // Enable motor controller
  digitalWrite(standbyPin, HIGH);

  Serial.print("LIDAR Reset: ");
    if( tflI2C.Soft_Reset( tfAddr))
    {
        Serial.println( "Passed");
    }
    else tflI2C.printStatus();

  Serial.print( "Set Frame Rate to: ");
  uint16_t tfFrame = FPS_250;
  if( tflI2C.Set_Frame_Rate( tfFrame, tfAddr))
  {
    Serial.println( tfFrame);
  }
  else tflI2C.printStatus();

  delay(5000);
}

void handleLine() {
  motor_right.drive(-255);
  motor_left.drive(-255);
  delay(100);
  motor_right.brake();
  motor_left.brake();
}

void loop()
{
  if (checkLine() == 1) {
    Serial.println("STOP");
    handleLine();
  }
  
  motor_right.drive(150);

  if (tflI2C.getData( tfDist, tfAddr) && tfDist < distThresh) {
          //      Serial.print("Dist: ");
  //      Serial.println(tfDist);
    Serial.println("BANG");
    motor_right.drive(255);
    motor_left.drive(255);
    while (1) {
      bool mustStop = false;
      bool needs_180 = false;
      if (tflI2C.getData( tfDist, tfAddr) && tfDist > distThresh) {
          mustStop = true;
       } else if (checkLine() == 1) {
          mustStop = true;
          needs_180 = true;
       }
       
       if (mustStop) {
        Serial.println("STOP");
        motor_right.brake();
        motor_left.brake();
        if (needs_180) {
          handleLine();
        }
        break;
      }
    } 
  }

  delay(5);
}

void printReflectance() {
  int leftReflectance = analogRead(leftReflectancePin);
  int rightReflectance = analogRead(rightReflectancePin);
  Serial.print("left: ");
  Serial.print(leftReflectance);
  Serial.print(" right: ");
  Serial.print(rightReflectance);
  Serial.println();
}

// TODO: return diff failure modes
int loopAndCheckLine(unsigned long delayMillis) {
  unsigned long start = millis();
  while (millis() - start < delayMillis) {
    if (checkLine != 0) {
      return 1;
    }
  }
  return 0;
}

int checkLine() {
  int leftReflectance = analogRead(leftReflectancePin);
  int rightReflectance = analogRead(rightReflectancePin);
  if (leftReflectance < leftLineThreshold) {
//    Serial.print("LEFT ");
//    Serial.println(leftReflectance);
    return 1;
  }
  if (rightReflectance < rightLineThreshold) {
//    Serial.print("RIGHT ");
//    Serial.println(rightReflectance);
    return 1;
  }
  return 0;
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
