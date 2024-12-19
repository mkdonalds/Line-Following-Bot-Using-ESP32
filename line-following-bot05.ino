#include <Arduino.h>
#include <PID_v1.h>

// Motor Driver Pins
#define IN1 25   // Left Motor Forward
#define IN2 33   // Left Motor Backward
#define IN3 32   // Right Motor Forward
#define IN4 4    // Right Motor Backward
#define ENA 27   // Left Motor Speed Control
#define ENB 26   // Right Motor Speed Control

// IR Sensor Pins
#define SENSOR_LEFT 23   // Left Line Sensor
#define SENSOR_RIGHT 15  // Right Line Sensor

// PID Variables
double setpoint = 0;     // Desired position (center of the line)
double input = 0;        // Current position
double output = 0;       // PID correction

// PID Tuning Parameters
double Kp = 30.0;   // Proportional gain
double Ki = 2;      // Integral gain
double Kd = 6;      // Derivative gain

// Create PID instance
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Base Motor Speeds
const int BASE_SPEED = 85;  // Adjust based on your motor characteristics
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;

void setup() {
  // Motor Driver Pins Setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor Pins Setup
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  // Initialize Serial for debugging
  Serial.begin(9600);

  // PID Setup
  myPID.SetMode(AUTOMATIC);         // Turn on PID
  myPID.SetOutputLimits(-100, 100); // Limit PID output
  myPID.SetSampleTime(10);          // Update every 10ms
}

void drive(int leftSpeed, int rightSpeed) {
  // Left Motor Control
  if (leftSpeed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, leftSpeed);
  } else if (leftSpeed == 0) {
    // Stop left motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(leftSpeed));
  }

  // Right Motor Control
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else if (rightSpeed == 0) {
    // Stop right motor
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(rightSpeed));
  }

  // Debug motor speeds
  Serial.print("Motor Speeds - Left: ");
  Serial.print(leftSpeed);
  Serial.print(" Right: ");
  Serial.println(rightSpeed);
}

void loop() {
  // Read sensor values
  int leftSensor = digitalRead(SENSOR_LEFT);
  int rightSensor = digitalRead(SENSOR_RIGHT);

  // Debug print sensor states
  Serial.print("Left Sensor: ");
  Serial.print(leftSensor);
  Serial.print(" Right Sensor: ");
  Serial.println(rightSensor);

  // Compute line position and motor speeds
  if (leftSensor == HIGH && rightSensor == HIGH) {
    // Both sensors on white - continue forward
    input = 0;   
    int leftSpeed = BASE_SPEED;
    int rightSpeed = BASE_SPEED;
    drive(leftSpeed, rightSpeed);
    Serial.println("On Line");
  } 
  else if (rightSensor == HIGH) {
    // Right sensor on black - stop right motor to turn right
    input = 1;  
    int leftSpeed = BASE_SPEED;
    int rightSpeed = 0;  // Stop right motor
    drive(leftSpeed, rightSpeed);
    Serial.println("Turning Right");
  } 
  else if (leftSensor == HIGH) {
    // Left sensor on black - stop left motor to turn left
    input = -1;  
    int leftSpeed = 0;  // Stop left motor
    int rightSpeed = BASE_SPEED;
    drive(leftSpeed, rightSpeed);
    Serial.println("Turning Left");
  }
  else {
    // Both sensors on black - stop both motors
    drive(0, 0);
    Serial.println("Both Sensors Black - Stopped");
  }

  // Compute PID (though less critical with direct motor stopping)
  myPID.Compute();

  // Small delay for stability and serial print readability
  delay(100);
}