#include <Wire.h>     // Include Wire library for I2C communication
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Optical_Flow_Sensor.h"

//optical flow sensor for drift
Optical_Flow_Sensor flow(53, PAA5100);
int16_t totalX = 0, totalY = 0;
float error_drift = 0.0;
float kp_drift = 0.5;
float ki_drift = 0.001;
float kd_drift = 0.1;
float total_drift = 0.0;
float prev_error_drift = 0.0;

// CHANGE HERE"
const char* paths[] = {"START", "FORWARD", "FORWARD", "FORWARD", "STOP"};
int index = 0;
float target_time = 30.0;

//time
float time_per_step = 0.0;
float start_time = 0.0;
float goal_time = 0.0;
float kp_time = .01;
// float kp_time = 0;
float left_over_time = 0.0;
float error_time = 0.0;

// angle
float kp = 4.0;
float ki = 0.1;
float kd = 1.0;
float total = 0.0;
volatile float error = 0.0;
volatile float prev_error = 0.0;

// state machine
bool started_new_action = true;
int length = 48;
volatile unsigned long prev_time = 0;

// Optical Encoder Pins
const int backLeftEncoder = 2;
const int frontLeftEncoder = 3;
const int frontRightEncoder = 18;
const int backRightEncoder = 19;

bool backLeftEncoderBool = false;
bool frontLeftEncoderBool = false;
bool frontRightEncoderBool = false;
bool backRightEncoderBool = false;

// Counters
volatile long backLeftEncoderCount = 0;
volatile long frontLeftEncoderCount = 0;
volatile long frontRightEncoderCount = 0;
volatile long backRightEncoderCount = 0;

// PWM speed to motor
int pwmBackRight = 100;
int pwmBackLeft = 100;
int pwmFrontRight = 100;
int pwmFrontLeft = 100;

int baseSpeed = 165;

int backRightMillis = 0;
int backLeftMillis = 0;
int frontRightMillis = 0;
int frontLeftMillis = 0;

// PWM Motor Control Pins
const int pwmBackRightMotor = 4;
const int pwmBackLeftMotor = 5;
const int pwmFrontRightMotor = 6;
const int pwmFrontLeftMotor = 7;

// Input Control Pins for Motors
const int input2BackRight = 22;
const int input1BackRight = 23;
const int input1BackLeft = 24;
const int stbyBackMotors = 25;
const int input2BackLeft = 26;
const int input1FrontRight = 27;
const int input2FrontRight = 28;
const int stbyFrontMotors = 29;
const int input1FrontLeft = 30;
const int input2FrontLeft = 31;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


void printCounters() {
  Serial.print("Index: ");
  Serial.print(index);
  Serial.print(" State: ");
  Serial.print(paths[index]);
  Serial.print(" Back Left: ");
  Serial.print(backLeftEncoderCount);
  Serial.print(" Front Left: ");
  Serial.print(frontLeftEncoderCount);
  Serial.print(" Back Right: ");
  Serial.print(backRightEncoderCount);
  Serial.print(" Front Right: ");
  Serial.println(frontRightEncoderCount);
}


// Updates back left encoder
void backLeftEncoderAdd() {
  backLeftEncoderCount++;
}


// Updates front left encoder
void frontLeftEncoderAdd() {
  frontLeftEncoderCount++;
}


// Updates front right encoder
void frontRightEncoderAdd() {
  frontRightEncoderCount++;
}


// Updates back right encoder
void backRightEncoderAdd() {
  backRightEncoderCount++;
}


void updateCounters() {
  if (backLeftEncoderBool == false && digitalRead(backLeftEncoder)) {  // RISING
    backLeftEncoderBool = true;
    backLeftEncoderAdd();
  } else if (backLeftEncoderBool == true && !digitalRead(backLeftEncoder)) {  // FALLING
    backLeftEncoderBool = false;
  }


  if (frontLeftEncoderBool == false && digitalRead(frontLeftEncoder)) {  // RISING
    frontLeftEncoderBool = true;
    frontLeftEncoderAdd();
  } else if (frontLeftEncoderBool == true && !digitalRead(frontLeftEncoder)) {  // FALLING
    frontLeftEncoderBool = false;
  }


  if (frontRightEncoderBool == false && digitalRead(frontRightEncoder)) {  // RISING
    frontRightEncoderBool = true;
    frontRightEncoderAdd();
  } else if (frontRightEncoderBool == true && !digitalRead(frontRightEncoder)) {  // FALLING
    frontRightEncoderBool = false;
  }


  if (backRightEncoderBool == false && digitalRead(backRightEncoder)) {  // RISING
    backRightEncoderBool = true;
    backRightEncoderAdd();
  } else if (backRightEncoderBool == true && !digitalRead(backRightEncoder)) {  // FALLING
    backRightEncoderBool = false;
  }
}


void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);


  // Setup encoder pins
  pinMode(backLeftEncoder, INPUT_PULLUP);
  pinMode(frontLeftEncoder, INPUT_PULLUP);
  pinMode(frontRightEncoder, INPUT_PULLUP);
  pinMode(backRightEncoder, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(backLeftEncoder), backLeftEncoderAdd, RISING);
  attachInterrupt(digitalPinToInterrupt(frontLeftEncoder), frontLeftEncoderAdd, RISING);
  attachInterrupt(digitalPinToInterrupt(backRightEncoder), backRightEncoderAdd, RISING);
  attachInterrupt(digitalPinToInterrupt(frontRightEncoder), frontRightEncoderAdd, RISING);


  // Setup motor PWM pins
  pinMode(pwmBackRightMotor, OUTPUT);
  pinMode(pwmBackLeftMotor, OUTPUT);
  pinMode(pwmFrontRightMotor, OUTPUT);
  pinMode(pwmFrontLeftMotor, OUTPUT);


  // Setup motor control pins
  pinMode(input2BackRight, OUTPUT);
  pinMode(input1BackRight, OUTPUT);
  pinMode(input1BackLeft, OUTPUT);
  pinMode(stbyBackMotors, OUTPUT);
  pinMode(input2BackLeft, OUTPUT);
  pinMode(input1FrontRight, OUTPUT);
  pinMode(input2FrontRight, OUTPUT);
  pinMode(stbyFrontMotors, OUTPUT);
  pinMode(input1FrontLeft, OUTPUT);
  pinMode(input2FrontLeft, OUTPUT);


  // Initialize I2C for MPU6050
  Wire.begin();
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }

  // calculate time per step
  float num_steps = 0.0;
  for (int x = 0; x<sizeof(paths)/sizeof(paths[0]); x++){
    if (paths[x]=="FORWARD" || paths[x]=="BACKWARD" || paths[x]=="LEFT" || paths[x]=="RIGHT"){
      num_steps += 1;
    }else if (paths[x]=="START"){
      num_steps += 0.5;
    }
  }
  time_per_step = target_time / num_steps;


  // Ensure motors are off by default
  digitalWrite(stbyBackMotors, LOW);
  digitalWrite(stbyFrontMotors, LOW);
}


int cmToTicks(int cm) {
  return (int)(cm * 20 / (6.5 * 3.14159));
}


void resetCounters() {
  backLeftEncoderCount = 0;
  frontLeftEncoderCount = 0;
  frontRightEncoderCount = 0;
  backRightEncoderCount = 0;
  totalY = 0;
  totalX = 0;
  total_drift = 0;
}


float calculateError(float angle){
  if (angle < 180){
    return angle;
  }else{
    return angle-360;
  }
}


int medianTick(int backLeftEncoderCount, int frontLeftEncoderCount, int frontRightEncoderCount, int backRightEncoderCount) {
  // Store the counts in an array for easier sorting
  int counts[4] = {backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount};
 
  // Sort the array of counts
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (counts[i] > counts[j]) {
        int temp = counts[i];
        counts[i] = counts[j];
        counts[j] = temp;
      }
    }
  }


  // Calculate the median as the average of the two middle values
  float median = (counts[1] + counts[2]) / 2.0;
  return median;

  goal_time = millis();
}


void adjustMotors() {
  unsigned long curr_time = millis();
  float dt = curr_time - prev_time;
  prev_time = curr_time;
  float derivative = (error-prev_error) / dt;
  prev_error = error;
  float derivative_drift = (error_drift - prev_error_drift)/dt;
  prev_error_drift = error_drift;
  total += error*dt;
  total_drift += error_drift*dt;
  total = max(-100, min(100, total));
  total_drift = max(-100, min(100, total));
  float correction =  kp * error + kd * derivative + ki * total;
  float time_correction = max(0, 1+kp_time * error_time/1000);
  float drift_correction = kp_drift * error_drift + kd_drift * derivative_drift + ki * total_drift;

  // float drift_correction = 0.0;
  baseSpeed = baseSpeed*time_correction;
  Serial.print("drift correction ");
  Serial.print(drift_correction);
  Serial.print(" total x ");
  Serial.print(totalX);
  Serial.print(" total y ");
  Serial.println(totalY);
  if (paths[index] == "FORWARD" || paths[index] == "START"){
    // pwmFrontLeft = baseSpeed;
    // pwmFrontRight = baseSpeed;
    // pwmBackLeft = baseSpeed;
    // pwmBackRight = baseSpeed;
    pwmFrontLeft = min(max(baseSpeed - correction + drift_correction, 75), 255);
    pwmFrontRight = min(max(baseSpeed + correction - drift_correction, 75), 255);
    pwmBackLeft = min(max(baseSpeed - correction - drift_correction, 75), 255);
    pwmBackRight = min(max(baseSpeed + correction + drift_correction, 75), 255);
  } else if (paths[index]=="BACKWARD"){
    pwmFrontLeft = min(max(baseSpeed + correction + drift_correction, 75), 255);
    pwmFrontRight = min(max(baseSpeed - correction - drift_correction, 75), 255);
    pwmBackLeft = min(max(baseSpeed + correction - drift_correction, 75), 255);
    pwmBackRight = min(max(baseSpeed - correction + drift_correction, 75), 255);
  } else if (paths[index]=="LEFT"){
    pwmFrontLeft = -min(max(baseSpeed + correction - drift_correction, 75), 255);
    pwmFrontRight = min(max(baseSpeed + correction + drift_correction, 75), 255);
    pwmBackLeft = min(max(baseSpeed - correction - drift_correction, 75), 255);
    pwmBackRight = -min(max(baseSpeed - correction + drift_correction, 75), 255);
  } else {
    pwmFrontLeft = min(max(baseSpeed - correction + drift_correction, 75), 255);
    pwmFrontRight = -min(max(baseSpeed - correction - drift_correction, 75), 255);
    pwmBackLeft = -min(max(baseSpeed + correction + drift_correction, 75), 255);
    pwmBackRight = min(max(baseSpeed + correction - drift_correction, 75), 255);
  }
}

void moveWheel(int input1, int input2, int pwm, int speed) {
  if (speed>0){
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
    analogWrite(pwm, speed);
  } else{
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
    analogWrite(pwm, -speed);
  }
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(stbyBackMotors, HIGH);   // Enable back motors
  digitalWrite(stbyFrontMotors, HIGH);  // Enable front motors

  // Set motor direction
  moveWheel(input1BackRight, input2BackRight, pwmBackRightMotor, pwmBackRight);

  moveWheel(input1BackLeft, input2BackLeft, pwmBackLeftMotor, pwmBackLeft);

  moveWheel(input1FrontRight, input2FrontRight, pwmFrontRightMotor, pwmFrontRight);

  moveWheel(input1FrontLeft, input2FrontLeft, pwmFrontLeftMotor, pwmFrontLeft);
}


// Function to move the robot backward
void moveBackward() {
  digitalWrite(stbyBackMotors, HIGH);   // Enable back motors
  digitalWrite(stbyFrontMotors, HIGH);  // Enable front motors

  moveWheel(input1BackRight, input2BackRight, pwmBackRightMotor, -pwmBackRight);

  moveWheel(input1BackLeft, input2BackLeft, pwmBackLeftMotor, -pwmBackLeft);

  moveWheel(input1FrontRight, input2FrontRight, pwmFrontRightMotor, -pwmFrontRight);

  moveWheel(input1FrontLeft, input2FrontLeft, pwmFrontLeftMotor, -pwmFrontLeft);


}


// Function to turn the robot
void strafeLeft() {
  digitalWrite(stbyBackMotors, HIGH);   // Enable back motors
  digitalWrite(stbyFrontMotors, HIGH);  // Enable front motors

  moveWheel(input1BackRight, input2BackRight, pwmBackRightMotor, pwmBackRight);

  moveWheel(input1BackLeft, input2BackLeft, pwmBackLeftMotor, pwmBackLeft);

  moveWheel(input1FrontRight, input2FrontRight, pwmFrontRightMotor, pwmFrontRight);

  moveWheel(input1FrontLeft, input2FrontLeft, pwmFrontLeftMotor, pwmFrontLeft);

}


// Function to turn the robot
void strafeRight() {
  digitalWrite(stbyBackMotors, HIGH);   // Enable back motors
  digitalWrite(stbyFrontMotors, HIGH);  // Enable front motors


  moveWheel(input1BackRight, input2BackRight, pwmBackRightMotor, pwmBackRight);

  moveWheel(input1BackLeft, input2BackLeft, pwmBackLeftMotor, pwmBackLeft);

  moveWheel(input1FrontRight, input2FrontRight, pwmFrontRightMotor, pwmFrontRight);

  moveWheel(input1FrontLeft, input2FrontLeft, pwmFrontLeftMotor, pwmFrontLeft);

}


// Function to stop all motors
void stopMotors() {
  // turn motors off
  digitalWrite(stbyBackMotors, LOW);
  digitalWrite(stbyFrontMotors, LOW);
  // Set all motors to low
  analogWrite(pwmBackRightMotor, 0);
  analogWrite(pwmBackLeftMotor, 0);
  analogWrite(pwmFrontRightMotor, 0);
  analogWrite(pwmFrontLeftMotor, 0);
}



void loop() {  // turn = 19 cm
  int16_t deltaX, deltaY;
  flow.readMotionCount(&deltaX, &deltaY);
  totalX += deltaX;
  totalY += deltaY;
  // Serial.print(" total x ");
  // Serial.print(totalX);
  // Serial.print(" total y ");
  // Serial.println(totalY);
  if (paths[index] == "FORWARD"){
    if (started_new_action){
      left_over_time += goal_time - millis();
      start_time = millis();
      goal_time = start_time + time_per_step*1000 + left_over_time;
      started_new_action = false;
      resetCounters();
      length = 68;
    }else{
      if (medianTick(backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount)>length){
        started_new_action = true;
        stopMotors();
        index++;
      }else{
        moveForward();
      }
    }
  } else if (paths[index] == "BACKWARD"){
    if (started_new_action){
      left_over_time += goal_time - millis();
      start_time = millis();
      goal_time = start_time + time_per_step*1000 + left_over_time;
      started_new_action = false;
      resetCounters();
      length = 68;
    }else{
      if (medianTick( backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount)>length){
        started_new_action = true;
        stopMotors();
        index++;
      }else{
        moveBackward();
      }
    }
  } else if (paths[index] == "START"){
    if (started_new_action){
      left_over_time += goal_time - millis();
      start_time = millis();
      goal_time = start_time + time_per_step*1000/2 + left_over_time;
      started_new_action = false;
      resetCounters();
      length = 34;
    }else{
      Serial.print(" ticks ");
      Serial.print(medianTick( backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount));
      if (medianTick( backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount)>length){
        started_new_action = true;
        Serial.print(" finished ");
        stopMotors();
        index++;
      }else{
        moveForward();
      }
    }
  } else if (paths[index] == "RIGHT"){
    if (started_new_action){
      left_over_time += goal_time - millis();
      start_time = millis();
      goal_time = start_time + time_per_step*1000 + left_over_time;
      started_new_action = false;
      resetCounters();
      length = 73;
    }else{
      if (medianTick( backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount)>length){
        started_new_action = true;
        stopMotors();
        index++;
      }else{
        strafeRight();
      }
    }
  } else if (paths[index] == "LEFT"){
    if (started_new_action){
      left_over_time += goal_time - millis();
      start_time = millis();
      goal_time = start_time + time_per_step*1000 + left_over_time;
      started_new_action = false;
      resetCounters();
      length = 73;
    }else{
      if (medianTick( backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount)>length){
        started_new_action = true;
        stopMotors();
        index++;
      }else{
        strafeLeft();
      }
    }
  } else {
    stopMotors();
    while (true) {
      delay(100);
    }
  }

  sensors_event_t event;
  bno.getEvent(&event);

  float angle = event.orientation.x;
  // float angle = 0;
  error = calculateError(angle);
  adjustMotors();
  error_time = (millis() - start_time) - (float)medianTick(backLeftEncoderCount, frontLeftEncoderCount, frontRightEncoderCount, backRightEncoderCount)/(float)length * (goal_time-start_time);

  if (paths[index] == "FORWARD" || paths[index] == "BACKWARD" || paths[index] == "START"){
    error_drift = totalY;
  } else {
    error_drift = totalX;
  }

  Serial.print(" ");
  Serial.print(error_time);
  Serial.print(" ");
  Serial.print(paths[index]);
  Serial.print(" ");
}



