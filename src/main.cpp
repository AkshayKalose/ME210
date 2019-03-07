#include "Arduino.h"
#include <Metro.h>
#include "NewPing.h"
#include "AccelStepper.h"

// Driving Motors
const int enablePinLeft = 8;
const int directionPinLeft = 9;
const int enablePinRight = 3;
const int directionPinRight = 4;

// Tape Sensors
const int leftIRpin = 14;
const int rightIRpin = 15;
const int edgeIRPin = 16;

// Ultrasonic Sensor
const int triggerPin = 7;
const int echoPin = 12;

// Ball Servo
const int stepPin = 6;
const int directionPin = 5;

// Flywheel
const int flywheelPin = 2;

Metro timerOne(1000);
Metro timerTwo(1000);

AccelStepper stepper(1, stepPin, directionPin);

typedef enum {
  INIT, TO_ARMOURY, TURN_TO_BUTTON, PUSH_BUTTON, RELOAD, BEFORE_TO_KINGS_LANDING, TO_KINGS_LANDING, FACE_KINGS_LANDING, SHOOT_KINGS_LANDING
} State_t;

typedef enum {
  BOTH_ON, LEFT_OFF, RIGHT_OFF, BOTH_OFF, // KINGS_LAND, FIYAH
} LineFollowingState_t;

typedef enum {
  NONDECREASING, NONINCREASING, MINIMIZING
} WallMinimizingState_t;

State_t state;
LineFollowingState_t lineFollowingState;
WallMinimizingState_t wallMinimizingState;

uint16_t leftThresh; //TODO: Are all the thresholds going to be different?
uint16_t rightThresh;
uint16_t kingsThresh; // TODO: Implement hysteresis

#define MAX_DISTANCE 200
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

int minimumDistance = MAX_DISTANCE;
int maximumDistance = 0;

bool kingsLandingDone = false;

bool lineFollowing = false;
bool minimizingWall = false;
bool edgeOffLine = false;

int currentDirectionLeft;
int currentDirectionRight;

void setDrivingMotors(int enableLeft, int enableRight, int directionLeft, int directionRight) {
  // TODO: allow input of -100 -> 100 and set direction automatically.
  currentDirectionLeft = directionLeft;
  currentDirectionRight = directionRight;
  analogWrite(enablePinLeft, enableLeft);
  analogWrite(enablePinRight, enableRight);
  digitalWrite(directionPinLeft, directionLeft);
  digitalWrite(directionPinRight, directionRight);
}

void setup() {
  pinMode(leftIRpin, INPUT);
  pinMode(rightIRpin, INPUT);
  pinMode(edgeIRPin, INPUT);

  pinMode(enablePinLeft, OUTPUT);
  pinMode(directionPinLeft, OUTPUT);
  pinMode(enablePinRight, OUTPUT);
  pinMode(directionPinRight, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(flywheelPin, OUTPUT);

  Serial.begin(9600);
  // while (!Serial);

  // Initial State
  // state = TO_KINGS_LANDING;
  // lineFollowing = true; // DISABLE WHEN CHANGING BACK TO INIT

  state = INIT;

  setDrivingMotors(255, 255, LOW, LOW);
  kingsLandingDone = false;

  // Calculate Thresholds
  uint16_t thresholdOffset = 100;
  leftThresh = (analogRead(leftIRpin) + analogRead(leftIRpin) +
          analogRead(leftIRpin))/3 + thresholdOffset; //starts as black
  Serial.print("leftThresh: ");
  Serial.println(leftThresh);
  rightThresh = (analogRead(rightIRpin)+analogRead(rightIRpin)+
          analogRead(rightIRpin))/3 + thresholdOffset;  //starts as black
  Serial.print("rightThresh: ");
  Serial.println(rightThresh);
  kingsThresh = (analogRead(edgeIRPin) + analogRead(edgeIRPin) +
          analogRead(edgeIRPin))/3 - thresholdOffset;  //starts as white
  Serial.print("kingThresh: ");
  Serial.println(kingsThresh);
}

// TODO: Hysteresis
bool leftOff() {
  return analogRead(leftIRpin) > leftThresh;
}

bool rightOff() {
  return analogRead(rightIRpin) > rightThresh;
}

bool checkKing() {
  return analogRead(edgeIRPin) < kingsThresh;
}

void executeLineFollowing() {
  switch (lineFollowingState) {
    case BOTH_ON:
    // Serial.println("Both on");
      if (leftOff()) {
        lineFollowingState = LEFT_OFF;
        Serial.println("Left off");
        setDrivingMotors(255, 100, LOW, LOW);
      } else if (rightOff()) {
        lineFollowingState = RIGHT_OFF;
        Serial.println("Right off");
        setDrivingMotors(100, 255, LOW, LOW);
      }
      break;
    case LEFT_OFF:
      if (!leftOff()) {
        lineFollowingState = BOTH_ON;
        Serial.println("Both on");
        setDrivingMotors(255, 255, LOW, LOW);
      } else if (rightOff()) {
        lineFollowingState = BOTH_OFF;
        Serial.println("Both off");
        setDrivingMotors(255, 255, LOW, HIGH);
      }
      break;
    case RIGHT_OFF:
      if (!rightOff()) {
        lineFollowingState = BOTH_ON;
        Serial.println("Both on");
        setDrivingMotors(255, 255, LOW, LOW);
      } else if (leftOff()) {
        lineFollowingState = BOTH_OFF;
        Serial.println("Both off");
        setDrivingMotors(255, 255, HIGH, LOW);
      }
      break;
    case BOTH_OFF:
      if (!rightOff()) {
        lineFollowingState = LEFT_OFF;
        Serial.println("Left off");
        setDrivingMotors(255, 100, LOW, LOW);
      } else if (!leftOff()) {
        lineFollowingState = RIGHT_OFF;
        Serial.println("Right off");
        setDrivingMotors(100, 255, LOW, LOW);
      }
      break;
    default: // Should never get into an unhandled lineFollowingState
      Serial.println("What is this I do not even...");
  }
}

void executeWallMinimization() {
  int distance = (int) sonar.ping_cm();
  Serial.println(distance);
  if (distance == 0) {
    return; // Not a valid reading.
  }
  switch (wallMinimizingState) {
    case NONDECREASING:
      if (distance > maximumDistance) {
        maximumDistance = distance;
      } else {
        // Current Distance <= Maximum Distance
        if (abs(maximumDistance - distance) >= 3) {
          // Sensor Threshold Exceeded
          // Wall Distance Has Switched From Increasing to Decreasing
          wallMinimizingState = NONINCREASING;
          minimumDistance = distance;
          Serial.println("\tNonincreasing");
        }
      }
      break;
    case NONINCREASING:
      if (distance < minimumDistance) {
        minimumDistance = distance;
      } else {
        // Current Distance >= Minimum Distance
        if (abs(minimumDistance - distance) >= 3) {
          // Sensor Threshold Exceeded
          // Wall Distance is Increasing For Sure
          wallMinimizingState = MINIMIZING;
          setDrivingMotors(255, 255, 1 - currentDirectionLeft, 1 - currentDirectionRight);
          Serial.println("\tMinimizing");
        }
      }
      break;
    case MINIMIZING:
        if (abs(distance - minimumDistance) <= 1) {
          setDrivingMotors(0, 0, LOW, LOW);
          minimizingWall = false;
          Serial.println("\tDone Minimizing");
        }
      break;
    default:
      break;
  }
  delay(100);
}

void loop() {
  if (lineFollowing) {
    executeLineFollowing();
  }
  if (minimizingWall) {
    executeWallMinimization();
  }
  switch (state) {
    case INIT:
      if (!leftOff() && !rightOff()) {
        state = TO_ARMOURY;
        lineFollowingState = BOTH_ON;
        lineFollowing = true;
        Serial.println("Moving to armoury");
      }
      // {
      //   unsigned long distance = sonar.ping_cm();
      //   Serial.println(distance);
      //   // Serial.println(analogRead(edgeIRPin));
      //   delay(100);
      // }
      break;
    case TO_ARMOURY:
      {
        unsigned long distance = sonar.ping_cm();
        Serial.println(distance);
        if (distance > 0 && distance < 15) {
          state = TURN_TO_BUTTON;
          lineFollowing = false;
          setDrivingMotors(255, 255, LOW, HIGH);
          wallMinimizingState = NONDECREASING;
          minimumDistance = MAX_DISTANCE;
          maximumDistance = 0;
          minimizingWall = true;
          // delay(300);
          Serial.println("Turning to button / Start Minimizing");
        } else {
          delay(100); // Don't immediately trigger sonar again.
        }
      }
      break;
    case TURN_TO_BUTTON:
      {
        // unsigned long distance = sonar.ping_cm();
        // Serial.println(distance);
        // if (distance > 0 && distance < 15) {
        //   state = PUSH_BUTTON;
        //   setDrivingMotors(255, 255, LOW, LOW);
        //   Serial.println("Pushing button");
        // } else {
        //   delay(100); // Don't immediately trigger sonar again.
        // }
        if (!minimizingWall) {
          state = PUSH_BUTTON;
          setDrivingMotors(255, 255, LOW, LOW);
          Serial.println("Pushing button");
        }
      }
      break;
    case PUSH_BUTTON:
      {
        unsigned long distance = sonar.ping_cm();
        Serial.println(distance);
        if (distance > 0 && distance < 5) {
          Serial.println("Reloading");
          state = RELOAD;
          delay(1000); // Make sure we hit the button. TODO: Use limit sensor.
          setDrivingMotors(0, 0, LOW, LOW);
          timerOne.interval(1000);
          timerOne.reset();
        } else {
          delay(100);
        }
      }
      break;
    case RELOAD:
      if (timerOne.check()) {
        Serial.println("Before Kings Landing");
        state = BEFORE_TO_KINGS_LANDING;
        setDrivingMotors(255, 255, HIGH, HIGH);
        delay(2000);
        setDrivingMotors(255, 255, LOW, HIGH);
        delay(1500);
        setDrivingMotors(255, 255, LOW, LOW);
      }
      break;
    case BEFORE_TO_KINGS_LANDING:
      {
        bool leftOffBool = leftOff();
        bool rightOffBool = rightOff();
        if (!leftOffBool || !rightOffBool) {
          Serial.println("Going to Kings Landing");
          state = TO_KINGS_LANDING;
          if (!leftOffBool && !rightOffBool)
            lineFollowingState = BOTH_ON;
          else if (!leftOffBool && rightOffBool)
            lineFollowingState = RIGHT_OFF;
          else
            lineFollowingState = LEFT_OFF;
          lineFollowing = true;
        }
      }
      break;
    case TO_KINGS_LANDING:
      if (checkKing()) {
        state = FACE_KINGS_LANDING;
        lineFollowing = false;
        // CODE TO MINIMIZE WALL
        // setDrivingMotors(255, 255, LOW, LOW);
        // delay(500);
        // setDrivingMotors(255, 255, HIGH, LOW);
        // delay(1000);
        // wallMinimizingState = NONDECREASING;
        // minimumDistance = MAX_DISTANCE;
        // maximumDistance = 0;
        // minimizingWall = true;

        // CODE TO TURN WITH EDGE PIN
        setDrivingMotors(255, 255, HIGH, LOW);
        delay(500);
        // edgeOffLine = false;
        edgeOffLine = true;
        Serial.println("Face Kings Landing");
        Serial.println(edgeOffLine);
      }
      break;
    case FACE_KINGS_LANDING:
      // CODE TO MINIMIZE WALL
      // if (!minimizingWall) {
      // CODE TO TURN WITH EDGE PIN
      Serial.println(edgeOffLine);
      Serial.print(analogRead(edgeIRPin));
      Serial.print(" ");
      Serial.print(kingsThresh);
      Serial.print(" ");
      Serial.println(checkKing());

      if (!edgeOffLine) {
        if (!checkKing()) {
          edgeOffLine = true;
        }
      }
      if (edgeOffLine && checkKing()) {
        state = SHOOT_KINGS_LANDING;
        setDrivingMotors(0, 0, LOW, LOW);
        digitalWrite(flywheelPin, HIGH);
        delay(1000);
        stepper.setMaxSpeed(1);
        stepper.move(-360);
        Serial.println("Shoot Kings Landing");
      }
      break;
    case SHOOT_KINGS_LANDING:
      if (stepper.distanceToGo() == 0) {
        digitalWrite(flywheelPin, LOW);
      } else {
        stepper.run();
      }
      break;
    default:
      Serial.println("Not implemented.");
      break;
  }
}
