#include "Arduino.h"
#include <Metro.h>
#include "NewPing.h"
#include "AccelStepper.h"

// Driving Motors
const int enablePinLeft = 3;
const int directionPinLeft = 4;
const int enablePinRight = 8;
const int directionPinRight = 9;

// Tape Sensors
const int leftIRpin = 14;
const int rightIRpin = 16;
const int edgeIRPin = 15;

// Ultrasonic Sensor
const int triggerPin = 7;
const int echoPin = 12;

// Ball Servo
const int stepPin = 6;
const int directionPin = 5;

// Flywheel
const int flywheelPin = 2;

// IR Sensor
const int IRPin = 22;

Metro timerOne(1000);
Metro gameTimer(130000);
//Metro sonarTimer(100);

IntervalTimer calculateFrequencyTimer;

int counter = 0;
int frequency = 0;

const int redFrequency = 1150;
const int blueFrequency = 1955;
const int attackFrequency = blueFrequency;

AccelStepper stepper(1, stepPin, directionPin);

typedef enum {
  INIT, TO_ARMOURY, TURN_TO_BUTTON, PUSH_BUTTON, RELOAD, STOP_BEFORE_TO_WINTERFELL, BEFORE_TO_WINTERFELL, TO_WINTERFELL, APPROACH_WINTERFELL, FACE_WINTERFELL, SHOOT_WINTERFELL, FACE_CASTERLY_ROCK, SHOOT_CASTERLY_ROCK, FIND_LINE_TO_ARMOURY, BEFORE_TO_KINGS_LANDING, TO_KINGS_LANDING, FACE_KINGS_LANDING, SHOOT_KINGS_LANDING, TO_SHOOT_DRAGONSTONE, FACE_DRAGONSTONE, SHOOT_DRAGONSTONE, END_GAME
} State_t;

typedef enum {
  BOTH_ON, LEFT_OFF, RIGHT_OFF, BOTH_OFF,
} LineFollowingState_t;

typedef enum {
  NONDECREASING, NONINCREASING, MINIMIZING
} WallMinimizingState_t;

State_t state;
LineFollowingState_t lineFollowingState;
WallMinimizingState_t wallMinimizingState;

uint16_t leftThresh; //TODO: Are all the thresholds going to be different?
uint16_t rightThresh;
uint16_t edgeThresh; // TODO: Implement hysteresis

uint16_t previousEdgeValues[2];

#define MAX_DISTANCE 200
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

int minimumDistance = MAX_DISTANCE;
int maximumDistance = 0;

bool winterfellDone = false;
bool casterlyRockDone = false;
bool kingsLandingDone = false;
bool dragonstoneDone = false;

bool lineFollowing = false;
bool minimizingWall = false;

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
  pinMode(IRPin, INPUT);

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
  state = INIT;

  setDrivingMotors(255, 255, LOW, LOW);

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
  edgeThresh = (analogRead(edgeIRPin) + analogRead(edgeIRPin) +
          analogRead(edgeIRPin))/3 - thresholdOffset;  //starts as white
  Serial.print("edgeThresh: ");
  Serial.println(edgeThresh);

  previousEdgeValues[0] = previousEdgeValues[1] = edgeThresh + thresholdOffset;
  gameTimer.reset();
}

void countFallingEdges() {
  counter++;
}

void calculateFrequency() {
  frequency = counter * 10;
  counter = 0;
}

// TODO: Hysteresis
bool leftOff() {
  return analogRead(leftIRpin) > leftThresh;
}

bool rightOff() {
  return analogRead(rightIRpin) > rightThresh;
}

bool checkKing() {
  uint16_t currentValue = analogRead(edgeIRPin);
  uint16_t edgeAverage = (currentValue + previousEdgeValues[0] + previousEdgeValues[1]) / 3;
  previousEdgeValues[0] = previousEdgeValues[1];
  previousEdgeValues[1] = currentValue;
  return edgeAverage < edgeThresh;
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
  if (gameTimer.check()) {
    state = END_GAME;
    lineFollowing = false;
    setDrivingMotors(0, 0, LOW, LOW);
    digitalWrite(flywheelPin, LOW);
    Serial.println("Game Round Complete");
  }
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
      //   // unsigned long distance = sonar.ping_cm();
      //   Serial.println(analogRead(IRPin));
      //   // Serial.println(distance);
      //   // Serial.println(analogRead(edgeIRPin));
      //   delay(100);
      // }
      // {
      //   state = FACE_WINTERFELL;
      //   calculateFrequencyTimer.begin(calculateFrequency, 100000);
      //   attachInterrupt(digitalPinToInterrupt(IRPin), countFallingEdges, FALLING);
      //   Serial.println("Face Winterfell");
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
          Serial.println("Turning to button");
        } else {
          delay(100); // Don't immediately trigger sonar again.
        }
      }
      break;
    case TURN_TO_BUTTON:
      {
        if (checkKing()) {
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
        if (!winterfellDone && !casterlyRockDone) {
          Serial.println("Stop Before Winterfell");
          state = STOP_BEFORE_TO_WINTERFELL;
          setDrivingMotors(255, 255, HIGH, HIGH);
        } else if (!kingsLandingDone) { //&& !dragonstoneDone) {
          Serial.println("Before Kings Landing");
          state = BEFORE_TO_KINGS_LANDING;
          setDrivingMotors(255, 255, HIGH, HIGH);
          delay(2000);
          setDrivingMotors(255, 255, LOW, HIGH);
          delay(1500);
          setDrivingMotors(255, 255, LOW, LOW);
        } else {
          // TODO: Figure out what to do here.
          winterfellDone = casterlyRockDone = kingsLandingDone = dragonstoneDone = false;
          Serial.println("Stop Before Winterfell");
          state = STOP_BEFORE_TO_WINTERFELL;
          setDrivingMotors(255, 255, HIGH, HIGH);
        }
      }
      break;
    case STOP_BEFORE_TO_WINTERFELL:
      if (checkKing()) {
        Serial.println("Before To Winterfell");
        state = BEFORE_TO_WINTERFELL;
        setDrivingMotors(255, 255, LOW, HIGH);
      }
      break;
    case BEFORE_TO_WINTERFELL:
      {
        bool leftOffBool = leftOff();
        bool rightOffBool = rightOff();
        if (!leftOffBool || !rightOffBool) {
          Serial.println("To Winterfell");
          state = TO_WINTERFELL;
          if (!leftOffBool && !rightOffBool)
            lineFollowingState = BOTH_ON;
          else if (!leftOffBool && rightOffBool)
            lineFollowingState = RIGHT_OFF;
          else
            lineFollowingState = LEFT_OFF;
          lineFollowing = true;
          timerOne.interval(3000);
          timerOne.reset();
        }
      }
      break;
    case TO_WINTERFELL:
      if (timerOne.check()) {
        state = APPROACH_WINTERFELL;
        lineFollowing = false;
        setDrivingMotors(255, 255, LOW, HIGH);
        delay(100);
        setDrivingMotors(255, 255, LOW, LOW);
        Serial.println("Approaching Winterfell");
      }
      break;
    case APPROACH_WINTERFELL:
      {
        unsigned long distance = sonar.ping_cm();
        Serial.println(distance);
        if (distance > 0 && distance < 20) {
          state = FACE_WINTERFELL;
          setDrivingMotors(125, 125, HIGH, LOW);
          frequency = 0;
          calculateFrequencyTimer.begin(calculateFrequency, 100000);
          attachInterrupt(digitalPinToInterrupt(IRPin), countFallingEdges, FALLING);
          Serial.println("Face Winterfell");
        } else {
          delay(100);
        }
      }
      break;
    case FACE_WINTERFELL:
      Serial.println(frequency);
      if ((blueFrequency - 200 < frequency && frequency < blueFrequency + 200)
          || (redFrequency - 200 < frequency && frequency < redFrequency + 200)) {
        state = SHOOT_WINTERFELL;
        calculateFrequencyTimer.end();
        detachInterrupt(digitalPinToInterrupt(IRPin));
        setDrivingMotors(0, 0, LOW, LOW);
        digitalWrite(flywheelPin, HIGH);
        delay(1000);
        stepper.setMaxSpeed(2);
        stepper.move(-16); // TODO: Determine Distance To Shoot Two Balls
        Serial.println("Shoot Winterfell");
      }
      if (!leftOff()) {
        // We missed finding the IR beacon, retrack to shoot King's Landing.
        setDrivingMotors(255, 255, LOW, HIGH);
        delay(500);
        state = BEFORE_TO_KINGS_LANDING;
        calculateFrequencyTimer.end();
        detachInterrupt(digitalPinToInterrupt(IRPin));
        Serial.println("Winterfell IR not detected, go to King's Landing");
      }
      break;
    case SHOOT_WINTERFELL:
      if (stepper.distanceToGo() == 0) {
        delay(500); // Wait for ball to shoot.
        state = FACE_CASTERLY_ROCK;
        setDrivingMotors(150, 150, LOW, HIGH);
        delay(200);
        setDrivingMotors(0, 0, LOW, LOW);
        winterfellDone = true;
        Serial.println("Face Casterly Rock");
      } else {
        stepper.run();
      }
      break;
    case FACE_CASTERLY_ROCK:
      state = SHOOT_CASTERLY_ROCK;
      stepper.setMaxSpeed(2);
      stepper.move(-16);
      Serial.println("Shoot Casterly Rock");
      break;
    case SHOOT_CASTERLY_ROCK:
      if (stepper.distanceToGo() == 0) {
        delay(500); // Wait for ball to shoot.
        state = FIND_LINE_TO_ARMOURY;
        digitalWrite(flywheelPin, LOW);
        setDrivingMotors(255, 255, LOW, HIGH);
        delay(2000);
        setDrivingMotors(255, 255, LOW, LOW);
        casterlyRockDone = true;
        Serial.println("Find line to Armoury");
      } else {
        stepper.run();
      }
      break;
    case FIND_LINE_TO_ARMOURY:
      {
        bool leftOffBool = leftOff();
        bool rightOffBool = rightOff();
        if (!leftOffBool || !rightOffBool) {
          Serial.println("Going to Armoury");
          state = TO_ARMOURY;
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
        setDrivingMotors(255, 255, HIGH, LOW);
        delay(500); // TODO: IDEA: create a variable called previousValue which is either high or low, and then trigger the transition only when previous is low and current is high.
        Serial.println("Face Kings Landing");
      }
      break;
    case FACE_KINGS_LANDING:
      Serial.print(analogRead(edgeIRPin));
      Serial.print(" ");
      Serial.print(edgeThresh);
      Serial.print(" ");
      Serial.println(checkKing());

      if (checkKing()) {
        state = SHOOT_KINGS_LANDING;
        setDrivingMotors(255, 255, LOW, HIGH);
        delay(100);
        setDrivingMotors(0, 0, LOW, LOW);
        digitalWrite(flywheelPin, HIGH);
        delay(1000);
        stepper.setMaxSpeed(2);
        stepper.move(-16); // TODO: Determine Distance To Shoot Two Balls
        Serial.println("Shoot Kings Landing");
      }
      break;
    case SHOOT_KINGS_LANDING:
      if (stepper.distanceToGo() == 0) {
        // state = FACE_DRAGONSTONE;
        // // digitalWrite(flywheelPin, LOW);
        // setDrivingMotors(150, 150, LOW, HIGH);
        // stepper.setMaxSpeed(2);
        // stepper.move(-24);
        // // delay(500); // TODO: IDEA: create a variable called previousValue which is either high or low, and then trigger the transition only when previous is low and current is high.
        // calculateFrequencyTimer.begin(calculateFrequency, 100000);
        // attachInterrupt(digitalPinToInterrupt(IRPin), countFallingEdges, FALLING);
        // Serial.println("Face Dragonstone");
        state = FIND_LINE_TO_ARMOURY;
        Serial.println("Find line to Armoury");
        setDrivingMotors(255, 255, HIGH, LOW);
        digitalWrite(flywheelPin, LOW);
        delay(500);
        kingsLandingDone = true;
      } else {
        stepper.run();
      }
      break;
    // case TO_SHOOT_DRAGONSTONE:
    //   {
    //     unsigned long distance = sonar.ping_cm();
    //     Serial.println(distance);
    //     if (distance > 0 && distance < 15) {
    //       state = FACE_DRAGONSTONE;
    //       setDrivingMotors(255, 255, HIGH, LOW);
    //       calculateFrequencyTimer.begin(calculateFrequency, 100000);
    //       attachInterrupt(digitalPinToInterrupt(IRPin), countFallingEdges, FALLING);
    //       Serial.println("Face Dragonstone");
    //     } else {
    //       delay(100);
    //     }
    //   }
      // break;
    case FACE_DRAGONSTONE:
      {
        if ((redFrequency - 200 < frequency && frequency < redFrequency + 200)
            || (blueFrequency - 200 < frequency && frequency < blueFrequency + 200)) {
          state = SHOOT_DRAGONSTONE;
          calculateFrequencyTimer.end();
          detachInterrupt(digitalPinToInterrupt(IRPin));
          setDrivingMotors(0, 0, LOW, LOW);
          // digitalWrite(flywheelPin, HIGH);
          // delay(1000);
          stepper.setMaxSpeed(2);
          stepper.move(-24);
          Serial.println("Shoot Dragonstone");
        }
        stepper.run();
      }
      break;
    case SHOOT_DRAGONSTONE:
      if (stepper.distanceToGo() == 0) {
        digitalWrite(flywheelPin, LOW);
      } else {
        stepper.run();
      }
      break;
    case END_GAME:
      // Round Over
      break;
    default:
      Serial.println("Not implemented.");
      break;
  }
}
