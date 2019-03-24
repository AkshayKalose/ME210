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
const int leftIRPin = 14;
const int rightIRPin = 16;
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

IntervalTimer calculateFrequencyTimer;

int counter = 0;
int frequency = 0;

const int redFrequency = 1150;
const int blueFrequency = 1955;

AccelStepper stepper(1, stepPin, directionPin);

typedef enum {
  INIT, TO_ARMOURY, TURN_TO_BUTTON, PUSH_BUTTON, RELOAD, STOP_BEFORE_TO_WINTERFELL, BEFORE_TO_WINTERFELL, TO_WINTERFELL, APPROACH_WINTERFELL, FACE_WINTERFELL, SHOOT_WINTERFELL, FACE_CASTERLY_ROCK, SHOOT_CASTERLY_ROCK, FIND_LINE_TO_ARMOURY, BEFORE_TO_KINGS_LANDING, TO_KINGS_LANDING, FACE_KINGS_LANDING, SHOOT_KINGS_LANDING, END_GAME
} State_t;

typedef enum {
  BOTH_ON, LEFT_OFF, RIGHT_OFF, BOTH_OFF,
} LineFollowingState_t;

State_t state;
LineFollowingState_t lineFollowingState;

uint16_t leftThresh;
uint16_t rightThresh;
uint16_t edgeThresh;

uint16_t previousEdgeValues[2];

#define MAX_DISTANCE 200
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

int minimumDistance = MAX_DISTANCE;
int maximumDistance = 0;

bool winterfellDone = false;
bool casterlyRockDone = false;
bool kingsLandingDone = false;

bool lineFollowing = false;

int currentDirectionLeft;
int currentDirectionRight;

void setDrivingMotors(int enableLeft, int enableRight, int directionLeft, int directionRight) {
  currentDirectionLeft = directionLeft;
  currentDirectionRight = directionRight;
  analogWrite(enablePinLeft, enableLeft);
  analogWrite(enablePinRight, enableRight);
  digitalWrite(directionPinLeft, directionLeft);
  digitalWrite(directionPinRight, directionRight);
}

void setup() {
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
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
  leftThresh = (analogRead(leftIRPin) + analogRead(leftIRPin) +
    analogRead(leftIRPin)) / 3 + thresholdOffset; // Starts as black
  Serial.print("leftThresh: "); Serial.println(leftThresh);

  rightThresh = (analogRead(rightIRPin) + analogRead(rightIRPin) +
    analogRead(rightIRPin)) / 3 + thresholdOffset;  // Starts as black
  Serial.print("rightThresh: "); Serial.println(rightThresh);

  edgeThresh = (analogRead(edgeIRPin) + analogRead(edgeIRPin) +
    analogRead(edgeIRPin)) / 3 - thresholdOffset;  // Starts as white
  Serial.print("edgeThresh: "); Serial.println(edgeThresh);

  // Start Moving Average for Edge Tape Sensor
  previousEdgeValues[0] = previousEdgeValues[1] = edgeThresh + thresholdOffset;

  // Start 2:10 Game Timer
  gameTimer.reset();
}

void countFallingEdges() {
  counter++;
}

void calculateFrequency() {
  frequency = counter * 10;
  counter = 0;
}

bool leftOff() {
  return analogRead(leftIRPin) > leftThresh;
}

bool rightOff() {
  return analogRead(rightIRPin) > rightThresh;
}

bool checkEdge() {
  uint16_t currentValue = analogRead(edgeIRPin);
  uint16_t edgeAverage = (currentValue + previousEdgeValues[0] + previousEdgeValues[1]) / 3;
  previousEdgeValues[0] = previousEdgeValues[1];
  previousEdgeValues[1] = currentValue;
  return edgeAverage < edgeThresh;
}

void executeLineFollowing() {
  switch (lineFollowingState) {
    case BOTH_ON:
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
    default:
      Serial.println("What is this I do not even...");
  }
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
  switch (state) {
    case INIT:
      if (!leftOff() && !rightOff()) {
        state = TO_ARMOURY;
        lineFollowingState = BOTH_ON;
        lineFollowing = true;
        Serial.println("Moving to armoury");
      }
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
        if (checkEdge()) {
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
          delay(1000); // Make sure we hit the button.
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
        } else if (!kingsLandingDone) {
          Serial.println("Before Kings Landing");
          state = BEFORE_TO_KINGS_LANDING;
          setDrivingMotors(255, 255, HIGH, HIGH);
          delay(2000);
          setDrivingMotors(255, 255, LOW, HIGH);
          delay(1500);
          setDrivingMotors(255, 255, LOW, LOW);
        } else {
          // Restart loop; go to shoot down Winterfell.
          winterfellDone = casterlyRockDone = kingsLandingDone = false;
          Serial.println("Stop Before Winterfell");
          state = STOP_BEFORE_TO_WINTERFELL;
          setDrivingMotors(255, 255, HIGH, HIGH);
        }
      }
      break;
    case STOP_BEFORE_TO_WINTERFELL:
      if (checkEdge()) {
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
        stepper.move(-16);
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
        delay(500); // Wait for current ball to shoot.
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
        delay(2000); // Turn back to armoury.
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
      if (checkEdge()) {
        state = FACE_KINGS_LANDING;
        lineFollowing = false;
        setDrivingMotors(255, 255, HIGH, LOW);
        delay(500); // Get edge tape sensor off the line.
        Serial.println("Face Kings Landing");
      }
      break;
    case FACE_KINGS_LANDING:
      if (checkEdge()) {
        state = SHOOT_KINGS_LANDING;
        setDrivingMotors(255, 255, LOW, HIGH);
        delay(100); // Turn slightly right to adjust for offset.
        setDrivingMotors(0, 0, LOW, LOW);
        digitalWrite(flywheelPin, HIGH);
        delay(1000);
        stepper.setMaxSpeed(2);
        stepper.move(-16);
        Serial.println("Shoot Kings Landing");
      }
      break;
    case SHOOT_KINGS_LANDING:
      if (stepper.distanceToGo() == 0) {
        state = FIND_LINE_TO_ARMOURY;
        setDrivingMotors(255, 255, HIGH, LOW);
        digitalWrite(flywheelPin, LOW);
        delay(500);
        kingsLandingDone = true;
        Serial.println("Find line to Armoury");
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
