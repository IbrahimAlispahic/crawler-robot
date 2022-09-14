#include <Servo.h>

Servo lowerJoint, upperJoint, wheel;
// Servo motors init positions
const int lowerJointInitPos = 0; //TODO: needs to be set
const int upperJointInitPos = 30; //TODO: needs to be set
const int wheelInitPos = 90;
// Servo motors pinout
const int lowerJointPin = 10;
const int upperJointPin = 9;
const int wheelPin = 8;

// Ultrasonic sensors pinout
const int pingPinLeft = 7; // Trigger Pin of Ultrasonic Sensor 1
const int echoPinLeft = 6; // Echo Pin of Ultrasonic Sensor 1
const int pingPinRight = 5; // Trigger Pin of Ultrasonic Sensor 2
const int echoPinRight = 4; // Echo Pin of Ultrasonic Sensor 2

bool wheelTurned = false;

void setup() {
  // initialisation of the motors
  lowerJoint.write(lowerJointInitPos);
  lowerJoint.attach(lowerJointPin);

  upperJoint.write(upperJointInitPos);
  upperJoint.attach(upperJointPin);

  wheel.write(wheelInitPos);
  wheel.attach(wheelPin);

  delay(100);

  // Serial.begin(9600); // Starting Serial Terminal
}

void loop() {
  long distanceLeft, distanceRight;
  distanceLeft = getDistance(pingPinLeft, echoPinLeft);
  distanceRight = getDistance(pingPinRight, echoPinRight);

  // distanceToSerial(distanceLeft, distanceRight);
  explore(distanceLeft, distanceRight);
}

int getDistance(int pingPin, int echoPin) {
  long duration, cm;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void explore(long distanceLeft, long distanceRight) {
  long threshold = 40;
  int turnAngle = 90;
  // if obstacle on both sides, move randomly left or right
  if (distanceLeft <= threshold && distanceRight <= threshold) {
    //    Serial.print("Stoping...Generating random movement...");
    //    Serial.println();
    int n = rand() % 2;
    if (n == 0) {
      //      Serial.print("Moving left...");
      //      turn(-turnAngle);
    } else {
      //      Serial.print("Moving right...");
      //      turn(turnAngle);
    }
    //    Serial.println();
  } else if (distanceLeft <= threshold && distanceRight > threshold) {
    //    Serial.print("Moving right...");
    //    turn(turnAngle);
    //    Serial.println();
  } else if (distanceLeft > threshold && distanceRight <= threshold) {
    //    Serial.print("Moving left...");
    //    turn(-turnAngle);
    //    Serial.println();
  } else {
    stepForward();
  }
}

void stepForward() {
  int d = 500;
  // Serial.print("Moving straight...");
  // move upper so the arm is straight
  upperJoint.write(upperJointInitPos + 90);
  delay(d);
  //move lower so it lifts the robot
  lowerJoint.write(lowerJointInitPos + 120);
  delay(d);
  if (wheelTurned) {
    wheel.write(wheelInitPos);
    wheelTurned = false;
  }
  //upper back so the robot moves forward
  upperJoint.write(upperJointInitPos + 40);
  delay(d * 2);
  //lower back so the robot is on the ground again
  upperJoint.write(upperJointInitPos + 10);
  lowerJoint.write(lowerJointInitPos + 60);
  delay(d);
}

void turn(int angle) {
  int d = 500;
  // if wheel turned, just return it to initial position
  if (wheelTurned) {
    upperJoint.write(upperJointInitPos + 45);
    delay(d);
    lowerJoint.write(lowerJointInitPos + 120);
    delay(d);
    lowerJoint.write(lowerJointInitPos);
    delay(d);
    upperJoint.write(upperJointInitPos);
    delay(d);
  }
  wheel.write(wheelInitPos + angle);
  wheelTurned = true;
  delay(d);
}

void distanceToSerial(long distanceLeft, long distanceRight) {
  Serial.print("Left obstacle distance: ");
  Serial.print(distanceLeft);
  Serial.print("cm");
  Serial.println();
  Serial.print("Right obstacle distance: ");
  Serial.print(distanceRight);
  Serial.print("cm");
  Serial.println();
  Serial.println();
}
