// Motor Driver
#define IN1 9  // Rear Motor Forward
#define IN2 6  // Rear Motor Backward
#define IN3 5  // Steering Motor Left
#define IN4 3  // Steering Motor Right

// Color Sensor
#include <Wire.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const uint16_t clearThreshold = 500; // Threshold to detect white
const float distancePerSegment = 8.125; // Distance per white segment in cm

bool lastWhiteDetected = false;
float totalDistance = 0.0;
int motorDirection = 1; // 1 for forward, -1 for reverse

void setup() {
  Serial.begin(2000000);

  // Initialize Color Sensor
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  Serial.println("TCS34725 found and initialized!");

  // Initialize Motor Driver
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Handle motor control
  handleMotorCommands();

  // Read color sensor data
  readColorSensor();

  delay(100); // Adjust loop frequency as needed
}

void handleMotorCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "FORWARD") {
      motorDirection = 1;
      moveForward();
    } else if (command == "BACKWARD") {
      motorDirection = -1;
      moveBackward();
    } else if (command == "LEFT") {
      moveLeft();
    } else if (command == "RIGHT") {
      moveRight();
    } else if (command == "STOP") {
      stopMotors();
    } else {
      Serial.println("Unknown Command");
    }
  }
}

void readColorSensor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  bool isWhite = (c > clearThreshold);

  if (isWhite && !lastWhiteDetected) {
    totalDistance += motorDirection * distancePerSegment;

    // Print distance data in JSON format
    Serial.print("{\"distance\":");
    Serial.print(totalDistance);
    Serial.println("}");
  }

  lastWhiteDetected = isWhite;
}

// Motor Control Functions
void moveForward() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 100);
  Serial.println("Motor Moving Forward");
}

void moveBackward() {
  digitalWrite(IN1, 100);
  digitalWrite(IN2, 0);
  Serial.println("Motor Moving Backward");
}

void moveLeft() {
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 100);
  Serial.println("Moving Left");
}

void moveRight() {
  digitalWrite(IN3, 100);
  digitalWrite(IN4, 0);
  Serial.println("Moving Right");
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motor Stopped");
}
