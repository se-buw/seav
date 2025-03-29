// Motor Driver
#define IN1 9  // Rear Motor Backward
#define IN2 6  //  Rear Motor Forward
#define IN3 5   // Steering Motor Left (Digital)
#define IN4 3   // Steering Motor Right (Digital)

// Color Sensor
#include <Wire.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const uint16_t clearThreshold = 500; // Threshold to detect white
const float distancePerSegment = 8.125; // Distance per white segment in cm

bool lastWhiteDetected = false;
float totalDistance = 0.0;
int motorDirection = 1; // 1 for forward, -1 for reverse
int motorSpeed = 100;    // Initial speed for the rear motor (0 to 255)

void setup() {
  Serial.begin(2000000); // Initialize Serial Communication
  Serial.println("Debug: Setup Started"); // Debug message

  // Initialize Color Sensor
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // Halt execution if the color sensor fails to initialize
  }
  Serial.println("TCS34725 found and initialized!");

  // Initialize Motor Driver
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Debug: Setup Completed");
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

    // Debug: Print the received command
    Serial.print("Debug: Received Command: ");
    Serial.println(command);

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
    } else if (command.startsWith("SPEED")) {
      int newSpeed = command.substring(6).toInt();
      if (newSpeed >= 0 && newSpeed <= 255) {
        motorSpeed = newSpeed;
        Serial.print("Debug: Speed set to: ");
        Serial.println(motorSpeed);
      } else {
        Serial.println("Invalid speed. Enter a value between 0 and 255.");
      }
    } else if (command == "RESET") {
      totalDistance = 0.0;
      Serial.println("Debug: Distance reset to zero");
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
  analogWrite(IN1, 0); // PWM for speed control
  analogWrite(IN2, motorSpeed);         // Ensure backward pin is off
  Serial.println("Debug: Motor Moving Forward");
}

void moveBackward() {
  analogWrite(IN1, motorSpeed);         // Ensure forward pin is off
  analogWrite(IN2, 0); // PWM for speed control
  Serial.println("Debug: Motor Moving Backward");
}

void moveLeft() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Debug: Moving Left");
}

void moveRight() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Debug: Moving Right");
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Debug: Motor Stopped");
}
