#include "Wire.h"
#include "Adafruit_TCS34725.h"

// Initialize the TCS34725 color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// Adjustable parameters
const float gearRatio = 25.0;      // Set this to 8.33 or 25 based on your gear setup
const int numStrips = 24;          // Number of total color strips (RGB/BGR)
const float steeringRange = 23.0;  // Full range in degrees (e.g., ±11.5°)
const int colorThreshold = 2000;   // Adjust according to your clear channel readings

// Derived values
const float stripeSpacing = 360.0 / numStrips;
const float anglePerTick = stripeSpacing / gearRatio;

// Variables
volatile int ticks = 0;
int previousTicks = 0;
bool lastWhite = false;
bool isRunning = true;

void setup() {
  Serial.begin(9600);
  if (!tcs.begin()) {
    Serial.println("No TCS34725 sensor found. Check wiring.");
    while (1);
  }

  Serial.println("System Initialized. Type 'START' or 'STOP' to control.");
}

void loop() {
  // Listen for start/stop commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "START") {
      isRunning = true;
      Serial.println("System Resumed.");
    } else if (command == "STOP") {
      isRunning = false;
      Serial.println("System Stopped.");
    }
  }

  if (!isRunning) {
    delay(500);
    return;
  }

  // Read RGB values
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("Raw Value: ");
  Serial.println(c);

  bool isWhite = (c > colorThreshold);

  // Detect transitions between strips
  if (isWhite != lastWhite) {
    ticks += (isWhite) ? 1 : 1; // Both transitions increase tick
    lastWhite = isWhite;
    Serial.print("Stripe transition detected. Ticks updated: ");
    Serial.println(ticks);
    delay(200);  // Debounce delay
  }

  // Wrap ticks within 360 degrees
  if (ticks > numStrips) ticks = 0;
  if (ticks < 0) ticks = numStrips;

  // Calculate steering angle
  float steeringAngle = ticks * anglePerTick;

  // Output tick and angle
  Serial.print("Ticks: ");
  Serial.print(ticks);
  Serial.print(", Steering Angle: ");
  Serial.print(steeringAngle, 2);
  Serial.println(" degrees");

  delay(500);  // Adjust based on rotation speed
}
