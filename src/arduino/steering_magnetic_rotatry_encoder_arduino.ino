#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

bool reading = false;  // Flag to control reading

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!encoder.begin()) {
    Serial.println("AS5600 not detected. Check connections.");
    while (1);
  }

  Serial.println("AS5600 initialized.");
  Serial.println("Type 'start' to begin reading angles, 'stop' to stop.");
}

void loop() {
  // Check if user sent a command via Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove newline/extra spaces

    if (command == "start") {
      reading = true;
      Serial.println("Start Reading...");
    } else if (command == "stop") {
      reading = false;
      Serial.println("Stopped Reading.");
    }
  }

  // If reading is enabled, print angle
  if (reading) {
    float angle = encoder.getAngle();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.println(" degrees");
    delay(100);
  }
}

