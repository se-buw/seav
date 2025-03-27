#include <Wire.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Threshold for distinguishing white vs black (this must be determined experimentally)
// Start with a guess and adjust as necessary.
const uint16_t clearThreshold = 500; 

// Distance per white segment in cm
const float distancePerSegment = 8.125;

bool lastWhiteDetected = false;
unsigned long whiteSegmentsCount = 0;
float totalDistance = 0.0;

void setup() {
  Serial.begin(2000000);
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  // tcs.setInterrupt(false); // Turn on LED
  Serial.println("TCS34725 found and initialized!");
}

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  
  // Depending on your lighting conditions, you may also consider normalizing or using colorTemp, lux:
  // uint16_t colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  // uint16_t lux = tcs.calculateLux(r, g, b);
  
  // For simplicity, we just compare clear channel "c" against a threshold.
  bool isWhite = (c > clearThreshold);

  // Check for transition from black to white
  if (isWhite && !lastWhiteDetected) {
    whiteSegmentsCount++;
    totalDistance = whiteSegmentsCount * distancePerSegment;

    // Serial.print("White segment detected! Count: ");
    // Serial.print(whiteSegmentsCount);
    //Serial.print("  Total Distance: ");
    //Serial.print(totalDistance);
    //Serial.println(" cm");

    Serial.print("{\"distance\":");
    Serial.print(totalDistance);
    Serial.print("}");  
  
  }
  lastWhiteDetected = isWhite;
  delay(100);
}
