#include <Wire.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);
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
  
  // Adjust these factors after testing your actual setup
  float R = (float)r;
  float G = (float)g;
  float B = (float)b;

  // To reduce noise, you can average multiple readings or filter them.
  // For now, we’ll just do a single read decision.

  String segment;
  // Define a factor to determine dominance. If one color must be at least X times the sum of the others:
  float dominanceFactor = 1.5; 
  // Example: R > (G + B) * 1.5 would mean Red is dominant.

  if (R > (G + B) * dominanceFactor) {
    segment = "LEFT (RED)";
  } else if (G > (R + B) * dominanceFactor) {
    segment = "CENTER (GREEN)";
  } else if (B > (R + G) * dominanceFactor) {
    segment = "RIGHT (BLUE)";
  } else {
    // In case no single color is strongly dominant:
    segment = "UNDEFINED - Adjust Thresholds";
  }

  // Print out the results
  Serial.print("Raw R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.print(b);
  Serial.print(" => Segment: ");
  Serial.println(segment);

  delay(100);
}