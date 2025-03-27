// Combined code for IMU, Color, Motor Driver
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_TCS34725.h>

// IMU Sensor (MPU6050)
MPU6050 imu;

// Color Sensor (TCS34725)
#define redpin 9   // Pin for RGB LED control (adjust as needed)
#define greenpin 10
#define bluepin 11
#define commonAnode true
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Motor Driver
#define IN1 9  // Motor A forward
#define IN2 6  // Motor A backward
#define IN3 5  // Motor B forward
#define IN4 3  // Motor B backward

void setup() {
  // Serial communication setup
  Serial.begin(115200);

  // Initialize I2C (default SDA = A4, SCL = A5)
  Wire.begin();

  // Initialize IMU Sensor
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Halt if IMU initialization fails
  }
  Serial.println("MPU6050 connected successfully");

  // Initialize Color Sensor
  if (tcs.begin()) {
    Serial.println("TCS34725 color sensor initialized");
  } else {
    Serial.println("No TCS34725 found ... check connections");
    while (1); // Halt if color sensor initialization fails
  }

  // Prepare gamma table for color sensor
  for (int i = 0; i < 256; i++) {
    float x = i / 255.0;
    x = pow(x, 2.5) * 255;
    gammatable[i] = commonAnode ? 255 - x : x;
  }
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  // Initialize Motor Driver
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // ==== IMU Sensor Data ====
  int16_t ax, ay, az, gx, gy, gz;
  imu.getAcceleration(&ax, &ay, &az);
  imu.getRotation(&gx, &gy, &gz);

  float ax_g = ax / 16384.0, ay_g = ay / 16384.0, az_g = az / 16384.0;
  float gx_dps = gx / 131.0, gy_dps = gy / 131.0, gz_dps = gz / 131.0;

  Serial.print("IMU: ax = "); Serial.print(ax_g, 2);
  Serial.print(", ay = "); Serial.print(ay_g, 2);
  Serial.print(", az = "); Serial.print(az_g, 2);
  Serial.print(", gx = "); Serial.print(gx_dps, 2);
  Serial.print(", gy = "); Serial.print(gy_dps, 2);
  Serial.print(", gz = "); Serial.println(gz_dps, 2);

  delay(500); // Delay for clarity

  // ==== Color Sensor Data ====
  float red, green, blue;
  tcs.setInterrupt(false);
  delay(60); // Allow sensor to stabilize
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);

  Serial.print("Color Sensor: R = "); Serial.print(int(red));
  Serial.print(", G = "); Serial.print(int(green));
  Serial.print(", B = "); Serial.println(int(blue));

  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);

  delay(500); // Delay for clarity

  // ==== Motor Driver ====
  // Move motor forward
  analogWrite(IN1, 170); // Forward direction Motor A
  analogWrite(IN2, 0);
  analogWrite(IN3, 170); // Forward direction Motor B
  analogWrite(IN4, 0);
  Serial.println("Moving Forward.");
  delay(3000); // Run for 3 seconds

  // Stop motor
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
  Serial.println("Stopping");
  delay(1000); // Stop for 1 second

  // Move motor backward
  analogWrite(IN1, 0);
  analogWrite(IN2, 170); // Backward direction Motor A
  analogWrite(IN3, 0);
  analogWrite(IN4, 170); // Backward direction Motor B
  Serial.println("Moving Backward.");
  delay(3000); // Run for 3 seconds

  // Stop motor
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
  Serial.println("Stopping");
  delay(1000); // Stop for 1 second
}
