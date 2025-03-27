// Motor Driver Pins
#define IN1 9  // Rear Motor Forward
#define IN2 6  // Rear Motor Backward
#define IN3 5  // Steering Motor Left
#define IN4 3  // Steering Motor Right

String inputCommand = ""; // Buffer for incoming serial commands

void setup() {
  Serial.begin(115200);  // Initialize serial communication
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    char received = Serial.read();
    if (received == '\n') {  // Process the command when a newline is received
      processCommand(inputCommand);
      inputCommand = "";  // Clear the buffer
    } else {
      inputCommand += received;  // Append the received character to the buffer
    }
  }
}

void processCommand(String command) {
  if (command == "FORWARD") {
    moveForward();
  } else if (command == "BACKWARD") {
    moveBackward();
  } else if (command == "LEFT") {
    steerLeft();
  } else if (command == "RIGHT") {
    steerRight();
  } else if (command == "FORWARD_RIGHT") {
    moveForward();
    steerRight();
  } else if (command == "FORWARD_LEFT") {
    moveForward();
    steerLeft();
  } else if (command == "BACKWARD_RIGHT") {
    moveBackward();
    steerRight();
  } else if (command == "BACKWARD_LEFT") {
    moveBackward();
    steerLeft();
  } else if (command == "STOP") {
    stopMotors();
  } else {
    Serial.println("Unknown Command");
  }
}

void moveForward() {
  digitalWrite(IN1, 0);  // Rear motor forward
  digitalWrite(IN2, 100);    // Rear motor backward off
  Serial.println("Motor Moving Forward");
}

void moveBackward() {
  digitalWrite(IN1, 100);    // Rear motor forward off
  digitalWrite(IN2, 0);  // Rear motor backward
  Serial.println("Motor Moving Backward");
}

void steerLeft() {
  digitalWrite(IN3, 0);  // Steering motor left
  digitalWrite(IN4, 100);    // Steering motor right off
  Serial.println("Steering Left");
}

void steerRight() {
  digitalWrite(IN3, 100);    // Steering motor left off
  digitalWrite(IN4, 0);  // Steering motor right
  Serial.println("Steering Right");
}

void stopMotors() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  Serial.println("Motors Stopped");
}
