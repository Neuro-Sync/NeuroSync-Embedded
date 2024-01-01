// Motor A connected to the motor driver
int motorA1 = 2; // Motor A input 1
int motorA2 = 3; // Motor A input 2

// Motor B connected to the motor driver
int motorB1 = 4; // Motor B input 1
int motorB2 = 5; // Motor B input 2

void setup() {
  // Define motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Begin serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString(); // Read the command from serial monitor

    // Check the received command and control motors accordingly
    if (command == "stop") {
      stopMotors();
    } else if (command == "forward") {
      moveForward();
      delay(2000); // Move forward for 2 seconds
      stopMotors();
    } else if (command == "turn_right") {
      turnRight();
      delay(500); // Adjust this delay to change turning speed
      stopMotors();
    } else if (command == "turn_left") {
      turnLeft();
      delay(500); // Adjust this delay to change turning speed
      stopMotors();
    } else {
      Serial.println("Invalid command"); // Print error message for unrecognized commands
    }
  }
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}
