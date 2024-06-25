// Motor A connected to the motor driver
int motorA1 =  5; // Motor A input 1
int motorA2 =  4; // Motor A input 2
int motorAPWM =  10; // Motor A PWM speed control

// Motor B connected to the motor driver
int motorB1 = 6; // Motor B input 1
int motorB2 = 7; // Motor B input 2
int motorBPWM = 9; // Motor B PWM speed control

void setup() {
  // Define motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Begin serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString(); // Read the command from serial monitor


    if (command == "stop") {
      stopMotors();
    } else if (command == "forward") {
      moveForward(255); 
        Serial.println("recieved forwarded command"); 
      delay(5000); 
      stopMotors();
    } else if (command == "turn_right") {
      turnRight(255); 
      delay(5000); 
      stopMotors();
    } else if (command == "turn_left") {
      turnLeft(255); 
      delay(5000); 
      stopMotors();
    } else if (command.startsWith("speed")) {
      int speed = command.substring(6).toInt(); // Get speed from the command
      moveForward(speed); // Move forward with the specified speed
    } else {
      Serial.println("Invalid command"); // Print error message for unrecognized commands
    }
  }
}

// Function to move the robot forward
void moveForward(int speed) {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(motorAPWM, speed);

  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorBPWM, speed);
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  analogWrite(motorAPWM, 0);

  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(motorBPWM, 0);
}

// Function to turn the robot right
void turnRight(int speed) {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(motorAPWM, speed);

  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorBPWM, speed);
}

// Function to turn the robot left
void turnLeft(int speed) {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(motorAPWM, speed);

  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(motorBPWM, speed);
}
