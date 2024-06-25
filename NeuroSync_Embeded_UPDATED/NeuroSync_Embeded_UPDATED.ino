// Pin definitions for motors
const int motorA1 = 4;    // Motor A input 1
const int motorA2 = 5;    // Motor A input 2
const int motorAPWM = 9; // Motor A PWM speed control

const int motorB1 = 6;    // Motor B input 1
const int motorB2 = 7;    // Motor B input 2
const int motorBPWM = 10;  // Motor B PWM speed control

// Pin definitions for sensors and buzzer
const int TRIGGER_PIN = 11;
const int ECHO_PIN = 12;
const int TILT_SENSOR_PIN = 2;
const int BUZZER_PIN = 13;

// Buzzer state variable
int buzzerState = 0;

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Initialize sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TILT_SENSOR_PIN, INPUT);
  
  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);

  // Begin serial communication
  Serial.begin(9600);
}

void loop() {
  long distance = getUltrasonicDistance();
  handleUltrasonicDistance(distance);
  handleTiltSensor();

  if (Serial.available() > 0) {
    String command = Serial.readString();
    processCommand(command);
  }

  digitalWrite(BUZZER_PIN, buzzerState);
}

// Function to get distance from the ultrasonic sensor
long getUltrasonicDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;

  // Serial.print("Distance = ");
  // Serial.print(distance);
  // Serial.println(" cm");

  return distance;
}

// Function to handle ultrasonic distance
void handleUltrasonicDistance(long distance) {
  if (distance < 30) {
    buzzerState = 1;
    stopMotors();
  } else {
    buzzerState = 0;
  }
}

// Function to handle tilt sensor
void handleTiltSensor() {
  int sensorState = digitalRead(TILT_SENSOR_PIN);
  if (sensorState == LOW) {
    buzzerState = 1;
  }
}

// Function to process serial commands
void processCommand(const String& command) {
  if (command == "stop") {
    stopMotors();
  } else if (command == "forward") {
    moveForward(90 +20);
  } else if (command == "turn_right") {
    turnRight(152 +20);
    // delay(1100);
    // stopMotors();
  } else if (command == "turn_left") {
    turnLeft(152 +20);
    // delay(1100);
    // stopMotors();
  } else {
    Serial.println("Invalid command");
    buzzerState = 1;
  }
}

// Function to move the robot forward
void moveForward(int speed) {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(motorAPWM, speed );

  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorBPWM, speed - 17);

  Serial.println("forward");
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  analogWrite(motorAPWM, 0);

  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(motorBPWM, 0);

    Serial.println("stop");
}

// Function to turn the robot right
void turnRight(int speed) {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(motorAPWM, speed +17 );

  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(motorBPWM, speed);

   Serial.println("right");
}

// Function to turn the robot left
void turnLeft(int speed) {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(motorAPWM, speed);

  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorBPWM, speed);

  Serial.println("left");
}
