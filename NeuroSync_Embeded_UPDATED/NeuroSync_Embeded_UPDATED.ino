// Pin definitions for motors
const int motorA1 = 4;    // Motor A input 1
const int motorA2 = 5;    // Motor A input 2
const int motorAPWM = 9;  // Motor A PWM speed control

const int motorB1 = 6;    // Motor B input 1
const int motorB2 = 7;    // Motor B input 2
const int motorBPWM = 10; // Motor B PWM speed control

// Pin definitions for sensors and buzzer
const int TRIGGER_PIN = 11;
const int ECHO_PIN = 12;
const int TILT_SENSOR_PIN = 3;
const int SOS_STATE_PIN = 13;
const int GREEN_STATE_PIN = 8;

// States variables
int SOS_STATE = 0;
int GREEN_STATE = 0;

void setup() {
  // Initialize motor control pins as outputs
  initializeMotorPins();
  
  // Initialize sensor pins
  initializeSensorPins();
  
  // Initialize buzzer pin
  pinMode(SOS_STATE_PIN, OUTPUT);
  pinMode(GREEN_STATE_PIN, OUTPUT);
  
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
  
  handleStates();
}

void initializeMotorPins() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);
}

void initializeSensorPins() {
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(TILT_SENSOR_PIN, INPUT);
}

void handleStates() {
  GREEN_STATE = (SOS_STATE == 0) ? 1 : 0;
  digitalWrite(GREEN_STATE_PIN, GREEN_STATE);
  digitalWrite(SOS_STATE_PIN, SOS_STATE);
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
  
  return distance;
}

// Function to handle ultrasonic distance
void handleUltrasonicDistance(long distance) {
  if (distance < 20) {
    SOS_STATE = 1;
    stopMotors();
  } else {
    SOS_STATE = 0;
  }
}

// Function to handle tilt sensor
void handleTiltSensor() {
  int sensorState = digitalRead(TILT_SENSOR_PIN);
  if (sensorState == LOW) {
    SOS_STATE = 1;
  }
}

// Function to process serial commands
void processCommand(const String& command) {
  if (command == "stop") {
    stopMotors();
  } else if (command == "forward") {
    moveForward(70);
  } else if (command == "right") {
    turnRight(120);
  } else if (command == "left") {
    turnLeft(120);
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
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(motorAPWM, speed);
  
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(motorBPWM, speed);
}

// Function to turn the robot left
void turnLeft(int speed) {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(motorAPWM, speed);
  
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorBPWM, speed);
}
