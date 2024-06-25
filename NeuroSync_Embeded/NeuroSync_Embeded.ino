// Motor A connected to the motor driver
int motorA1 =  4;    // Motor A input 1
int motorA2 =  5;    // Motor A input 2
int motorAPWM =  10; // Motor A PWM speed control

// Motor B connected to the motor driver
int motorB1 = 6;   // Motor B input 1
int motorB2 = 7;   // Motor B input 2
int motorBPWM = 9; // Motor B PWM speed control

// Define the pins for the ultrasonic sensor and buzzer
const int TRIGGER_PIN = 11;
const int ECHO_PIN = 12;

// Define Pin for tilt sensor
int TILT_SENSOR_PIN =2;

// Pin connected to the buzzer
const int BUZZER_PIN = 13; 

void setup() {
  // Define motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Define ultrasonic sensor pins and buzzer pin as outputs/inputs
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT); 

  // Set tilt sensor pin as input
  pinMode(TILT_SENSOR_PIN,INPUT);

  // Begin serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Read distance from ultrasonic sensor and take action if needed
  long distance = getUltrasonicDistance();
  handleUltrasonicDistance(distance);
  handleTiltSensor();

  // Check for serial commands to control the motors
  if (Serial.available() > 0) {
    String command = Serial.readString(); // Read the command from serial monitor

    if (command == "stop") {
      stopMotors();
    } else if (command == "forward") {
      moveForward(80); 

    } else if (command == "turn_right") {
      turnRight(140); 
      delay(1000);
      stopMotors();

    } else if (command == "turn_left") {
      turnLeft(140); 
      delay(1000);
      stopMotors();


    } else {
      Serial.println("Invalid command"); // Print error message for unrecognized commands
    }
  }

  delay(500); // Wait before taking the next measurement
}



// Function to get distance from ultrasonic sensor
long getUltrasonicDistance() {
  long duration, distance;

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse to trigger pin
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(8);
  digitalWrite(TRIGGER_PIN, LOW);

  // Measure the duration of the pulse on the echo pin
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in centimeters based on the speed of sound
  distance = duration * 0.034 / 2; // Speed of sound is approximately 34 microseconds per centimeter

  // Print the distance
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

// Function to handle ultrasonic distance
void handleUltrasonicDistance(long distance) {
  if (distance < 30) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
    stopMotors(); // Stop the motors
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
  }
}

 // Function to handle titl sensor 
 void handleTiltSensor() {
  int sensorState = digitalRead(TILT_SENSOR_PIN);
  
  if (sensorState == LOW) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
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
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(motorAPWM, speed);

  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(motorBPWM, speed);
}

// Function to turn the robot left
void turnLeft(int speed) {
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorAPWM, speed);

  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(motorBPWM, speed);
}
