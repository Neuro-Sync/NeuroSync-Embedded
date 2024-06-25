// Define the pins for the ultrasonic sensor and LED
const int TRIGGER_PIN = 11;
const int ECHO_PIN = 12;
const int LED_PIN = 13; // Pin connected to the LED

void setup() {
  Serial.begin(9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); // Set LED pin as output
}

void loop() {
  long duration, distance;
  

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond pulse to trigger pin
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  // Measure the duration of the pulse on the echo pin
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance in centimeters based on the speed of sound
  distance = duration * 0.034 / 2; // Speed of sound is approximately 34 microseconds per centimeter
  
  // Print the distance
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Check if distance is less than 100 cm
  if (distance < 90) {
    digitalWrite(LED_PIN, HIGH); // Turn on the LED
  } else {
    digitalWrite(LED_PIN, LOW); // Turn off the LED
  }
  
  delay(500); // Wait before taking the next measurement
}
