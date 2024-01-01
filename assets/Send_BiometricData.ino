#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <pulseSensor.h>

#define TEMPERATURE_SENSOR_ADDRESS 0x5A // I2C address for temperature sensor

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
pulseSensor pulseSensor; // Initialize pulse sensor object

const int PULSE_SENSOR_PIN = A0; // Pin to which the pulse sensor is connected

unsigned long previousMillis = 0;
const long interval = 60000; // Interval in milliseconds (1 minute)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mlx.begin();
  pulseSensor.attach(PULSE_SENSOR_PIN); // Attaching the pulse sensor to specified pin
  pulseSensor.initialize();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float temperature = mlx.readObjectTempC();
    int heartRate = pulseSensor.getBeatsPerMinute();

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Heart Rate: ");
    Serial.print(heartRate);
    Serial.println(" BPM");
  }
}


// The I2C communication for the MLX90614 temperature sensor doesn't require specific pin definitions in the code. Just ensure that the sensor is connected to the correct SDA and SCL pins on your Arduino board (typically A4 and A5).