#include <Arduino.h>
#include <Wire.h> //I2C communication library

// Pins
#define MPU_ADDR  0x68  // My I2C device responds at the address 0x68
#define TRIG_PIN  4
#define ECHO_PIN  5           

#ifndef LED_BUILTIN
  #define LED_BUILTIN 38
#endif
const uint8_t led = LED_BUILTIN;

// MPU6050 registers
#define PWR_MGMT_1   0x6B // Wake the sensor up
#define CONFIG_REG   0x1A // Configure the digital low pass filter
#define GYRO_CONFIG  0x1B // Define the gyroscope range
#define ACCEL_CONFIG 0x1C // Define the accelerometer range
#define ACCEL_XOUT_H 0x3B // Read the accelerometer
#define GYRO_XOUT_H  0x43 // Read the gyroscope

// Detection thresholds
const float distanceThresholdCm = 80.0f;  // Object must be closer than this
const float accelThreshold      = 0.5f;         // m/s^2 deviation from 1 g counts as movement
const unsigned long movementHoldMs = 1000;      // keep "movement" true this long after last spike
const float GRAVITY = 9.80665f;
float previous_Az = 0.0, delta_Az = 0.0;
float cand_thresh = 0.20;

// Timing conditions
const unsigned long pingInterval  = 100;  // Ultrasonic sensor pinged every 100 ms
const unsigned long accelInterval = 20;   // Accelerometer read every 20 ms
const unsigned long printInterval = 500;  // Serial monitor updated every 500 ms
const unsigned long ledAlertDuration = 2000;
const unsigned long ledToggleDuration = 150;
unsigned long lastPing = 0, lastAccel = 0, lastPrint = 0;
unsigned long lastMovement = 0;
unsigned long lastFlashTime = 0, lastToggleTime = 0;

// Initialize state variables
float distanceCm = -1.0f;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float accelDeviation = 0; // Denotes how far the actual acceleration is from the actual gravity
bool nearObject = false;
bool moving = false;
bool isLedOn = false;
bool ledAlertActive = false;

// Ultrasonic sensor reader runs in the loop
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (us == 0) return -1.0f;
  return us * 0.0343f / 2.0f; // Speed of sound in the air/2
}

// Function to write into the registers at the MPU address
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);  // Defines the register that needs to be accessed
  Wire.write(value);
  Wire.endTransmission();
}

// Reads two bytes from the registers and combines to a 16-bit value
int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);  // Ends the transmission without releasing the bus
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2); // Requests two bytes from the sensor
  int16_t high = Wire.read(); // Each acceleration is stroed as high and low
  int16_t low = Wire.read();
  return (high << 8) | low; // Shift the byte to store as a 16 bit value
}

// 1g = 4096 raw units; Shifting the register by 2 enables the access to other axis values
void readMpu() {
  ax = (read16(ACCEL_XOUT_H)     / 16384.0f) * GRAVITY;
  ay = (read16(ACCEL_XOUT_H + 2) / 16384.0f) * GRAVITY;
  az = (read16(ACCEL_XOUT_H + 4) / 16384.0f) * GRAVITY;

  gx = (read16(GYRO_XOUT_H)     / 65.5f) * PI / 180.0f;
  gy = (read16(GYRO_XOUT_H + 2) / 65.5f) * PI / 180.0f;
  gz = (read16(GYRO_XOUT_H + 4) / 65.5f) * PI / 180.0f;

}

void setup() {
  Serial.begin(115200);
  // Wait for serial maximum of 3 seconds
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("Initializing MPU6050 & Ultrasonic Sensor");

  // Start the I2C communication accessing the I2C pins of the ESP32-S3
  // SDA -> 8 ; SCL -> 9
  Wire.begin(8, 9);

  // Define the trigger as output and echo as input
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Define the built in LED as output
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Write the registers to the corresponding address
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);
  writeRegister(ACCEL_CONFIG, 0x00);   // +-2 g for higher sensitivity
  writeRegister(GYRO_CONFIG, 0x08);    // +-500 deg/s
  writeRegister(CONFIG_REG, 0x04);     // DLPF 21 Hz
  delay(100);

  Serial.println("Ready. LED = distance < 80 cm AND movement");
}

void loop() {
  unsigned long now = millis(); // Update the current time

  // Checks if 20ms has passed since the last check
  if (now - lastAccel >= accelInterval) {
    lastAccel = now;
    readMpu();
    // Check the first run
    static bool firstRun = true;
    if (firstRun) {
      previous_Az = az;
      firstRun = false;
      return; 
    }

    // Since the x and y are static, just the change in az needs to be calculated
    delta_Az = fabs(az - previous_Az);
    previous_Az = az;

    // Manually measured that the peak is around 0.20 to 0.25 m/s2
    if(delta_Az >= cand_thresh)

      moving = true;

    else
      moving = false;
  }

  // Checks if 100 ms has passed since the last measurement
  if (now - lastPing >= pingInterval) {
    lastPing = now;
    distanceCm = readDistanceCm();
    // Checks the distance threshold
    nearObject = (distanceCm > 0 && distanceCm < distanceThresholdCm);
  }

  // If both conditions are met initiate the led alert
  if(nearObject && moving){
    ledAlertActive = true;
    lastFlashTime = now;
    lastToggleTime = now;
    isLedOn = true;
    neopixelWrite(led, 255, 0, 0);
    Serial.println("Intrusion Alerted!");
  }


  // Telemetry
  if (now - lastPrint >= printInterval) {
    lastPrint = now;
    Serial.print("Accel X: ");  Serial.print(ax, 2);
    Serial.print(" | Y: ");     Serial.print(ay, 2);
    Serial.print(" | Z: ");     Serial.print(az, 2);
    Serial.print("  ||  Dist: "); Serial.print(distanceCm, 1);
    Serial.print(" cm | Near: "); Serial.print(nearObject ? "Y" : "N");
    Serial.print(" | Moving: "); Serial.println(moving ? "Y" : "N");
    
  }
  // Led alert flashing red and white
  if(ledAlertActive){

    if(now - lastFlashTime >= ledAlertDuration){
      ledAlertActive = false;
      isLedOn = false;
      neopixelWrite(led, 0, 0, 0);
    }

    else if(now - lastToggleTime >= ledToggleDuration){
      lastToggleTime = now;
      isLedOn = !isLedOn;
      if(isLedOn)
        neopixelWrite(led, 255, 0, 0);
      else
        neopixelWrite(led, 0, 0, 0);

    }
  }
}