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

// Timing conditions
const unsigned long pingInterval  = 100;  // Ultrasonic sensor pinged every 100 ms
const unsigned long accelInterval = 20;   // Accelerometer read every 20 ms
const unsigned long printInterval = 500;  // Serial monitor updated every 500 ms
unsigned long lastPing = 0, lastAccel = 0, lastPrint = 0;
unsigned long lastMovement = 0;

// --- State ---
float distanceCm = -1.0f;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float accelDeviation = 0;
bool nearObject = false;
bool moving = false;
bool ledOn = false;

// Ultrasonic sensor reader
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

void readMpu() {
  ax = (read16(ACCEL_XOUT_H)     / 4096.0f) * GRAVITY;
  ay = (read16(ACCEL_XOUT_H + 2) / 4096.0f) * GRAVITY;
  az = (read16(ACCEL_XOUT_H + 4) / 4096.0f) * GRAVITY;

  gx = (read16(GYRO_XOUT_H)     / 65.5f) * PI / 180.0f;
  gy = (read16(GYRO_XOUT_H + 2) / 65.5f) * PI / 180.0f;
  gz = (read16(GYRO_XOUT_H + 4) / 65.5f) * PI / 180.0f;

  float magnitude = sqrtf(ax * ax + ay * ay + az * az);
  accelDeviation = fabsf(magnitude - GRAVITY);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("Initializing MPU6050 & Ultrasonic System...");

  // Start the I2C communication accessing the I2C pins of the ESP32-S3
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
  writeRegister(ACCEL_CONFIG, 0x10);   // +-8 g
  writeRegister(GYRO_CONFIG, 0x08);    // +-500 deg/s
  writeRegister(CONFIG_REG, 0x04);     // DLPF 21 Hz
  delay(100);

  Serial.println("Ready. LED = distance < 80 cm AND movement.");
}

void loop() {
  unsigned long now = millis();

  // --- Accelerometer: fast sampling, movement detection ---
  if (now - lastAccel >= accelInterval) {
    lastAccel = now;
    readMpu();
    if (accelDeviation > accelThreshold) {
      lastMovement = now;
    }
    moving = (lastMovement != 0) && (now - lastMovement <= movementHoldMs);
  }

  // --- Ultrasonic: distance check ---
  if (now - lastPing >= pingInterval) {
    lastPing = now;
    distanceCm = readDistanceCm();
    nearObject = (distanceCm > 0 && distanceCm < distanceThresholdCm);
  }

  // --- LED: ON only if BOTH conditions are true ---
  bool shouldBeOn = nearObject && moving;
  if (shouldBeOn != ledOn) {
    ledOn = shouldBeOn;
    digitalWrite(led, ledOn ? HIGH : LOW);
    Serial.println(ledOn ? "\n>>> BOTH CONDITIONS MET: LED ON <<<"
                         : "\n>>> Condition lost: LED OFF <<<");
  }

  // --- Telemetry ---
  if (now - lastPrint >= printInterval) {
    lastPrint = now;
    Serial.print("Accel X: ");  Serial.print(ax, 2);
    Serial.print(" | Y: ");     Serial.print(ay, 2);
    Serial.print(" | Z: ");     Serial.print(az, 2);
    Serial.print("  ||  Dev: "); Serial.print(accelDeviation, 2);
    Serial.print("  ||  Dist: "); Serial.print(distanceCm, 1);
    Serial.print(" cm | Near: "); Serial.print(nearObject ? "Y" : "N");
    Serial.print(" | Moving: "); Serial.print(moving ? "Y" : "N");
    Serial.print(" | LED: ");   Serial.println(ledOn ? "ON" : "OFF");
  }
}