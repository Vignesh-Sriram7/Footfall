#include <Arduino.h>
#include <Wire.h>

// --- Pin & Hardware Definitions ---
#define MPU_ADDR     0x68
#define TRIG_PIN     4           // HC-SR04 TRIG
#define ECHO_PIN     5           // HC-SR04 ECHO -> via voltage divider (1k series, 2k to GND) unless 3.3 V sensor

// Uses the board's built-in LED definition (fallback to GPIO 38 if undefined)
#ifndef LED_BUILTIN
  #define LED_BUILTIN 38
#endif
const uint8_t led = LED_BUILTIN;

// MPU6050 registers
#define PWR_MGMT_1   0x6B
#define CONFIG_REG   0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// --- Ultrasonic detection settings ---
const float triggerCm = 100.0f;                 // closer than this = "motion"
const unsigned long pingInterval = 100;         // ping every 100 ms
const unsigned long timeSeconds = 20 * 1000UL;  // 20 s hold time

float distanceCm = -1.0f;                       // -1 = no echo / out of range
bool motionActive = false;
unsigned long lastTrigger = 0;
unsigned long lastPing = 0;

// --- Timing Variables ---
unsigned long now = 0;
unsigned long lastMpuRead = 0;
const unsigned long mpuInterval = 500;          // Read MPU6050 every 500 ms

// --- Ultrasonic helper ---
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30 ms timeout (~5 m)
  if (us == 0) return -1.0f;                           // no echo
  return us * 0.0343f / 2.0f;
}

// --- Wire Helper Functions ---
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);

  int16_t high = Wire.read();
  int16_t low = Wire.read();

  return (high << 8) | low;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  Serial.println("Initializing MPU6050 & Ultrasonic System...");

  // ESP32-S3 I2C pins (SDA = GPIO 8, SCL = GPIO 9)
  Wire.begin(8, 9);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Configure internal LED pin
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Wake up MPU6050
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Accelerometer: +-8 g -> 4096 LSB/g
  writeRegister(ACCEL_CONFIG, 0x10);

  // Gyroscope: +-500 deg/s -> 65.5 LSB/(deg/s)
  writeRegister(GYRO_CONFIG, 0x08);

  // Digital Low Pass Filter: 21 Hz
  writeRegister(CONFIG_REG, 0x04);

  delay(100);
  Serial.println("MPU6050 & ultrasonic sensor configured.");
}

void loop() {
  now = millis();

  // ------------------------------------------------
  // ULTRASONIC TRIGGER & LED LOGIC
  // ------------------------------------------------
  if (now - lastPing >= pingInterval) {
    lastPing = now;
    distanceCm = readDistanceCm();

    if (distanceCm > 0 && distanceCm < triggerCm) {
      lastTrigger = now;               // retrigger while object stays in range
      if (!motionActive) {
        motionActive = true;
        digitalWrite(led, HIGH);
        Serial.println("\n>>> OBJECT DETECTED! Onboard LED lit. <<<");
      }
    }
  }

  // Turn LED off after the hold time without a new detection
  if (motionActive && (now - lastTrigger > timeSeconds)) {
    motionActive = false;
    digitalWrite(led, LOW);
    Serial.println("\n>>> Timeout reached. Onboard LED off. <<<");
  }

  // ------------------------------------------------
  // MPU6050 TELEMETRY (Non-blocking every 500 ms)
  // ------------------------------------------------
  if (now - lastMpuRead >= mpuInterval) {
    lastMpuRead = now;

    int16_t rawAx = read16(ACCEL_XOUT_H);
    int16_t rawAy = read16(ACCEL_XOUT_H + 2);
    int16_t rawAz = read16(ACCEL_XOUT_H + 4);

    int16_t rawGx = read16(GYRO_XOUT_H);
    int16_t rawGy = read16(GYRO_XOUT_H + 2);
    int16_t rawGz = read16(GYRO_XOUT_H + 4);

    // Accelerometer (+-8 g -> 4096 LSB/g) in m/s^2
    float ax = (rawAx / 4096.0) * 9.80665;
    float ay = (rawAy / 4096.0) * 9.80665;
    float az = (rawAz / 4096.0) * 9.80665;

    // Gyroscope (+-500 deg/s -> 65.5 LSB/(deg/s)) in rad/s
    float gx = (rawGx / 65.5) * PI / 180.0;
    float gy = (rawGy / 65.5) * PI / 180.0;
    float gz = (rawGz / 65.5) * PI / 180.0;

    Serial.print("Accel X: ");
    Serial.print(ax, 2);
    Serial.print(" | Y: ");
    Serial.print(ay, 2);
    Serial.print(" | Z: ");
    Serial.print(az, 2);

    Serial.print("  ||  Gyro X: ");
    Serial.print(gx, 2);
    Serial.print(" | Y: ");
    Serial.print(gy, 2);
    Serial.print(" | Z: ");
    Serial.print(gz, 2);

    Serial.print("  ||  Dist: ");
    Serial.print(distanceCm, 1);
    Serial.print(" cm | State: ");
    Serial.println(motionActive ? "ACTIVE" : "IDLE");
  }
}