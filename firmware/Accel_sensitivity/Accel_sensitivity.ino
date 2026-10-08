#include <Arduino.h>
#include <Wire.h> //I2C communication library

// Pins
#define MPU_ADDR  0x68  // My I2C device responds at the address 0x68

// MPU6050 registers
#define PWR_MGMT_1   0x6B // Wake the sensor up
#define CONFIG_REG   0x1A // Configure the digital low pass filter
#define GYRO_CONFIG  0x1B // Define the gyroscope range
#define ACCEL_CONFIG 0x1C // Define the accelerometer range
#define ACCEL_XOUT_H 0x3B // Read the accelerometer
#define GYRO_XOUT_H  0x43 // Read the gyroscope

const float GRAVITY = 9.80665f;
const unsigned long accelInterval = 20;   // Accelerometer read every 20 ms
const unsigned long printInterval = 500;  // Serial monitor updated every 500 ms
unsigned long lastPing = 0, lastAccel = 0, lastPrint = 0;
float previous_Az = 0.0, delta_Az = 0.0;
float cand_thresh = 0.25;
float maxIdleDelta = 0.0;
unsigned long countSpike = 0;

// Initialize state variables
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

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

  float magnitude = sqrtf(ax * ax + ay * ay + az * az);
}

void setup() {
  Serial.begin(115200);
  // Wait for serial maximum of 3 seconds
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("Initializing MPU6050 Sensor");
  

  // Start the I2C communication accessing the I2C pins of the ESP32-S3
  // SDA -> 8 ; SCL -> 9
  Wire.begin(8, 9);

  // Write the registers to the corresponding address
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);
  writeRegister(ACCEL_CONFIG, 0x00);   // +-2 g
  writeRegister(GYRO_CONFIG, 0x08);    // +-500 deg/s
  writeRegister(CONFIG_REG, 0x04);     // DLPF 21 Hz
  delay(100);

  Serial.println("Accelerometer ready");
}

void loop() {
  unsigned long now = millis();
  if (now - lastAccel >= accelInterval) {
    lastAccel = now;
    readMpu();
    static bool firstRun = true;
    if (firstRun) {
      previous_Az = az;
      firstRun = false;
      return; 
    }

    // Normal telemetry logic (runs from sample #2 onward)
    delta_Az = fabs(az - previous_Az);
    previous_Az = az;

    if (delta_Az > maxIdleDelta) {
      maxIdleDelta = delta_Az;
      Serial.print("New Noise Peak: ");
      Serial.println(maxIdleDelta);
    }

    if(delta_Az >= cand_thresh)
    {
    countSpike++;
    Serial.println("Spike count: "); 
    Serial.print(countSpike);
    Serial.println("");
    Serial.print("Delta Az: "); Serial.print(delta_Az);
    Serial.println("");
    Serial.print("Accel X: ");  Serial.print(ax, 2);
    Serial.print(" | Y: ");     Serial.print(ay, 2);
    Serial.print(" | Z: ");     Serial.print(az, 2);
    }
    
  }
}

  