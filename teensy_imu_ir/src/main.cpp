#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// [!] ADAPTATION NOTES IN CAPS [!]
#define BNO055_I2C_ADDRESS 0x28  // DEFAULT I2C ADDRESS (0x29 IF COM PIN PULLED HIGH)
#define UPDATE_RATE 50           // Hz (MATCH ROS2 NODE EXPECTATIONS)

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_I2C_ADDRESS);

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}  // WAIT FOR SERIAL IN TEENSY USB MODE

  // [!] SENSOR INITIALIZATION CHECK [!]
  if (!bno.begin()) {
    Serial.println("BNO055 NOT DETECTED!");
    while (1);  // HALT IF INIT FAILS
  }
  
  // [!] CALIBRATION IMPORTANT - SEE NOTE BELOW [!]
  bno.setExtCrystalUse(true);  // USE EXTERNAL 32.768KHZ CRYSTAL IF PRESENT
  
  // CONFIGURE SENSOR MODE
  bno.setMode(OPERATION_MODE_NDOF);  // 9-DOF FUSION MODE
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  
  if (now - lastUpdate >= (1000/UPDATE_RATE)) {
    lastUpdate = now;
    
    // Get all sensor data
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // Send formatted data
    Serial.print("IMU:");
    Serial.print(quat.x(), 4); Serial.print(",");
    Serial.print(quat.y(), 4); Serial.print(",");
    Serial.print(quat.z(), 4); Serial.print(",");
    Serial.print(quat.w(), 4); Serial.print(",");
    Serial.print(gyro.x(), 4); Serial.print(",");
    Serial.print(gyro.y(), 4); Serial.print(",");
    Serial.print(gyro.z(), 4); Serial.print(",");
    Serial.print(accel.x(), 4); Serial.print(",");
    Serial.print(accel.y(), 4); Serial.print(",");
    Serial.println(accel.z(), 4);
  
    // // [!] OPTIONAL - CALIBRATION STATUS MONITORING [!]
    // uint8_t sys, gyro, accel, mag = 0;
    // bno.getCalibration(&sys, &gyro, &accel, &mag);
    // Serial.print("CALIB: ");
    // Serial.print(sys);   // SYSTEM (3=FULLY CALIBRATED)
    // Serial.print(",");
    // Serial.print(gyro);  // GYROSCOPE
    // Serial.print(",");
    // Serial.print(accel); // ACCELEROMETER
    // Serial.print(",");
    // Serial.println(mag); // MAGNETOMETER
  }
}