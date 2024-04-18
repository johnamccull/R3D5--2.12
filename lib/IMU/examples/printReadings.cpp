#include <Arduino.h>
#include "imu.h"
#include "EulerAngles.h"

#define IMU_RST 9
#define IMU_CS 10
#define IMU_INT 11

IMU imu(IMU_RST, IMU_CS, IMU_INT);

void setup() {
  imu.setup();
  Serial.begin(115200);
}


unsigned long previousMillis = 0;
const unsigned long interval = 100; // 100 ms interval

void loop() {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Update the previousMillis value
        imu.update();
        imu.readIMU();
        printEulerDeg(imu.getEulerAngles());
        // printGyroDeg(imu.getGyroReadings());
    }
}

