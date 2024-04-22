#include <Arduino.h>
#include "pinout.h"
#include "MotorDriver.h"

#define DELAY 5000 // Delay between motor movements in milliseconds


// Create an instance of the MotorDriver class
MotorDriver motor(B_DIR1, B_PWM1, 0);

void setup() {
    // Initialize serial communication
    Serial.begin();

    // Setup the motor driver
    motor.setup();
}

void loop() {
    // Move the motor forward at full speed
    Serial.println("Moving Forward at full speed");
    motor.drive(1.0); // 100% duty cycle
    delay(DELAY);

    // Stop the motor
    Serial.println("Stopping");
    motor.drive(0.0); // 0% duty cycle
    delay(DELAY);

   
}


