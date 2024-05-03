#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

ControllerMessage prevControllerMessage;
double JOYSTICK_DEADZONE = 0.06;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN); // forward/backward
Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN); // turn
#define V_PIN 8// speed mode
// #define A_PIN 9 // autonomous mode
#define SLOW_FACTOR 0.2

void setup() {
    Serial.begin(115200);

    setupWireless();

    joystick1.setup();
    joystick2.setup();

    Serial.println("Setup complete.");
    pinMode(V_PIN, INPUT_PULLUP);
    // pinMode(A_PIN, INPUT_PULLUP);
}

void loop() {
    // Read and send controller sensors
    
    EVERY_N_MILLIS(5) {
        //controllerMessage.millis = millis();

        JoystickReading joystick1_reading = joystick1.read();
        JoystickReading joystick2_reading = joystick2.read();

        if (abs(joystick1_reading.x) < JOYSTICK_DEADZONE) {
            joystick1_reading.x = 0;
        }

        if (abs(joystick2_reading.y) < JOYSTICK_DEADZONE) {
            joystick2_reading.y = 0;
        }

        // not dealing with joystick1_reading.y and joystick2_reading.x since those are not used by robot

        // high speed mode (default)
        controllerMessage.joystick1 = joystick1_reading;
        controllerMessage.joystick2 = joystick2_reading;


        Serial.println(digitalRead(V_PIN));        

        //if low speed mode
        if (!digitalRead(V_PIN)){
            controllerMessage.joystick1.x *= SLOW_FACTOR;
            controllerMessage.joystick1.y *= SLOW_FACTOR;
            controllerMessage.joystick2.x *= SLOW_FACTOR;
            controllerMessage.joystick2.y *= SLOW_FACTOR;
        }

        
        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}