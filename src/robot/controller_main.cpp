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

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);
Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN);

void setup() {
    Serial.begin(115200);

    setupWireless();

    joystick1.setup();
    //joystick2.setup();

    Serial.println("Setup complete.");
}

void loop() {
    // Read and send controller sensors
    EVERY_N_MILLIS(5) {
        controllerMessage.millis = millis();
        controllerMessage.joystick1 = joystick1.read();
        //controllerMessage.joystick2 = joystick2.read();
        
        
        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}