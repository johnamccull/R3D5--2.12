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

const int buffer_length = 8;
double joystick1_buffer[buffer_length] = {0,0,0,0,0,0,0,0};
double joystick2_buffer[buffer_length] = {0,0,0,0,0,0,0,0};
unsigned long iter = 0; 
const int n = 3; // number of recent raw joystick readings to use to compute a filtered value


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
        if (abs(joystick1_reading.y) < JOYSTICK_DEADZONE) {
            joystick1_reading.y = 0;
        }
        if (abs(joystick2_reading.x) < JOYSTICK_DEADZONE) {
            joystick2_reading.x = 0;
        }
        if (abs(joystick2_reading.y) < JOYSTICK_DEADZONE) {
            joystick2_reading.y = 0;
        }        

        // insert readings into buffers
        int index_buffer = iter % buffer_length;
        joystick1_buffer[index_buffer] = joystick1_reading.x;
        joystick2_buffer[index_buffer] = joystick2_reading.y;
        iter += 1;

        // filter
        // for now just moving average on the most recent n values
        double joystick1_x_filtered = joystick1_reading.x;
        if (index_buffer >= n-1) {
            joystick1_x_filtered = 0;
            for (int i = index_buffer - (n-1); i <= index_buffer ; i++) {
                joystick1_x_filtered += joystick1_buffer[i];
            } 
            joystick1_x_filtered /= n;
        }

        Serial.println(digitalRead(V_PIN));        
        // if low speed mode
        if (!digitalRead(V_PIN)){
            joystick1_reading.x *= SLOW_FACTOR;
            //joystick1_x_filtered *= SLOW_FACTOR;
            joystick1_reading.y *= SLOW_FACTOR;
            joystick2_reading.x *= SLOW_FACTOR;
            joystick2_reading.y *= SLOW_FACTOR;
        }

        controllerMessage.joystick1 = joystick1_reading;
        controllerMessage.joystick2 = joystick2_reading;

        

        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}