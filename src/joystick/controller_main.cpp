#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "controller_pinout.h"
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

ControllerMessage prevControllerMessage;

double JOYSTICK_DEADZONE = 0.075;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN); // forward/backward
Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN); // turn

#define V_PIN 8 // speed mode switch pin 
// #define A_PIN 9 // autonomous mode

#define SLOW_FACTOR 0.2


const int buffer_length = 15;
double joystick1_x_buffer[buffer_length] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double joystick1_y_buffer[buffer_length] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double joystick2_x_buffer[buffer_length] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double joystick2_y_buffer[buffer_length] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

unsigned long iter = 0; 
const int n = 15; // number of recent raw joystick readings to use to compute a filtered value


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
    
    EVERY_N_MILLIS(10) {
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
        joystick1_x_buffer[index_buffer] = joystick1_reading.x;
        joystick1_y_buffer[index_buffer] = joystick1_reading.y;
        joystick2_x_buffer[index_buffer] = joystick2_reading.x;
        joystick2_y_buffer[index_buffer] = joystick2_reading.y;

        // filter to smooth sudden joystick movements
        // moving average of the most recent n values (including current one) in the circular buffer
        double joystick1_x_filtered = 0;
        double joystick1_y_filtered = 0;
        double joystick2_x_filtered = 0;
        double joystick2_y_filtered = 0;
        for (int i = index_buffer - (n-1); i <= index_buffer ; i++) {
            int ind = i;
            if (ind < 0) {
                ind = buffer_length + i;
            }
            joystick1_x_filtered += joystick1_x_buffer[ind];
            joystick1_y_filtered += joystick1_y_buffer[ind];
            joystick2_x_filtered += joystick2_x_buffer[ind];
            joystick2_y_filtered += joystick2_y_buffer[ind];
        } 
        joystick1_x_filtered /= n;
        joystick1_y_filtered /= n;
        joystick2_x_filtered /= n;
        joystick2_y_filtered /= n;

        iter += 1;

        //Serial.println(digitalRead(V_PIN));        
        // if low speed mode
        if (!digitalRead(V_PIN)){
            joystick1_x_filtered *= SLOW_FACTOR;
            joystick1_y_filtered *= SLOW_FACTOR;
            joystick2_x_filtered *= SLOW_FACTOR;
            joystick2_y_filtered *= SLOW_FACTOR;
        }

        joystick1_reading.x = joystick1_x_filtered;
        joystick1_reading.y = joystick1_y_filtered;
        joystick2_reading.x = joystick2_x_filtered;
        joystick2_reading.y = joystick2_y_filtered;

        controllerMessage.joystick1 = joystick1_reading;
        controllerMessage.joystick2 = joystick2_reading;

        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}