#include <Arduino.h>
#include "wireless.h"
#include "util.h"

#define PRINT_DELAY 250


void setup() {
    Serial.begin(115200);
    setupWireless();
}

void loop(){
    int input_waypoints[2][3] = {{1, 2, 3},{1, 1, 1}};
        Serial.printf("input waypoints size is %.2f\n", sizeof(input_waypoints));
        /*for (int i=0; i>sizeof(input_waypoints); i++){
            Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n", robotMessage.x, robotMessage.y, robotMessage.theta);
        
        }*/
    
    
}



