#include <ESP32Servo.h>
#include <UMS3.h>
#include "util.h"

#define MAX_DEG 180
#define L_STARTING_ANGLE 0 

#define SERVO_1_PIN 5

#define MIN_US 500
#define MAX_US 2500

Servo servo1;

int degToUs (float angleOffset, float startingAngle);
void clawAngle (float angleOffset);
UMS3 ums3;

int pos = 0;      // position in degrees
ESP32PWM pwm;

void setup() {
    ums3.begin();
  // Brightness is 0-255. We set it to 1/3 brightness here
    ums3.setPixelBrightness(255 / 3);    

	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	Serial.begin(115200);
	servo1.setPeriodHertz(50);      // Standard 50hz servo
    servo1.attach(SERVO_1_PIN, MIN_US, MAX_US);
      // Initialize all board peripherals, call this first


}

void loop() {
    clawAngle(45); // This is OPEN configuration
    ums3.setPixelColor(UMS3::colorWheel(0));
    delay(2000);             
	
    clawAngle(160); // This is CLOSED configuration 
    ums3.setPixelColor(UMS3::colorWheel(50));
    delay(5000);
	
	
}


int degToUs (float angleOffset, float startingAngle) {
    float angle = startingAngle + angleOffset;
    return mapDouble(angle, 0, MAX_DEG, MIN_US, MAX_US);

}

// it sets the angle of the claw in the beginning 
void clawAngle (float angleOffset) {
    servo1.writeMicroseconds(degToUs(angleOffset/2,L_STARTING_ANGLE));
}

// hello this is a test