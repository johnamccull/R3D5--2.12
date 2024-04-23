#include <ESP32Servo.h>
#include <UMS3.h>
#include "util.h"
#include "pinout.h"
#include "MotorDriver.h"

#define DELAY 5000 // Delay between motor movements in milliseconds
#define MAX_DEG 300
#define L_STARTING_ANGLE 0 
#define R_STARTING_ANGLE 0 

#define SERVO_1_PIN 39
#define SERVO_2_PIN 41

#define MIN_US 500
#define MAX_US 2500

MotorDriver motor(B_DIR1, B_PWM1, 0);
Servo servo1;
Servo servo2;

int degToUs (float angleOffset, float startingAngle);
void clawAngle (float angleOffset);
void turnOnMagnet(float time);

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
    motor.setup();
	servo1.setPeriodHertz(50);      // Standard 50hz servo
	servo2.setPeriodHertz(50);      // Standard 50hz servo
    servo1.attach(SERVO_1_PIN, MIN_US, MAX_US);
	servo2.attach(SERVO_2_PIN, MIN_US, MAX_US);

}

void loop() {
    clawAngle(90);
    ums3.setPixelColor(UMS3::colorWheel(0));
    delay(2000);             // waits 20ms for the servo to reach the position
	
    clawAngle(350);
    ums3.setPixelColor(UMS3::colorWheel(50));
    delay(3000);

    clawAngle(90);
    ums3.setPixelColor(UMS3::colorWheel(100));
    delay(1000);             // waits 20ms for the servo to reach the position
	
    turnOnMagnet(DELAY);
}


int degToUs (float angleOffset, float startingAngle) {
    float angle = startingAngle + angleOffset;
    return mapDouble(angle, 0, MAX_DEG, MIN_US, MAX_US);

}

// it sets the angle of the claw in the beginning 
void clawAngle (float angleOffset) {
    servo1.writeMicroseconds(degToUs(angleOffset/2,L_STARTING_ANGLE));
    servo2.writeMicroseconds(degToUs(-angleOffset/2,MAX_DEG - R_STARTING_ANGLE));
}

//Turn on the magnet function 
void turnOnMagnet (float time) {
    Serial.println("Moving Forward at full speed");
    motor.drive(1.0); // 100% duty cycle
    delay(time);

    // Stop the motor
    Serial.println("Stopping");
    motor.drive(0.0); // 0% duty cycle
    delay(time);
}


