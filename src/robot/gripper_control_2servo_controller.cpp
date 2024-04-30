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

#define TEMP_THRESHOLD 30

#define CLAW_OPEN "v"
#define CLAW_CLOSE "c"
#define MAG_ON "m"
#define MAG_OFF "n"

#include <Wire.h> 
#include <Adafruit_AMG88xx.h>
#define TCAADDR 0x70
Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

MotorDriver electroMagnet(B_DIR1, B_PWM1, 0);
Servo servo1;
Servo servo2;

int degToUs (float angleOffset, float startingAngle);
void clawAngle (float angleOffset);
void turnOnMagnet (bool on);

void IR_test();
void PIXELS();
void tcaselect(uint8_t i);
bool see_hot;

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
    electroMagnet.setup();
	servo1.setPeriodHertz(50);      // Standard 50hz servo
	servo2.setPeriodHertz(50);      // Standard 50hz servo
    servo1.attach(SERVO_1_PIN, MIN_US, MAX_US);
	servo2.attach(SERVO_2_PIN, MIN_US, MAX_US);

}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove any trailing newline or whitespace

        if (command == CLAW_OPEN) {  // Open claw
            clawAngle(90);  // Open claw angle
            Serial.println("Claw Opened");
        } else if (command == CLAW_CLOSE) { // Close claw
            clawAngle(350);   // Close claw angle
            Serial.println("Claw Closed");
        } else if (command == MAG_ON) { // Turn on electromagnet
            turnOnMagnet(true);
            Serial.println("Electromagnet On");
        } else if (command == MAG_OFF) { // Turn off electromagnet
            turnOnMagnet(false);
            Serial.println("Electromagnet Off");
        } else {
            Serial.println("Unknown command");
        }
        if (millis()%1000 == 100){PIXELS();}
    }
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

// Turns the electromagnet on or off
void turnOnMagnet (bool on) {
    if (on) {
        electroMagnet.drive(1.0); // 100% duty cycle for ON
    } else {
        electroMagnet.drive(0.0); // 0% duty cycle for OFF
    }
}


void IR_test() {
    Serial.begin(115200);
    Serial.println(F("AMG88xx pixels"));
    Wire.begin();
    tcaselect(0);

    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    
    Serial.println("-- Pixels Test --");

    Serial.println();
    
    delay(100); // let sensor boot up
}


void PIXELS() { 
    //read all the pixels
    amg.readPixels(pixels);

    Serial.print("[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      Serial.print(pixels[i-1]);
      if(pixels[i-1]> TEMP_THRESHOLD){see_hot = true;}
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    Serial.println();
    if(see_hot) Serial.println("I SEE SOMETHING HOT");
    //delay a second
    delay(100);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

