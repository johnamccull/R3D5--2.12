#include <ESP32Servo.h>
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <UMS3.h>
#include "util.h"
#include "pinout.h"
#include "MotorDriver.h"
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <iostream>
using namespace std;


// #define DELAY 5000 // Delay between motor movements in milliseconds
// #define MAX_DEG 300
// #define L_STARTING_ANGLE 0 
// #define R_STARTING_ANGLE 0 

// #define SERVO_1_PIN 39
// #define SERVO_2_PIN 41

// #define MIN_US 500
// #define MAX_US 2500

#define TEMP_THRESHOLD 30
#define PRINT_TEMPS 0

// #define CLAW_OPEN "v"
// #define CLAW_CLOSE "c"
// #define MAG_ON "m"
// #define MAG_OFF "n"

// #define CLAW_OPEN_ANG 160 //120 //40 //80
// #define CLAW_CLOSE_ANG 350

#include <Wire.h> 
#include <Adafruit_AMG88xx.h>
#define TCAADDR 0x70
Adafruit_AMG88xx amg;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

//ToF Setup
#define DEV_I2C Wire
#define SerialPort Serial
// Components.
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C,B_PWM2);
void tofsetup();
void tofdata(); 
void getmode();

// MotorDriver electroMagnet(B_DIR1, B_PWM1, 0);
// Servo servo1;
// Servo servo2;

// int degToUs (float angleOffset, float startingAngle);
// void clawAngle (float angleOffset);
// void turnOnMagnet (bool on);

void irSetup();
void irRead();
bool see_hot; // = false;
//float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
//float threshold = 30;

// UMS3 ums3;

// int pos = 0;      // position in degrees
// ESP32PWM pwm;

enum Mode {
    OFF, 
    IR, 
    DISTANCE
};




void setup() {
    Serial.begin(115200);
    
//     ums3.begin();
//   // Brightness is 0-255. We set it to 1/3 brightness here
//     ums3.setPixelBrightness(255 / 3);    

// 	// Allow allocation of all timers
// 	ESP32PWM::allocateTimer(0);
// 	ESP32PWM::allocateTimer(1);
	
//     electroMagnet.setup();
// 	servo1.setPeriodHertz(50);      // Standard 50hz servo
// 	servo2.setPeriodHertz(50);      // Standard 50hz servo
//     servo1.attach(SERVO_1_PIN, MIN_US, MAX_US);
// 	servo2.attach(SERVO_2_PIN, MIN_US, MAX_US);

    // Setup IR sensor
    irSetup();
    // Setup ToF sensor 
    tofsetup();

    delay(100); // let sensors boot up

    Serial.println("Gripper setup complete");
}

void loop() {
    // This only runs if there is an incoming command through the serial port: https://www.arduino.cc/reference/en/language/functions/communication/serial/available/       
    if (Serial.available() > 0) { 
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove any trailing newline or whitespace

        // if (command == CLAW_OPEN) {  // Open claw
        //     clawAngle(CLAW_OPEN_ANG);  // Open claw angle 90
        //     Serial.println("Claw Opened");
        // } else if (command == CLAW_CLOSE) { // Close claw
        //     clawAngle(CLAW_CLOSE_ANG);   // Close claw angle 350
        //     Serial.println("Claw Closed");
        // } else if (command == MAG_ON) { // Turn on electromagnet
        //     turnOnMagnet(true);
        //     Serial.println("Electromagnet On");
        // } else if (command == MAG_OFF) { // Turn off electromagnet
        //     turnOnMagnet(false);
        //     Serial.println("Electromagnet Off");
        // } else {
        //     Serial.println("Unknown command");
        }

    // This runs every 1 second
    if (millis()%1000 == 0){irRead();}
    tofdata();
}


// int degToUs (float angleOffset, float startingAngle) {
//     float angle = startingAngle + angleOffset;
//     return mapDouble(angle, 0, MAX_DEG, MIN_US, MAX_US);

// }

// it sets the angle of the claw in the beginning 
// void clawAngle (float angleOffset) {
//     servo1.writeMicroseconds(degToUs(angleOffset/2,L_STARTING_ANGLE));
//     servo2.writeMicroseconds(degToUs(-angleOffset/2,MAX_DEG - R_STARTING_ANGLE));
// }

// // Turns the electromagnet on or off
// void turnOnMagnet (bool on) {
//     if (on) {
//         electroMagnet.drive(1.0); // 100% duty cycle for ON
//     } else {
//         electroMagnet.drive(0.0); // 0% duty cycle for OFF
//     }
// }

void irSetup() {
    //Serial.begin(115200);
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
    
    Serial.println("-- IR setup --");
    //Serial.println();
    
    // delay(100); // let sensor boot up
}

void irRead() {   
    //read all the pixels
    see_hot = false;
    amg.readPixels(pixels);

    // Print the temperatures if the flag is set (open bracket for array)
    if(PRINT_TEMPS){
        Serial.print("[");
    }

    // Loop through all pixels returned from the ir sensor
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
        // If the temperature is above the threshold, set the flag
        if(pixels[i-1]> TEMP_THRESHOLD){see_hot = true;}
        
        // Print the temperatures if the flag is set
        if(PRINT_TEMPS){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) {Serial.println();}
        }
    }

    // Print the temperatures if the flag is set (close bracket for array)
    if(PRINT_TEMPS){
        Serial.println("]");
        Serial.println();
    }

    if(see_hot) Serial.println("I SEE SOMETHING HOT");
    //delay a second
    //delay(1000);   
}

void tofsetup()
{
  
  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();
  tcaselect(1);
  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();

  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  //Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(0x12);

  // Start Measurements
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
}

int readTOF()
{
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

    return pMultiRangingData->RangeData[0].RangeMilliMeter;
    // if (status == 0) {
    //   status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    // }
  }
}

void getmode() {
    if (Serial.available() > 0) {
        char receivedChar = Serial.read(); 
        switch (receivedChar) {
            case '0': 
                currentMode = OFF; 
                Serial.println("Mode set to OFF");
                break; 
            case '1':
                currentMode = IR;
                Serial.println("Mode set to DISTANCE");
                break; 
            default: 
                Serial.println("Invalid mode");
                break;
        }
    }
}

Mode currentMode = OFF;

void loop() {
  // read the sensor:
    getmode();
    switch (currentMode) {
    case IR:
        irRead();
      break;
    case DISTANCE:
      int dist = readTOF();
      if (dist < 15) {
        Serial.println("CLOSE");
      }
      break;
    case OFF:
      break;
      }
    }
