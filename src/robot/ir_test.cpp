#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

void setup() {
    Serial.begin(115200);  // Serial.begin(9600);
    //Serial.println("Setup complete");
    //Serial.println(F("AMG88xx pixels"));

    Serial.println("Setup started");
    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }

    Serial.println("Setup complete");

    delay(100); // let sensor boot up
}


void loop() { 
    //read all the pixels
    Serial.print("Looping");

    // amg.readPixels(pixels);

    // Serial.print("[");
    // for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    //   Serial.print(pixels[i-1]);
    //   Serial.print(", ");
    //   if( i%8 == 0 ) Serial.println();
    // }
    // Serial.println("]");
    // Serial.println();

    // //delay a second
    // delay(1000);
}