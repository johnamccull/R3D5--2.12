#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include "pinout.h"
#include "util.h"

#define DEV_I2C Wire
#define SerialPort Serial


// Components.
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C,B_PWM2);

/* Setup ---------------------------------------------------------------------*/

void setup()
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

void loop()
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
    //snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    //SerialPort.print(report);
    for (j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        //SerialPort.print("\r\n                               ");
      }
      //SerialPort.print("status=");
      //SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
      SerialPort.print("D=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      SerialPort.print("mm");
      //SerialPort.print(", Signal=");
      //SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
      //SerialPort.print(" Mcps, Ambient=");
      //SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
      //SerialPort.print(" Mcps");
    }
    SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

}