#include "util.h"
#include <Arduino.h>

//map from an input to output range linearly
double mapDouble(double input_value, double in_min, double in_max, double out_min, double out_max) {
  // if (input_value <= in_min) return out_min;
  // if (input_value >= in_max) return out_max;
  return (input_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * map from an input to output range = quadratically,
 * piecewise using two parabolas that are functions of x, where x is a value in the output range,
 * and that both share a vertex at x = center value of the output range:
 * 
 * final output value = f(x) [upward sloping parabola] for x in the upper half of the output  (x >= center value of output range),
 *                      otherwise -f(x) for x in the lower half of the output range (x < center value of output range), 
 *    and f(x) is such that:
 *        vertex occurs at x = center of output range
 *        f(x=out_max) = out_max 
 *        f(x=out_min) = out_min
 * 
 * input_value: original value in input range to map
 * in_min, in_max: lower and upper bounds of input range
 * out_min, out_max: lower and upper bounds of output range
 **/
double quadraticMapDouble(double input_value, double in_min, double in_max, double out_min, double out_max) {
  double out_range_size = out_max - out_min;
  double out_range_center = out_min + out_range_size / 2;
  double x = mapDouble(input_value, in_min, in_max, out_min, out_max);
  if (x < out_range_center) {
    return -pow((x - out_range_center), 2) / (out_range_size/2);
  } 
  else {
    return pow((x - out_range_center), 2) / (out_range_size/2);
  }
}

void printTabs(uint8_t nTabs) {
    for (uint8_t i = 0; i < nTabs; ++i) {
      if (Serial) 
        Serial.print("\t");
    }
} 