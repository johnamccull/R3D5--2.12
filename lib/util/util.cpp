#include "util.h"
#include <Arduino.h>

//map from an input to output range linearly
double mapDouble(double input_value, double in_min, double in_max, double out_min, double out_max) {
  // if (input_value <= in_min) return out_min;
  // if (input_value >= in_max) return out_max;
  return (input_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * map from an input to output range quadratically.
 * 
 * NOTE: input and output ranges both must be symmetric about 0.
 * 
 * map is piecewise using two parabolas that are functions of x, where x is a value in the output range:
 * 
 * final output value = f(x) [upward sloping parabola] for x in the upper half of the output  (x >= 0),
 *                      otherwise -f(x) for x in the lower half of the output range (x < 0), 
 *    and f(x) is such that:
 *        vertex is f(x = 0) = 0
 *        f(x=out_max) = abs(out_max)
 *        f(x=out_min) = -abs(out_min)
 * 
 * input_value: original value in input range to map
 * in_bound: positive value, lower and upper bounds of input range are -in_bound and in_bound respectively
 * out_bound, out_max: positive value, lower and upper bounds of output range are -out_bound and out_bound respectively
 **/
double quadraticMapDouble(double input_value, double in_bound, double out_bound) {
  double out_range_size = out_bound * 2;
  double out_range_center = -out_bound + out_range_size / 2;
  double x = mapDouble(input_value, -in_bound, in_bound, -out_bound, out_bound);
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