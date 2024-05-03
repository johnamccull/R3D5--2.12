#ifndef UTIL_H 
#define UTIL_H

#include "EveryNMicros.h"
#include "EveryNMillis.h"

double mapDouble(double input_value, double in_min, double in_max, double out_min, double out_max);
double quadraticMapDouble(double input_value, double in_bound, double out_bound);
void printTabs(uint8_t nTabs);

#endif // UTIL_H
