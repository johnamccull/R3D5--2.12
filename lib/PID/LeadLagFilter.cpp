#include "SimpleFilters.h"
#include <Arduino.h>

LeadLagFilter::LeadLagFilter(double alpha, double Td, double Ti) : _alpha(alpha), _Td(Td), _Ti(Ti), _leadFilter(alpha, Td), _lagFilter(Ti) {}

void LeadLagFilter::setParameters(double alpha, double Td, double Ti) {
    _alpha = alpha;
    _Td = Td;
    _Ti = Ti;
    _leadFilter.setParameters(alpha, Td);
    _lagFilter.setParameters(Ti);
}

double LeadLagFilter::calculate(double input) {
    double lagOutput = _lagFilter.calculate(input);
    return _leadFilter.calculate(lagOutput);
}