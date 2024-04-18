// EveryNMillis.h
#ifndef EVERY_N_MILLIS_H
#define EVERY_N_MILLIS_H

#include <Arduino.h>

class EveryNMillis {
public:
    EveryNMillis(unsigned long interval) : interval(interval), prevMillis(0) {}

    bool update() {
        unsigned long currentMillis = millis();
        if ((unsigned long)(currentMillis - prevMillis) >= interval) {
            prevMillis = currentMillis;
            return true;
        }
        return false;
    }

private:
    unsigned long interval;
    unsigned long prevMillis;
};

#define EVERY_N_MILLIS(n) for (static EveryNMillis _timer(n); _timer.update(); )

#endif // EVERY_N_MILLIS_H
