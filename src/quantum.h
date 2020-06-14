//
// Created by user on 14-Jun-20.
//

#include "WVariant.h"

#ifndef NRF52_TEST_COMPAT_H
#define NRF52_TEST_COMPAT_H

typedef uint8_t pin_t;

#define setPinInput(pin) pinMode(pin, INPUT)
#define setPinInputHigh(pin) pinMode(pin, INPUT_PULLUP)
#define setPinInputLow(pin) pinMode(pin, INPUT_PULLDOWN)
#define setPinOutput(pin) pinMode(pin, OUTPUT)

#define writePinHigh(pin) digitalWrite(pin, 1)
#define writePinLow(pin) digitalWrite(pin, 0)
#define writePin(pin, level) digitalWrite(pin, level)

#define readPin(pin) digitalRead(pin)

#define _delay_us delayMicroseconds

#endif //NRF52_TEST_COMPAT_H
