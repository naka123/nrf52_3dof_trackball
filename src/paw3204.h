
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <Arduino.h>
#include <stdlib.h>
#include <stdint.h>
#include "quantum.h"


typedef struct {
    pin_t pin_sclk;
    pin_t pin_sdio;
} dev_pins;

const dev_pins DEV1  = {
.pin_sclk = PIN_D5,
.pin_sdio = PIN_D6
};

const dev_pins DEV2 = {
.pin_sclk = PIN_D7,
.pin_sdio = PIN_D8
};

//#ifndef PAW3204_SCLK
//#    define PAW3204_SCLK D7
//#endif
//
//#ifndef PAW3204_DATA
//#    define PAW3204_DATA D8
//#endif


const uint8_t DPI_1000 = 0b00;
const uint8_t DPI_1200 = 0b01;
const uint8_t DPI_1600 = 0b10;

typedef union {
    uint8_t reg[8];
} paw3204_all_reg;

uint8_t read_pid_paw3204(const dev_pins pins);
void    init_paw3204(const dev_pins pins);
int     read_paw3204(const dev_pins pins, uint8_t *stat, uint8_t *qua, int8_t *x, int8_t *y);
void    read_all_paw3204(const dev_pins pins, paw3204_all_reg *dat);
void    set_dpi_paw3204(const dev_pins pins, uint8_t bits);

#ifdef __cplusplus
}
#endif
