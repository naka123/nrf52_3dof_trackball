#include <Arduino.h>
#include <Adafruit_USBD_CDC.h>

#include "paw3204.h"

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    init_paw3204(DEV1);
    set_dpi_paw3204(DEV1, DPI_1600);

    init_paw3204(DEV2);
    set_dpi_paw3204(DEV2, DPI_1600);

    Serial.begin(115200);
    printf("nrf52 mouse test\n");
    printf("---------------------------\n");

}

paw3204_all_reg dat;

int8_t m1x, m1y, m2x, m2y;
uint8_t stat1, stat2;


void loop()
{
    read_paw3204(DEV1, &stat1, &m1x, &m1y);
    read_paw3204(DEV2, &stat2, &m2x, &m2y);

    if (stat1&0x80 && (m1x || m1y)) {
        ledOn(LED_RED);
    } else {
        ledOff(LED_RED);
    };

    if (stat2&0x80 && (m2x || m2y)) {
        ledOn(LED_GREEN);
    } else {
        ledOff(LED_GREEN);
    };


    printf("read: [1] %02x x=%-4d y=%-4d    [2] %02x x=%-4d y=%-4d\n", stat1, m1x, m1y, stat2, m2x, m2y);


    delay(20);

}