#include <Arduino.h>
#include <Adafruit_USBD_CDC.h>

#include "paw3204.h"

#define CFG_TUD_HID 1

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

//    pinMode(D7, OUTPUT);
//    digitalWrite(D7, 0);

    init_paw3204();

    Serial.begin(115200);
    printf("nrf52 mouse test\n");
    printf("---------------------------\n");

}

paw3204_all_reg dat;

void loop()
{
//    digitalWrite(LED_BUILTIN, HIGH);
//    delay(10);
//    digitalWrite(LED_BUILTIN, LOW);
//    delay(10);

//    digitalWrite(D7, 1);
//    delayMicroseconds(20);
//    digitalWrite(D7, 0);


    read_all_paw3204(&dat);

    printf("regs: ");
    for (uint8_t i=0; i<8; i++) {
        printf("%d: %02x ", i, dat.reg[i]);
    }
    printf("\n");

    delay(20);

}