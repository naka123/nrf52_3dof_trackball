#include <Arduino.h>
#include <Adafruit_USBD_CDC.h>
#include <Adafruit_USBD_HID.h>

#include "paw3204.h"
#include "hid_3dx.h"

enum {
    MODE_TRANS2_ROT1 = 1,
    MODE_ROT_3DOF = 0,
};

uint8_t mode = MODE_ROT_3DOF;

// HID report descriptor using TinyUSB's template
// Generic In Out with 64 bytes report (max)
uint8_t const desc_hid_report[] =
        {
                TUD_HID_REPORT_DESC_MULTIAXIS_CONTROLLER( )
        };

// USB HID object
Adafruit_USBD_HID usb_hid;


// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    // not used in this example
    return 0;
}

void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    // This example doesn't use multiple report and report ID
    (void) report_id;
    (void) report_type;

    printf("set_report_callback report _id: %02x, _type: %02x, _sz: %d\n", report_id, (uint8_t)report_type, bufsize);

    if (!(report_type == HID_REPORT_TYPE_FEATURE && report_id == 0x80) ) {
        return;
    }

//    for(uint8_t i=0; i<bufsize; i++) {
//        printf("%02x ", buffer[i]);
//    }
//    printf("\n");

    mode = buffer[1];
    printf("mode changed to: %d\n", mode);

//     echo back anything we received from host
//    usb_hid.sendReport(0, buffer, bufsize);
}


void setup()
{
    usb_hid.enableOutEndpoint(true);
    usb_hid.setPollInterval(5);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.setReportCallback(get_report_callback, set_report_callback);

    usb_hid.begin();


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
uint8_t stat1, stat2, qua1, qua2;

void loop()
{

    ledOn(LED_BLUE);
    while (! usb_hid.ready() ) {
        delay(1);
    }
    ledOff(LED_BLUE);

    read_paw3204(DEV1, &stat1, &qua1, &m1x, &m1y);
    read_paw3204(DEV2, &stat2, &qua2, &m2x, &m2y);

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

    // второй сенсор повёрнут относительно первого на 180 градусов
    m2x = -m2x;
    m2y = -m2y;

//    printf("read: [1] %02x q=%-3d x=%-4d y=%-4d    [2] %02x q=%-3d x=%-4d y=%-4d\n", stat1, qua1, m1x, m1y, stat2, qua2, m2x, m2y);
    const int16_t cx = m1x + m2x;
    const int16_t cy = (m1y + m2y) / 2;
    const int16_t cz = (m1x - m2x);

//    printf("mix:  [X] CX=%-4d CY=%-4d CZ=%-4d\n", cx, cy, cz);

    if(mode == MODE_TRANS2_ROT1 ) {
        send_3dx_report1(cx, cy, cz);
    } else if( mode == MODE_ROT_3DOF ) {
        send_3dx_report2(cx, cy, cz);
    }

}




