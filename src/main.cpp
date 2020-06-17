#include <Arduino.h>
//#include <Adafruit_USBD_CDC.h>
//#include <Adafruit_USBD_HID.h>

#include "paw3204.h"
#include "hid_3dx.h"

#include "RotaryEncoder.h"

// HID report descriptor using TinyUSB's template
// Generic In Out with 64 bytes report (max)
uint8_t const desc_hid_report[] =
        {
                TUD_HID_REPORT_DESC_MULTIAXIS_CONTROLLER( )
        };

//Encoder myEnc(PIN_D19, PIN_D20);

// USB HID object
Adafruit_USBD_HID usb_hid;

long oldPosition  = -999;

#define PIN_ENC_BUTTON PIN_D21

void setup()
{
// board_config.update("build.hwids", [
//    ["0x046D",  "0xC62b" ]
//])
//
//board_config.update("vendor", "Naka")
//board_config.update("build.usb_product", "SpaceMouse PRO")

    usb_hid.enableOutEndpoint(true);
    usb_hid.setPollInterval(5);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.setReportCallback(get_report_callback, set_report_callback);

    usb_hid.begin();

    Serial.begin(115200);
    printf("nrf52 mouse test\n");
    printf("---------------------------\n");

    init_paw3204(DEV1);
    if( read_pid_paw3204(DEV1) != SENSOR_PRODUCT_ID1 ) {
        Serial.println("failed to init SENSOR1 !!!");
    }
    set_dpi_paw3204(DEV1, DPI_1600);

    init_paw3204(DEV2);
    if( read_pid_paw3204(DEV2) != SENSOR_PRODUCT_ID1 ) {
        Serial.println("failed to init SENSOR2 !!!");
    }
    set_dpi_paw3204(DEV2, DPI_1600);

    // Initialize Encoder
    RotaryEncoder.begin(PIN_D19, PIN_D20);
    // Start encoder
    RotaryEncoder.start();

    setPinInput(PIN_ENC_BUTTON);

    ledOn(LED_GREEN);

}

paw3204_all_reg dat;

int8_t m1x, m1y, m2x, m2y;
uint8_t stat1, stat2, qua1, qua2;

int encoder_value = 0;

void loop()
{
//    ledOn(LED_BLUE);
    while (! usb_hid.ready() ) {
        delay(2);
    }
//    ledOff(LED_BLUE);

    int enc_delta = RotaryEncoder.read();
    if (enc_delta) {
        encoder_value += enc_delta;
        printf("encoder: %d\n", encoder_value);
        if(translation_mode == MODE_ROT_3DOF ) {
            SENSOR_R_SCALE += 0.05f * (float) (enc_delta);
            if (SENSOR_R_SCALE < 0.2) {
                SENSOR_R_SCALE = 0.2;
            } else if (SENSOR_R_SCALE > 5) {
                SENSOR_R_SCALE = 5;
            }
        }
    }

    read_paw3204(DEV1, &stat1, &qua1, &m1x, &m1y);
    read_paw3204(DEV2, &stat2, &qua2, &m2x, &m2y);

    // второй сенсор повёрнут относительно первого на 180 градусов
    m2x = -m2x;
    m2y = -m2y;

//    printf("read: [1] %02x q=%-3d x=%-4d y=%-4d    [2] %02x q=%-3d x=%-4d y=%-4d\n", stat1, qua1, m1x, m1y, stat2, qua2, m2x, m2y);
    const int16_t cx = m1x + m2x;
    const int16_t cy = (m1y + m2y) / 2;
    const int16_t cz = (m1x - m2x);

//    printf("mix:  [X] CX=%-4d CY=%-4d CZ=%-4d\n", cx, cy, cz);

    hid_3dx_report_6dof_t report;

    if(translation_mode == MODE_TRANS2_ROT1 ) {
        map_as_2T1Rdof(cx, cy, enc_delta, &report);
        send_3dx_report_6dof(&report);
    } else if(translation_mode == MODE_ROT_3DOF ) {
        map_as_3Rdof_and_zoom(cx, cy, cz, 0, &report);
        send_3dx_report_6dof(&report);
    }

    send_3dx_report_buttons(digitalRead(PIN_ENC_BUTTON)? 1<<31: 0);


}




