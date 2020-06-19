#include <string.h>
#include <Arduino.h>
#include <SEGGER_SYSVIEW.h>
#include <Adafruit_USBD_CDC.h>
#include <Adafruit_USBD_HID.h>
#include <SEGGER_RTT.h>
#include "RotaryEncoder.h"

#include "paw3204.h"
#include "hid_3dx.h"
#include "gyro.h"



// HID report descriptor using TinyUSB's template
// Generic In Out with 64 bytes report (max)
uint8_t const desc_hid_report[] =
        {
                TUD_HID_REPORT_DESC_MULTIAXIS_CONTROLLER( )
        };

//Encoder myEnc(PIN_D19, PIN_D20);

// USB HID object
Adafruit_USBD_HID usb_hid;

#define STACK_SZ       (256*4)

static TaskHandle_t  _bsensHandle;

[[noreturn]] static void ball_task(void* arg) {
    (void) arg;

    while(true) {
        yield();
        delay(1);
    }

}

static TaskHandle_t  _gyroHandle;

[[noreturn]] static void gyro_task(void* arg) {
    (void) arg;

    Gyro.init();
    Gyro.enableSendRawGyro(true);

    while(true) {
        Gyro.update_from_mpu();
//        yield();
        delay(4);
    }

}


#define PIN_ENC_BUTTON PIN_D21

uint8_t prev_btn;

void setup()
{
// board_config.update("build.hwids", [
//    ["0x046D",  "0xC62b" ]
//])
//
//board_config.update("vendor", "Naka")
//board_config.update("build.usb_product", "SpaceMouse PRO")

    usb_hid.enableOutEndpoint(true);
    usb_hid.setPollInterval(4);
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

    pinMode(PIN_ENC_BUTTON, INPUT);
    prev_btn = digitalRead(PIN_ENC_BUTTON);

    ledOn(LED_GREEN);

    SEGGER_RTT_printf(0, "setup end\n");


    xTaskCreate(ball_task, "ball", STACK_SZ, NULL, TASK_PRIO_LOW, &_bsensHandle);
    xTaskCreate(gyro_task, "gyro", STACK_SZ, NULL, TASK_PRIO_LOW, &_gyroHandle);

}





paw3204_all_reg dat;

int8_t m1x, m1y, m2x, m2y;
uint8_t stat1, stat2, qua1, qua2;

int encoder_value = 0;
long oldPosition  = -999;

//enum {
//    BTN_RELEASED = 0,
//    BTN_JUST_PRESSED,
//    BTN_PRESSED,
//    BTN_JUST_RELEASED,
//};

//uint8_t btn_state = BTN_RELEASED;
//uint btn_ticks = 0;
//
//void upd_btn() {
//    switch (btn_state) {
//        case BTN_RELEASED:
//            if (digitalRead(PIN_ENC_BUTTON)) {
//                btn_state = BTN_JUST_PRESSED;
//                btn_ticks = 0
//            }
//            break;
//
//        case BTN_JUST_PRESSED:
//            if (digitalRead(PIN_ENC_BUTTON)) {
//                if (btn_ticks > 10) {
//                    btn_state = BTN_PRESSED;
//                } else {
//                    btn_ticks += 1;
//                }
//            } else {
//                btn_state = BTN_RELEASED;
//            }
//            break;
//
//        case BTN_PRESSED:
//            break;
//
//        case BTN_JUST_RELEASED:
//            break;
//    }
//}


float prev_angles[3] = {0,0,0};

uint32_t do_encoder();

bool zero_motion_sent = true;

const int USB_WAIT_DELAY_MS = 2;

uint32_t t_gyro_send_prev = 0;

void loop()
{
//    SEGGER_RTT_printf(0, "loop tick\n");

    uint32_t enc_delta = do_encoder();
//    ledOn(LED_BLUE);
//    ledOff(LED_BLUE);

    while (! usb_hid.ready() ) {
        delay(USB_WAIT_DELAY_MS);
    }

    uint32_t m = millis();

    if ( m - t_gyro_send_prev > 10 ) {

        t_gyro_send_prev = m;

        float xyz[3];
        if (Gyro.get_delta_angles(xyz) && translation_mode != MODE_ROT_2DOF) {
//        printf("angles loop: %5.2f\t%5.2f\t%5.2f\n", xyz[0], xyz[1], xyz[2]);

            int16_t dx = xyz[1] * 10;
            int16_t dy = -xyz[2] * 10;
            int16_t dz = xyz[0] * 10;

            if (dx != 0 || dy != 0 || dz != 0) {
                send_motion(dx, dy, dz, 0);

//                printf("send_motion: %5d\t%5d\t%5d\n", dx, dy, dz);

                while (!usb_hid.ready()) {
                    delay(USB_WAIT_DELAY_MS);
                }
            }

        }
    }

    read_paw3204_status(DEV1, &stat1);
    read_paw3204_status(DEV2, &stat2);

    // если никаких изменений нет - можно ничего не посылать
    if (stat1 & 0x80 || stat2 & 0x80) {

        zero_motion_sent = false;

        yield();
        read_paw3204_data(DEV1, &qua1, &m1x, &m1y);
        yield();
        read_paw3204_data(DEV2, &qua2, &m2x, &m2y);

        // второй сенсор повёрнут относительно первого на 180 градусов
        m2x = -m2x;
        m2y = -m2y;

        const int16_t cx = m1x + m2x;
        const int16_t cy = (m1y + m2y) / 2;
        const int16_t cz = (m1x - m2x);

        send_motion(cx, cy, cz, enc_delta);

    } else if (!zero_motion_sent) {
        send_motion(0, 0, 0, 0);
        zero_motion_sent = true;
    } else {
        delay(1);
    }


}

uint32_t do_encoder() {
    uint8_t cur_btn = digitalRead(PIN_ENC_BUTTON);
    if (cur_btn != prev_btn) {
        prev_btn = cur_btn;
        SEGGER_RTT_printf(0, "enc button %s\n", cur_btn ? "pressed" : "released");
//        send_3dx_report_buttons(cur_btn ? 0 : V3DK_1 );
//        return;

        if (cur_btn) {

            Gyro.reset_origin();

            ledOff(LED_RED);
            ledOff(LED_GREEN);
            ledOff(LED_BLUE);;

            switch (translation_mode) {
                case MODE_ROT_3DOF:
                    translation_mode = MODE_TRANS2_ROT1;
                    ledOn(LED_BLUE);
                    break;
                case MODE_TRANS2_ROT1:
                    translation_mode = MODE_ROT_2DOF;
                    ledOn(LED_RED);
                    break;
                case MODE_ROT_2DOF:
                    translation_mode = MODE_ROT_3DOF;
                    ledOn(LED_GREEN);
                    break;
            }
        }

    }

    int enc_delta = RotaryEncoder.read();
    if (enc_delta) {
        encoder_value += enc_delta;
        SEGGER_RTT_printf(0, "encoder: %d\n", encoder_value);
        if(translation_mode == MODE_ROT_3DOF ) {
            SENSOR_R_SCALE += 0.05f * (float) (enc_delta);
            if (SENSOR_R_SCALE < 0.2) {
                SENSOR_R_SCALE = 0.2;
            } else if (SENSOR_R_SCALE > 5) {
                SENSOR_R_SCALE = 5;
            }
        }
    }

    return enc_delta;

}




