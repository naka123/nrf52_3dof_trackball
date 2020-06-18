#include <stdio.h>
#include <Arduino.h>
#include <SEGGER_SYSVIEW.h>
#include <Adafruit_USBD_CDC.h>
#include <Adafruit_USBD_HID.h>
#include <SEGGER_RTT.h>

#include "paw3204.h"
#include "hid_3dx.h"

#include "RotaryEncoder.h"

#include <Wire.h>
#include "Adafruit_MPU6050.h"

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;


// HID report descriptor using TinyUSB's template
// Generic In Out with 64 bytes report (max)
uint8_t const desc_hid_report[] =
        {
                TUD_HID_REPORT_DESC_MULTIAXIS_CONTROLLER( )
        };

//Encoder myEnc(PIN_D19, PIN_D20);

// USB HID object
Adafruit_USBD_HID usb_hid;

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
    Wire.setClock(400000);

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

    pinMode(PIN_ENC_BUTTON, INPUT);
    prev_btn = digitalRead(PIN_ENC_BUTTON);

    ledOn(LED_GREEN);

    if (!mpu.begin()) {
        SEGGER_RTT_printf(0, "Failed to find MPU6050 chip\n");
    };

    mpu.enableCycle(true);
    mpu.setCycleRate(MPU6050_CYCLE_40_HZ);

    mpu_temp = mpu.getTemperatureSensor();
//    mpu_temp->printSensorDetails();

    mpu_accel = mpu.getAccelerometerSensor();
//    mpu_accel->printSensorDetails();

    mpu_gyro = mpu.getGyroSensor();
//    mpu_gyro->printSensorDetails();

    SEGGER_RTT_printf(0, "setup end\n");

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


uint32_t prev_mpu_millis = 0;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

char tmp[128];

void loop()
{
//    SEGGER_RTT_printf(0, "loop tick\n");

//    ledOn(LED_BLUE);
    while (! usb_hid.ready() ) {
        delay(2);
    }
//    ledOff(LED_BLUE);

    if (millis() - prev_mpu_millis > 40) {
//        mpu_temp->getEvent(&temp);
//        mpu_accel->getEvent(&accel);
//        mpu_gyro->getEvent(&gyro);
        if (mpu.getEvent(&accel, &gyro, &temp)) {
            sprintf(tmp, "Temperature: %.3f deg C\n\0", temp.temperature);
            SEGGER_RTT_printf(0, tmp);
            sprintf(tmp, "Accel: X:%.3f Y:%.3f Z:%.3f m/s^\n\0", accel.acceleration.x, accel.acceleration.y,
                    accel.acceleration.z);
            SEGGER_RTT_printf(0, tmp);
            sprintf(tmp, "Gyro: X:%.3f Y:%.3f Z:%.3f radians/s^\n\0", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
            SEGGER_RTT_printf(0, tmp);

            prev_mpu_millis = millis();
        } else {
            SEGGER_RTT_printf(0, "MPU getEvent failed!\n");
        }


    }


    uint8_t cur_btn = digitalRead(PIN_ENC_BUTTON);
    if (cur_btn != prev_btn) {
        prev_btn = cur_btn;
        SEGGER_RTT_printf(0, "enc button %s\n", cur_btn ? "pressed" : "released");
//        send_3dx_report_buttons(cur_btn ? 0 : V3DK_1 );
//        return;

        if (cur_btn) {
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

//    upd_btn();

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

    read_paw3204(DEV1, &stat1, &qua1, &m1x, &m1y);
    yield();
    read_paw3204(DEV2, &stat2, &qua2, &m2x, &m2y);
    yield();

    // второй сенсор повёрнут относительно первого на 180 градусов
    m2x = -m2x;
    m2y = -m2y;

    const int16_t cx = m1x + m2x;
    const int16_t cy = (m1y + m2y) / 2;
    const int16_t cz = (m1x - m2x);

    send_motion(cx, cy, cz, enc_delta);

//    while (! usb_hid.ready() ) {
//        delay(2);
//    }

}




