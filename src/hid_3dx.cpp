#include "hid_3dx.h"
#include <string.h>

uint8_t translation_mode = MODE_ROT_3DOF;

float SENSOR_R_SCALE = 1.5f;
float SENSOR_T_SCALE = 1.5f;

void map_as_2T1Rdof(int16_t cx, int16_t cy, int16_t cz, hid_3dx_report_t *report) {
    const auto x = (int16_t)((float)(cx) * SENSOR_T_SCALE);
//    const auto y = (int16_t)((float)(cz) * SENSOR_T_SCALE);
    const auto z = (int16_t)((float)(cy) * SENSOR_T_SCALE);

    const auto rz = (int16_t)((float)(-cz) * SENSOR_R_SCALE);

    report->x       = x; // влево/вправо
    report->y       = 0; // на себя/от себя
    report->z       = z; // вверх/вниз
    report->rx       = 0;
    report->ry       = -rz;
    report->rz       = 0;
}


void map_as_3Rdof_and_zoom(int16_t cx, int16_t cy, int16_t cz, int16_t zoom, hid_3dx_report_t *report) {
    const auto rx = (int16_t)((float)(cy) * SENSOR_R_SCALE);
    const auto ry = (int16_t)((float)(-cx) * SENSOR_R_SCALE);
    const auto rz = (int16_t)((float)(-cz) * SENSOR_R_SCALE);

    report->x       = 0;
    report->y       = zoom;
    report->z       = 0;
    report->rx     = rx;
    report->ry     = ry;
    report->rz     = rz;
}

bool send_3dx_report_6dof(const hid_3dx_report_t *report) {
    return tud_hid_report(1, report, sizeof(*report));
}

bool send_3dx_report_buttons(const uint32_t buttons) {
    return tud_hid_report(3, &buttons, 4);
}


// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{

    printf("get_report_callback report _id: %02x, _type: %02x, _reqlen: %d\n", report_id, (uint8_t)report_type, reqlen);


    if ( report_type == HID_REPORT_TYPE_FEATURE ) {
        switch(report_id) {
//            case REPORT_ID_MODE:
//                translation_mode = buffer[1];
//                printf("mode changed to: %d\n", translation_mode);
//
//                ledOff(LED_GREEN);
//                ledOff(LED_BLUE);;
//
//                switch(translation_mode) {
//                    case MODE_ROT_3DOF:
//                        ledOn(LED_GREEN);
//                        break;
//
//                    case MODE_TRANS2_ROT1:
//                        ledOn(LED_BLUE);
//                        break;
//                };
//                break;

            case REPORT_ID_R_SCALE:
                struct {
                    uint8_t report_id = REPORT_ID_R_SCALE;
                    uint16_t scale = uint16_t (SENSOR_R_SCALE*100);
                } res;
                memcpy(buffer, &res, sizeof(res));
                break;
        }
    }


    return 0;
}

union f{
    uint16_t v;
    uint8_t b[2];
};

void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    // This example doesn't use multiple report and report ID
    (void) report_id;
    (void) report_type;

    printf("set_report_callback report _id: %02x, _type: %02x, _sz: %d\n", report_id, (uint8_t)report_type, bufsize);

    if ( report_type == HID_REPORT_TYPE_FEATURE ) {
        switch(report_id) {
            case REPORT_ID_MODE:
                translation_mode = buffer[1];
                printf("mode changed to: %d\n", translation_mode);

                ledOff(LED_GREEN);
                ledOff(LED_BLUE);;

                switch(translation_mode) {
                    case MODE_ROT_3DOF:
                        ledOn(LED_GREEN);
                        break;

                    case MODE_TRANS2_ROT1:
                        ledOn(LED_BLUE);
                        break;
                };
                break;

            case REPORT_ID_R_SCALE:
//                printf("REPORT_ID_R_SCALE: %d\n", (f)(buffer).v);
                break;
        }
    }

}