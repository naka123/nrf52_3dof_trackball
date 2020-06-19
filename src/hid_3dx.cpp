#include "hid_3dx.h"
#include <string.h>

uint8_t translation_mode = MODE_ROT_3DOF;

float SENSOR_R_SCALE = 1.5f;
float SENSOR_T_SCALE = 1.5f;

void send_motion(int16_t dx, int16_t dy, int16_t dz, int16_t d_enc) {

    hid_3dx_report_6dof_t report;

    if(translation_mode == MODE_TRANS2_ROT1 ) {
        map_as_2T1Rdof(dx, dy, d_enc, &report);
        send_3dx_report_6dof(&report);
    } else if(translation_mode == MODE_ROT_3DOF ) {
        map_as_3Rdof_and_zoom(dx, dy, dz, 0, &report);
        send_3dx_report_6dof(&report);
    }  else if(translation_mode == MODE_ROT_2DOF ) {
//        map_as_2Rdof(dx, dy, dz, 0, &report);
        tud_hid_mouse_report(REPORT_ID_MOUSE, 0, dx/2, dy/2, 0, 0);
    } else {
        return;
    }


}

void map_as_2T1Rdof(int16_t dx, int16_t dy, int16_t dz, hid_3dx_report_6dof_t *report) {
    const auto x = (int16_t)((float)(dx) * SENSOR_T_SCALE);
//    const auto y = (int16_t)((float)(cz) * SENSOR_T_SCALE);
    const auto z = (int16_t)((float)(dy) * SENSOR_T_SCALE);

    const auto rz = (int16_t)((float)(-dz) * SENSOR_R_SCALE);

    report->x       = x; // влево/вправо
    report->y       = 0; // на себя/от себя
    report->z       = z; // вверх/вниз
    report->rx       = 0;
    report->ry       = -rz * 10;
    report->rz       = 0;
}


void map_as_3Rdof_and_zoom(int16_t dx, int16_t dy, int16_t dz, int16_t d_enc, hid_3dx_report_6dof_t *report) {
    const auto rx = (int16_t)((float)(dy) * SENSOR_R_SCALE);
    const auto ry = (int16_t)((float)(-dx) * SENSOR_R_SCALE);
    const auto rz = (int16_t)((float)(-dz) * SENSOR_R_SCALE);

    report->x       = 0;
    report->y       = d_enc;
    report->z       = 0;
    report->rx     = rx;
    report->ry     = ry;
    report->rz     = rz;
}


void map_as_2Rdof(int16_t dx, int16_t dy, int16_t dz, int16_t d_enc, hid_3dx_report_6dof_t *report) {
    const auto rx = (int16_t)((float)(dy) * SENSOR_R_SCALE);
    const auto ry = (int16_t)((float)(-dx) * SENSOR_R_SCALE);
    const auto rz = (int16_t)((float)(-dz) * SENSOR_R_SCALE);

    report->x       = 0;
    report->y       = 0;
    report->z       = 0;
    report->rx     = rx; // pitch
    report->ry     = ry; // roll
    report->rz     = 0; // yaw
}


bool send_3dx_report_raw_gyro(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    hid_3dx_report_raw_gyro_t report = {
            .ax = ax,
            .ay = ay,
            .az = az,
            .gx = gx,
            .gy = gy,
            .gz = gz
    };
    return tud_hid_report(REPORT_ID_RAW_GYRO, &report, sizeof(report));
};

bool send_3dx_report_6dof(const hid_3dx_report_6dof_t *report) {
    return tud_hid_report(REPORT_ID_6DOF, report, sizeof(*report));
}

bool send_3dx_report_buttons(const uint32_t buttons) {
    return tud_hid_report(REPORT_ID_BUTTONS, &buttons, 4);
}

// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{

    printf("get_report_callback report _id: %02x, _type: %02x, _reqlen: %d\n", report_id, (uint8_t)report_type, reqlen);


    if ( report_type == HID_REPORT_TYPE_FEATURE ) {
        switch(report_id) {
            case REPORT_ID_MODE: {
                if (sizeof(hid_3dx_raw_feature_mode_t) != reqlen) {
                    printf("bad reqlen: %d (should be %d)\n", reqlen, sizeof(hid_3dx_raw_feature_scale_t));
                    return 0;
                };

                hid_3dx_raw_feature_mode_t report_mode = {
                        .report_id = REPORT_ID_MODE,
                        .mode = translation_mode
                };
                memcpy(buffer, &report_mode, sizeof(report_mode));
                return sizeof(hid_3dx_raw_feature_scale_t);;
            }
            case REPORT_ID_R_SCALE: {
                if (sizeof(hid_3dx_raw_feature_scale_t) != reqlen) {
                    printf("bad reqlen: %d (should be %d)\n", reqlen, sizeof(hid_3dx_raw_feature_scale_t));
                    return 0;
                };

                hid_3dx_raw_feature_scale_t report_r_scale = {
                        .report_id = REPORT_ID_R_SCALE,
                        .scale = (uint16_t) (SENSOR_R_SCALE * 100)
                };
                memcpy(buffer, &report_r_scale, sizeof(report_r_scale));
                return sizeof(report_r_scale);
            }
            case REPORT_ID_T_SCALE: {
                if (sizeof(hid_3dx_raw_feature_scale_t) != reqlen) {
                    printf("bad reqlen: %d (should be %d)\n", reqlen, sizeof(hid_3dx_raw_feature_scale_t));
                    return 0;
                };

                hid_3dx_raw_feature_scale_t report_t_scale = {
                        .report_id = REPORT_ID_T_SCALE,
                        .scale = (uint16_t) (SENSOR_T_SCALE * 100)
                };
                memcpy(buffer, &report_t_scale, sizeof(report_t_scale));
                return sizeof(report_t_scale);;
            }
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

    printf("set_report_callback report _id: %02x, _type: %02x, _sz: %d\n", report_id, (uint8_t)report_type, bufsize);

    if ( report_type == HID_REPORT_TYPE_FEATURE ) {
        switch(report_id) {
            case REPORT_ID_MODE: {
                translation_mode = buffer[1];
                printf("mode changed to: %d\n", translation_mode);

                ledOff(LED_GREEN);
                ledOff(LED_BLUE);;

                switch (translation_mode) {
                    case MODE_ROT_3DOF:
                        ledOn(LED_GREEN);
                        break;

                    case MODE_TRANS2_ROT1:
                        ledOn(LED_BLUE);
                        break;
                };
                break;
            }
            case REPORT_ID_R_SCALE: {
                if (sizeof(hid_3dx_raw_feature_scale_t) != bufsize) {
                    printf("bad bufsize: %d (should be %d)\n", bufsize, sizeof(hid_3dx_raw_feature_scale_t));
                    return;
                };
                auto *r1 = (hid_3dx_raw_feature_scale_t *) (buffer);
                SENSOR_R_SCALE = (float) r1->scale / 100.f;
                break;
            }
            case REPORT_ID_T_SCALE: {
                if (sizeof(hid_3dx_raw_feature_scale_t) != bufsize) {
                    printf("bad bufsize: %d (should be %d)\n", bufsize, sizeof(hid_3dx_raw_feature_scale_t));
                    return;
                };
                auto *r2 = (hid_3dx_raw_feature_scale_t *) (buffer);
                SENSOR_T_SCALE = (float) r2->scale / 100.f;
                break;
            }

        }
    }

}