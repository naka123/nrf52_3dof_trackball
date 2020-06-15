#include "hid_3dx.h"

bool send_3dx_report1(const int16_t cx, const int16_t cy, const int16_t cz) {
    const auto x = (int16_t)((float)(cx) * SENSOR_T_SCALE);
    const auto y = (int16_t)((float)(cz) * SENSOR_T_SCALE);
    const auto z = (int16_t)((float)(cy) * SENSOR_T_SCALE);

    hid_3dx_report_t report =
            {
                    .x       = x,
                    .y       = y,
                    .z       = z,
                    .rx       = 0,
                    .ry       = 0,
                    .rz       = 0,
            };

    return tud_hid_report(1, &report, sizeof(report));
}

bool send_3dx_report2(const int16_t cx, const int16_t cy, const int16_t cz) {
    const auto rx = (int16_t)((float)(cy) * SENSOR_R_SCALE);
    const auto ry = (int16_t)((float)(-cx) * SENSOR_R_SCALE);
    const auto rz = (int16_t)((float)(-cz) * SENSOR_R_SCALE);

    hid_3dx_report_t report =
            {
                    .x       = 0,
                    .y       = 0,
                    .z       = 0,
                    .rx     = rx,
                    .ry     = ry,
                    .rz     = rz,
            };

    return tud_hid_report(1, &report, sizeof(report));
}
