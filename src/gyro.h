//
// Created by user on 19-Jun-20.
//

#pragma once

#ifndef NRF52_TEST_GYRO_H
#define NRF52_TEST_GYRO_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "helper_3dmath_my.h"

// #define USE_GYRO


void ConvertToDegrees(float*ypr, float*xyz);
void GetEuler(float *data, Quaternion *q);

class GYRO {

public:

    float drift_angle = 0.02;
    VectorFloat drift_axis = VectorFloat(-0.55, -0.15, -0.84);

    void init();
    bool update_from_mpu();
    void reset_origin();
    void get_angles(float *ypr);
//    bool get_delta_angles(float *ypr);
    bool get_delta_quat(Quaternion *q);
    void enableSendRawGyro(bool enable) { send_raw_gyro = enable; }

private:
    MPU6050 mpu;

    Quaternion drift_q;

    bool send_raw_gyro = false;

    // [-1.98778344e-04, -6.32767075e-05, -3.39180196e-04]

    const float GYRO_X_TRIM = -1.98778344e-04;
    const float GYRO_Y_TRIM = -6.32767075e-05;
    const float GYRO_Z_TRIM = -3.39180196e-04;

    const float dt_gyro = 1./100; // 100Hz
    const int SAMPLE_RATE = 9; // 1000 / (1 + n)
    const float scale_gyro = 16.384f; // +32768 int16_t +2000 Deg/sec -> 16.384
    const float raw_to_rad_sec = 1.f / scale_gyro / 180 * PI * dt_gyro;

    enum {
        GYRO_ST_INITIAL = 0,
        GYRO_ST_RUNNING = 1,
    };

//    VectorFloat orientation_axis = VectorFloat(0, 0, 1);
//    Quaternion gyro_q_orientation = Quaternion::fromAngleAxis(PI/2, orientation_axis).getConjugate();
    Quaternion gyro_q = Quaternion();
//    Quaternion gyro_q_centered = Quaternion();
    Quaternion gyro_q_delta_accumulated = Quaternion();

    int32_t state_gyro = GYRO_ST_INITIAL;

    uint32_t prev_mpu_millis = 0;

    void gyro_q_update(int16_t raw_x, int16_t raw_y, int16_t raw_z);


};

extern GYRO Gyro;

#ifdef __cplusplus
}
#endif

#endif //NRF52_TEST_GYRO_H
