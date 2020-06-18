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
#include "helper_3dmath.h"


void ConvertToDegrees(float*ypr, float*xyz);
void GetEuler(float *data, Quaternion *q);

class GYRO {

public:
    void init();
    bool update_from_mpu();
    void reset_origin();
    void get_angles(float *ypr);

private:
    MPU6050 mpu;

    const float GYRO_X_TRIM = -0.45f;
    const float GYRO_Y_TRIM = 1.53f;
    const float GYRO_Z_TRIM = 0.f;

    const float dt_gyro = 1./100; // 100Hz
    const float scale_gyro = 16.384f; // +32768 int16_t +2000 Deg/sec -> 16.384

    enum {
        GYRO_ST_INITIAL = 0,
        GYRO_ST_RUNNING = 1,
    };

    Quaternion gyro_q_center = Quaternion();
    Quaternion gyro_q = Quaternion();
    Quaternion gyro_q_centered = Quaternion();

    int32_t state_gyro = GYRO_ST_INITIAL;

    uint32_t prev_mpu_millis = 0;

    void gyro_q_update(int16_t raw_x, int16_t raw_y, int16_t raw_z);


};

extern GYRO Gyro;

#ifdef __cplusplus
}
#endif

#endif //NRF52_TEST_GYRO_H
