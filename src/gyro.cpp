//
// Created by user on 19-Jun-20.
//

#include <Arduino.h>
#include "gyro.h"
#include <SEGGER_RTT.h>

const float radians_to_degrees = 180.0 / M_PI;

void GetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
}

void ConvertToDegrees(float*ypr, float*xyz) {
    //const float radians_to_degrees = 180.0 / M_PI;
    for (int i = 0; i < 3; i++) {
        xyz[i] = ypr[i] * radians_to_degrees;

    }
    if ( xyz[0] < -180 ) xyz[0] += 360;
}

GYRO Gyro;

void GYRO::init() {
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
//    mpu.reset();
//    delay(60);
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // 16g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // 2000 deg/s
    mpu.setSleepEnabled(false);

//    delay(10);
//    mpu.CalibrateGyro();
//    SEGGER_RTT_printf(0, "MPU6050 gyro calibration done\n");

    mpu.setRate(9); // 1kHz / (1+9) = 100Hz
    mpu.setAccelFIFOEnabled(true);
    mpu.setXGyroFIFOEnabled(true);
    mpu.setYGyroFIFOEnabled(true);
    mpu.setZGyroFIFOEnabled(true);
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();
    mpu.setFIFOEnabled(true);
}

char tmp[128];

void GYRO::gyro_q_update(int16_t raw_x, int16_t raw_y, int16_t raw_z) {

    const float gx = ((float)raw_x + GYRO_X_TRIM) / scale_gyro;
    const float gy = ((float)raw_y + GYRO_Y_TRIM) / scale_gyro;
    const float gz = ((float)raw_z + GYRO_Z_TRIM) / scale_gyro;

    const float Wx = gx / 180 * PI * dt_gyro;
    const float Wy = gy / 180 * PI * dt_gyro;
    const float Wz = gz / 180 * PI * dt_gyro;

    const float Qw = gyro_q.w, Qx = gyro_q.x, Qy = gyro_q.y, Qz = gyro_q.z;

    const float Qw1 = -Qx * Wx - Qy * Wy - Qz * Wz;
    const float Qx1 = +Qw * Wx - Qz * Wy + Qy * Wz;
    const float Qy1 = +Qz * Wx + Qw * Wy - Qx * Wz;
    const float Qz1 = -Qy * Wx + Qx * Wy + Qw * Wz;

    Quaternion Q1 = Quaternion(Qw1/2, Qx1/2, Qy1/2, Qz1/2);

    gyro_q.w += Q1.w;
    gyro_q.x += Q1.x;
    gyro_q.y += Q1.y;
    gyro_q.z += Q1.z;

    gyro_q.normalize();

    gyro_q_centered = gyro_q.getProduct(gyro_q_center);
}

bool GYRO::update_from_mpu() {

    if (millis() - prev_mpu_millis < 5) return false;
    prev_mpu_millis = millis();

    // читаю довольно часто, и фифо может оказаться пустым
    uint8_t buffer[12];
    if (! mpu.GetCurrentFIFOPacket(buffer, sizeof(buffer))) return false;


//    int16_t accel_x = (((int16_t) buffer[0]) << 8) | buffer[1];
//    int16_t accel_y = (((int16_t) buffer[2]) << 8) | buffer[3];
//    int16_t accel_z = (((int16_t) buffer[4]) << 8) | buffer[5];
    int16_t gyro_x = (((int16_t) buffer[6]) << 8) | buffer[7];
    int16_t gyro_y = (((int16_t) buffer[8]) << 8) | buffer[9];
    int16_t gyro_z = (((int16_t) buffer[10]) << 8) | buffer[11];

//            send_3dx_report_raw_gyro(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
//            send_motion(gyro_x/10, gyro_y/10, gyro_z/10, 0);
//            delay(1);
//            return;


    if (state_gyro == GYRO_ST_INITIAL) {
        state_gyro = GYRO_ST_RUNNING;
    }


//        sprintf(tmp, "Accel: X:%6d Y:%6d Z:%6d \tGyro: X:%6d Y:%6d Z:%6d\t dt=%u\n\0",
//                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt);
//
//        SEGGER_RTT_printf(0, tmp);

    gyro_q_update(gyro_x, gyro_y, gyro_z);

    return true;
}


void GYRO::reset_origin() {
    gyro_q_center = gyro_q.getConjugate();
}

void GYRO::get_angles(float *xyz) {
    float ypr[3];

    GetEuler(ypr, &gyro_q_centered);
    ConvertToDegrees(ypr, xyz);

        sprintf(tmp, "Anlges2: %-7.2f / %-7.2f / %-7.2f\n\0",
                xyz[0], xyz[1], xyz[2]);

        SEGGER_RTT_printf(0, tmp);

}