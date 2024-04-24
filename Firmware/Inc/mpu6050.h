/*
 * mpu6050.h
 *
 *  Created on: Aug 2, 2023
 *      Author: ADMIN
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#endif /* INC_MPU6050_H_ */
#include <stdint.h>
#include <main.h>

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
    double roll;
    double pitch;
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

void MPU6050_Init();
void MPU6050_Read_Accel(MPU6050_t *DataStruct);
void MPU6050_Read_Gyro(MPU6050_t *DataStruct);
void MPU6050_Read_Temp(MPU6050_t *DataStruct);
void MPU6050_Read_All(MPU6050_t *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
