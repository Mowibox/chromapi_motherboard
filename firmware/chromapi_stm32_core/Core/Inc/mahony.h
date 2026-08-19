#ifndef MAHONY_H
#define MAHONY_H

#include "bmi088.h"

typedef struct {
	float gyro_bias[3]; // rad/s
	float acc_bias[3];  // m/s^2
	uint8_t calibrated;
} ImuCalibration_t;

typedef struct {
	float q0, q1, q2, q3;
	float bx, by, bz;
	float Kp, Ki;
	ImuCalibration_t cal;
} MahonyFilter_t;

void Mahony_Init(MahonyFilter_t *f);
void Mahony_Calibrate(MahonyFilter_t *f, BMI088 *imu);
void Mahony_Update(MahonyFilter_t *f, float gx, float gy, float gz,
		float ax, float ay, float az, float dt);

#endif
