#include "mahony.h"
#include "cordic_sqrt.h"
#include "stm32g4xx_hal.h"

#define CALIB_SAMPLES 500

void Mahony_Init(MahonyFilter_t *f) {
	f->q0 = 1.0f; f->q1 = f->q2 = f->q3 = 0.0f;
	f->bx = f->by = f->bz = 0.0f;
	f->Kp = 2.0f;
	f->Ki = 0.05f;
	f->cal.calibrated = 0;
}

void Mahony_Calibrate(MahonyFilter_t *f, BMI088 *imu) {
	float gsum[3] = {0}, asum[3] = {0};

	for (int i = 0; i < CALIB_SAMPLES; i++) {
		BMI088_ReadAccelerometer(imu);
		BMI088_ReadGyroscope(imu);
		for (int a = 0; a < 3; a++) {
			gsum[a] += imu->gyr_rps[a];
			asum[a] += imu->acc_mps2[a];
		}
		HAL_Delay(10);
	}

	for (int a = 0; a < 3; a++) {
		f->cal.gyro_bias[a] = gsum[a] / CALIB_SAMPLES;
		f->cal.acc_bias[a]  = asum[a] / CALIB_SAMPLES;
	}
	f->cal.acc_bias[2] -= 9.81f;

	f->cal.calibrated = 1;
}

void Mahony_Update(MahonyFilter_t *f, float gx, float gy, float gz,
		float ax, float ay, float az, float dt) {

	if (f->cal.calibrated) {
		gx -= f->cal.gyro_bias[0];
		gy -= f->cal.gyro_bias[1];
		gz -= f->cal.gyro_bias[2];
		ax -= f->cal.acc_bias[0];
		ay -= f->cal.acc_bias[1];
		az -= f->cal.acc_bias[2];
	}

	float ax_g = ax / 9.81f, ay_g = ay / 9.81f, az_g = az / 9.81f;
	float norm = CordicSqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
	if (norm < 1e-3f) return;
	ax_g /= norm; ay_g /= norm; az_g /= norm;

	float vx = 2.0f * (f->q1*f->q3 - f->q0*f->q2);
	float vy = 2.0f * (f->q0*f->q1 + f->q2*f->q3);
	float vz = f->q0*f->q0 - f->q1*f->q1 - f->q2*f->q2 + f->q3*f->q3;

	float ex = ay_g*vz - az_g*vy;
	float ey = az_g*vx - ax_g*vz;
	float ez = ax_g*vy - ay_g*vx;

	if (f->Ki > 0.0f) {
		f->bx += f->Ki * ex * dt;
		f->by += f->Ki * ey * dt;
		f->bz += f->Ki * ez * dt;
		gx += f->bx; gy += f->by; gz += f->bz;
	}

	gx += f->Kp * ex;
	gy += f->Kp * ey;
	gz += f->Kp * ez;

	float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;
	f->q0 += (-q1*gx - q2*gy - q3*gz) * 0.5f * dt;
	f->q1 += ( q0*gx + q2*gz - q3*gy) * 0.5f * dt;
	f->q2 += ( q0*gy - q1*gz + q3*gx) * 0.5f * dt;
	f->q3 += ( q0*gz + q1*gy - q2*gx) * 0.5f * dt;

	norm = CordicSqrt(f->q0*f->q0 + f->q1*f->q1 + f->q2*f->q2 + f->q3*f->q3);
	f->q0 /= norm; f->q1 /= norm; f->q2 /= norm; f->q3 /= norm;
}
