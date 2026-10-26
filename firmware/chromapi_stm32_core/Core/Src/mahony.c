/* SPDX-License-Identifier: Apache-2.0 */
#include "mahony.h"
#include <math.h>
#include "stm32g4xx_hal.h"

#define CALIB_WINDOW       100     // samples per stillness check, 10 ms apart: 1 s
#define CALIB_MAX_WINDOWS  5       // after 5 s of motion, give up: Ki learns the bias instead
#define CALIB_STILL_SPREAD 0.02f   // rad/s, max - min of each gyro axis over a window (~1.1 deg/s)

void Mahony_Init(MahonyFilter_t *f) {
	f->q0 = 1.0f; f->q1 = f->q2 = f->q3 = 0.0f;
	f->bx = f->by = f->bz = 0.0f;
	f->wx = f->wy = f->wz = 0.0f;
	f->Kp = 1.0f;
	f->Ki = 0.01f;
	f->cal.calibrated = 0;
}

static void init_attitude_from_accel(MahonyFilter_t *f, const float a[3]) {
	if (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < 1.0f) return;  // no gravity reading: stay level
	float roll  = atan2f(a[1], a[2]);
	float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
	float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
	float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
	f->q0 = cr * cp;
	f->q1 = sr * cp;
	f->q2 = cr * sp;
	f->q3 = -sr * sp;
}

void Mahony_Calibrate(MahonyFilter_t *f, BMI088 *imu) {
	float acc[3] = {0};

	for (int w = 0; w < CALIB_MAX_WINDOWS && !f->cal.calibrated; w++) {
		float gsum[3] = {0}, gmin[3] = {0}, gmax[3] = {0}, asum[3] = {0};
		for (int i = 0; i < CALIB_WINDOW; i++) {
			BMI088_ReadAccelerometer(imu);
			BMI088_ReadGyroscope(imu);
			for (int a = 0; a < 3; a++) {
				float g = imu->gyr_rps[a];
				gsum[a] += g;
				if (i == 0 || g < gmin[a]) gmin[a] = g;
				if (i == 0 || g > gmax[a]) gmax[a] = g;
				asum[a] += imu->acc_mps2[a];
			}
			HAL_Delay(10);
		}

		uint8_t still = 1;
		for (int a = 0; a < 3; a++) {
			acc[a] = asum[a] / CALIB_WINDOW;
			if (gmax[a] - gmin[a] > CALIB_STILL_SPREAD) still = 0;
		}
		if (still) {
			for (int a = 0; a < 3; a++) f->cal.gyro_bias[a] = gsum[a] / CALIB_WINDOW;
			f->cal.calibrated = 1;
		}
	}

	init_attitude_from_accel(f, acc);
}

void Mahony_Update(MahonyFilter_t *f, float gx, float gy, float gz,
		float ax, float ay, float az, float dt) {

	if (f->cal.calibrated) {
		gx -= f->cal.gyro_bias[0];
		gy -= f->cal.gyro_bias[1];
		gz -= f->cal.gyro_bias[2];
	}

	f->wx = gx + f->bx;
	f->wy = gy + f->by;
	f->wz = gz + f->bz;

	float ax_g = ax / 9.81f, ay_g = ay / 9.81f, az_g = az / 9.81f;
	float norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
	if (norm < 1e-3f) return;
	ax_g /= norm; ay_g /= norm; az_g /= norm;

	float vx = 2.0f * (f->q1*f->q3 - f->q0*f->q2);
	float vy = 2.0f * (f->q0*f->q1 + f->q2*f->q3);
	float vz = f->q0*f->q0 - f->q1*f->q1 - f->q2*f->q2 + f->q3*f->q3;

	float ex = ay_g * vz - az_g * vy;
	float ey = az_g * vx - ax_g * vz;
	float ez = ax_g * vy - ay_g * vx;

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

	norm = sqrtf(f->q0*f->q0 + f->q1*f->q1 + f->q2*f->q2 + f->q3*f->q3);
	f->q0 /= norm; f->q1 /= norm; f->q2 /= norm; f->q3 /= norm;
}
