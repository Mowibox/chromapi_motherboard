#include "cordic_sqrt.h"

static CORDIC_HandleTypeDef *g_hcordic = NULL;

bool CordicSqrt_Init(CORDIC_HandleTypeDef *hcordic)
{
	g_hcordic = hcordic;

	CORDIC_ConfigTypeDef cfg = {
			.Function = CORDIC_FUNCTION_SQUAREROOT,
			.Precision = CORDIC_PRECISION_6CYCLES,
			.Scale = CORDIC_SCALE_0,
			.NbWrite = CORDIC_NBWRITE_1,
			.NbRead = CORDIC_NBREAD_1,
			.InSize = CORDIC_INSIZE_32BITS,
			.OutSize = CORDIC_OUTSIZE_32BITS
	};

	return HAL_CORDIC_Configure(g_hcordic, &cfg) == HAL_OK;
}

float CordicSqrt(float x) {
	int32_t x_fixed = (int32_t)((x / 4.0f) * 2147483648.0f);
	int32_t result_fixed;

	HAL_CORDIC_Calculate(g_hcordic, &x_fixed, &result_fixed, 1, 10);

	return ((float)result_fixed / 2147483648.0f) * 2.0f;
}
