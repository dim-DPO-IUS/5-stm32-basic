#include "pid_controller.h"

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ПИД (определяем здесь)
// ============================================================================
int32_t setpoint = 0;
int32_t feedback = 0;
int32_t error = 0;
int32_t output = 0;

// Внутренние переменные (только для pid_controller.c)
static int32_t integral = 0;
static int32_t derivative = 0;
static int32_t last_error = 0;

// ============================================================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ (точно как было)
// ============================================================================
int32_t pid_update_mv(int32_t sp_mv, int32_t fb_mv) {
	setpoint = sp_mv;
	feedback = fb_mv;
	error = sp_mv - fb_mv;

	integral += error;
	if (integral > 5000 * PID_SCALE)
		integral = 5000 * PID_SCALE;
	if (integral < -5000 * PID_SCALE)
		integral = -5000 * PID_SCALE;

	derivative = error - last_error;
	last_error = error;

	output = (KP * error + KI * integral + KD * derivative) / PID_SCALE;

	if (output > 1000)
		output = 1000;
	if (output < 0)
		output = 0;

	return output;
}
