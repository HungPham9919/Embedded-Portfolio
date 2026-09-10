#include "PWM_Control.h"
#include <math.h>
#include <sys/_stdint.h>
#include "stm32f405xx.h"
#include "zephyr/irq.h"
#include "zephyr/kernel.h"

PWM_To_Motor pwm_motor;

#define THRUST_SCOPE 0.3003f // (g / %pwm) - PWM max -> CCR 3360
#define VOLTAGE_NOMINAL 4.0f
#define PWM_DEADBAND_MIN 403
#define PWM_MAX 3360

uint16_t Thrust_To_PWM_Compensation(float desired_thrust_gam, float bat_measured){
	if(bat_measured < 3.3f) bat_measured = 3.3f;

	float pwm_percent = (desired_thrust_gam/ THRUST_SCOPE);
	float votage_scale = VOLTAGE_NOMINAL/bat_measured;
	float compensation_pwm = pwm_percent *votage_scale;

	int16_t final_pwm = (int16_t)(compensation_pwm * 33.6f);

	if(final_pwm > 0) final_pwm += PWM_DEADBAND_MIN;
	if(final_pwm > PWM_MAX) final_pwm = PWM_MAX;
	if(final_pwm < 0) final_pwm = 0;

	return (uint16_t)final_pwm;
}

void Motor(void){
	TIM4->CCR4 = pwm_motor.M1_Thrust_comp;
	TIM2->CCR1 = pwm_motor.M2_Thrust_comp;

	TIM2->CCR4 = pwm_motor.M3_Thrust_comp;
	TIM2->CCR2 = pwm_motor.M4_Thrust_comp;
}

