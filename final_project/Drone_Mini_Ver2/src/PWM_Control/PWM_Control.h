#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H
#include "stdio.h"
#include "stdint.h"
#include <sys/_stdint.h>

extern int16_t M1,M2,M3,M4;
typedef struct {
	int16_t M1, M2, M3, M4;
	uint16_t M1_Thrust_comp, M2_Thrust_comp, M3_Thrust_comp , M4_Thrust_comp;
} PWM_To_Motor;

extern PWM_To_Motor pwm_motor;
uint16_t Thrust_To_PWM_Compensation(float desired_thrust_gam, float bat_measured);
void Motor(void);
#endif
