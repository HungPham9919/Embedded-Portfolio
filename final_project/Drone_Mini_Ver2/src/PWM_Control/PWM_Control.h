
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H
#include "stdio.h"
#include "stdint.h"

typedef struct {
	float Kp,Ki,Kd;
	float sp;
	float integral;
	float derivative;
	float previous_error;
	float previous_measure;
	float output_lim;
	float d_filter;
	float N;
}PID_Controller;

extern volatile float u_roll,u_pitch,u_yaw;
extern int16_t M1,M2,M3,M4;
void Setup_PID(void);
void Control_PWM(float roll_sp, float pitch_sp, float yaw_sp);

#endif