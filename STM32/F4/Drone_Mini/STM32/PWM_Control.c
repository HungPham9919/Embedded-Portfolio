/*
 * PWM_Control.c
 *
 *  Created on: Apr 26, 2026
 *      Author: hung
 */
#include "stm32f4xx.h"
#include "stdio.h"
#include "PWM_Control.h"
#include "bmi088.h"
#include "cmsis_os.h"

typedef struct {
	float Kp,Ki,Kd;
	float sp;
	float integral;
	float derivative;
	float previous_error;
	float output_lim;
	float d_filter;
	float N;
}PID_Controller;

PID_Controller Pos_Pitch, Pos_Roll, Pos_Yaw;
PID_Controller roll_pid,pitch_pid,yaw_pid;
PID_Controller roll_pid_rate, pitch_pid_rate, yaw_pid_rate;


void Setup_PID(void){
	// Position
	Pos_Roll.Kp = 1.0f;
	Pos_Roll.Ki = 0;
	Pos_Roll.Kd = 0.005f;
	Pos_Roll.N = 50.0f;

	Pos_Roll.integral = 0;
	Pos_Roll.previous_error = 0;

	Pos_Pitch.Kp = 1.0f;
	Pos_Pitch.Ki = 0;
	Pos_Pitch.Kd = 0.005f;
	Pos_Pitch.N = 50.0f;

	Pos_Pitch.integral = 0;
	Pos_Pitch.previous_error = 0;

	Pos_Yaw.Kp = 1.0f;
	Pos_Yaw.Ki = 0;
	Pos_Yaw.Kd = 0.005f;
	Pos_Yaw.N = 50.0f;

	Pos_Yaw.integral = 0;
	Pos_Yaw.previous_error = 0;


	// dt = 1/200f
	roll_pid.Kp = 1.0f;
	roll_pid.Ki = 0;
	roll_pid.Kd = 0.005;
	roll_pid.N = 50.0f;


	pitch_pid.Kp = 1.0f;
	pitch_pid.Ki = 0;
	pitch_pid.Kd = 0.005;
	pitch_pid.N = 50.0f;

	yaw_pid.Kp = 1.0f;
	yaw_pid.Ki = 0;
	yaw_pid.Kd = 0.005;
	yaw_pid.N = 50.0f;

	roll_pid.integral = 0;
	roll_pid.previous_error = 0;

	pitch_pid.integral = 0;
	pitch_pid.previous_error = 0;

	yaw_pid.integral = 0;
	yaw_pid.previous_error = 0;

	float limit = 3000.0f;
	roll_pid.output_lim = limit;
	pitch_pid.output_lim = limit;
	yaw_pid.output_lim = limit;

	// Rate

	roll_pid_rate.Kp = 1.0f;
	roll_pid_rate.Ki = 0;
	roll_pid_rate.Kd = 0.01f;
	roll_pid_rate.N = 50.0f;

	pitch_pid_rate.Kp = 1.0f;
	pitch_pid_rate.Ki = 0;
	pitch_pid_rate.Kd = 0.01f;
	pitch_pid_rate.N = 50.0f;

	yaw_pid_rate.Kp = 1.0f;
	yaw_pid_rate.Ki = 0;
	yaw_pid_rate.Kd = 0.01f;
	yaw_pid_rate.N = 50.0f;

	roll_pid_rate.integral = 0;
	roll_pid_rate.previous_error = 0;

	pitch_pid_rate.integral = 0;
	pitch_pid_rate.previous_error = 0;

	yaw_pid_rate.integral = 0;
	yaw_pid_rate.previous_error = 0;

	roll_pid_rate.output_lim = limit;
	pitch_pid_rate.output_lim = limit;
	yaw_pid_rate.output_lim = limit;

}

float Update_PID(PID_Controller *pid, float measure_value,float dt){ // velocity angle
	float error_Rate = pid->sp - measure_value;
	// Cal P
	float P = pid->Kp * error_Rate;
	// Cal I and Anti-Windup
	pid->integral += error_Rate*dt;
	if(pid->integral > 150) pid->integral = 150; // saturation
	else if(pid->integral < -150) pid->integral = -150;

	float I = pid->Ki * pid->integral;

	// Cal D
	float derivative = measure_value;
	float D = -(pid->Kd * derivative);

	float filter_const = pid->N * dt;
	pid->d_filter += (D - pid->d_filter)*filter_const;

	float output = P + I + pid->d_filter;

	if(output > pid->output_lim) output = pid->output_lim;
	if(output < -pid->output_lim) output = - pid->output_lim;

	pid->previous_error = error_Rate;
	return output;
}

// M1 (CW) rear , M4 (CCW) rear
volatile float u_roll = 0,u_pitch = 0,u_yaw = 0;
volatile float u_roll_rate = 0, u_pitch_rate = 0, u_yaw_rate = 0;
volatile float u_roll_pos = 0,u_pitch_pos = 0,u_yaw_pos = 0;
int16_t M1 = 0,M2 = 0,M3 = 0,M4 = 0;

float base_pwm = 1680.0f; // 20%

float dt = 0.005f;
int x_pos = 0, y_pos = 0;

extern osThreadId_t Motor_Thread;

//float Pos_Cam = 0; Angle_Cam = 0;
//float Cam_hz = 0.03333f;

void Control_PWM(void){
	// Position Loop
//	u_roll_pos = Update_PID(&Pos_Roll, Pos_Cam, Cam_hz);

	// The outer circle - The angle of IMU
	u_roll_rate = Update_PID(&roll_pid, drone_angle.Roll_angle, dt);
	u_pitch_rate = Update_PID(&pitch_pid, drone_angle.Pitch_angle,dt);
	u_yaw_rate = Update_PID(&yaw_pid, drone_angle.Yaw_angle, dt);

	// The inner circle
	roll_pid_rate.sp = u_roll_rate;
	pitch_pid_rate.sp = u_pitch_rate;
	yaw_pid_rate.sp = u_yaw_rate;

	u_roll = Update_PID(&roll_pid_rate, final.gx, dt);
	u_pitch = Update_PID(&pitch_pid_rate, final.gy, dt);
	u_yaw = Update_PID(&yaw_pid_rate, final.gz, dt);

	if(fabs(u_roll) > 30 || fabs(u_pitch) > 30){
		M1 = 0,M2 = 0,M3 = 0,M4 = 0;
		osThreadFlagsSet(Motor_Thread, 2);
	}
	else {
		M1 = (int16_t)(base_pwm - u_roll + u_pitch - u_yaw); // M1 (CW) rear - right
		M2 = (int16_t)(base_pwm + u_roll + u_pitch + u_yaw); // M2 (CCW) rear - left
		M3 = (int16_t)(base_pwm + u_roll - u_pitch - u_yaw); // M3 (CW) front - left
		M4 = (int16_t)(base_pwm - u_roll - u_pitch + u_yaw); // M4 (CCW) front - right
	}

	// ARR = 3360 = 100%
	TIM2->CCR4 = (M1 > 2688) ? 2688 : (M1 < 0 ? 0 :(uint32_t)M1); // 80%
	TIM2->CCR2 = (M2 > 2688) ? 2688 : (M2 < 0 ? 0 :(uint32_t)M2);
	TIM4->CCR4 = (M3 > 2688) ? 2688 : (M3 < 0 ? 0 :(uint32_t)M3);
	TIM2->CCR1 = (M4 > 2688) ? 2688 : (M4 < 0 ? 0 :(uint32_t)M4);
}

void stable(void){
	roll_pid.sp = 0;
	pitch_pid.sp = 0;
	yaw_pid.sp = 0;

	Control_PWM();
}

void Move_Forward(void){
	// M3 & M4 decrease
	roll_pid.sp = 0.05f;
	pitch_pid.sp = 10.0f;
	yaw_pid.sp = 0;

	Control_PWM();
}

void Move_Back(void){
	//  M1 & M2 decrease
	roll_pid.sp = 0;
	pitch_pid.sp = -10.0f;
	yaw_pid.sp = 0;

	Control_PWM();
}

void Rotate_CW(void){
	// M2, M4 increase
	roll_pid.sp = 0;
	pitch_pid.sp = 0;
	yaw_pid.sp = -10.0f;

	Control_PWM();
}

void Rotate_CCW(void){
	// M1, M3 increase
	roll_pid.sp = 0;
	pitch_pid.sp = 0;
	yaw_pid.sp = 10.0f;

	Control_PWM();
}

void Move_Left(void){
	// M1,M4 decrease
	roll_pid.sp = -10.0f;
	pitch_pid.sp = 0;
	yaw_pid.sp = 0;

	Control_PWM();
}

void Move_Right(void){
	// M2,M3 decrease
	roll_pid.sp = 10.0f;
	pitch_pid.sp = 0;
	yaw_pid.sp = 0;

	Control_PWM();
}






















