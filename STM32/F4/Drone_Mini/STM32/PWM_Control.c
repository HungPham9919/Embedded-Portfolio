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


	// Angle

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

	float limit_angle = 150.0f; // deg/s
	float limit_rate = 300.0f; // 10% pwm

	roll_pid.output_lim = limit_angle;
	pitch_pid.output_lim = limit_angle;
	yaw_pid.output_lim = limit_angle - 50.0f;

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

	roll_pid_rate.output_lim = limit_rate;
	pitch_pid_rate.output_lim = limit_rate;
	yaw_pid_rate.output_lim = limit_rate - 100.0f;

}

float Update_PID(PID_Controller *pid, float measure_value,float dt){
	float error = pid->sp - measure_value;
	// Cal P
	float P = pid->Kp * error;
	// Cal I and Anti-Windup
	pid->integral += error*dt;
	if(pid->integral > 150) pid->integral = 150; // saturation
	else if(pid->integral < -150) pid->integral = -150;

	float I = pid->Ki * pid->integral;

	// Cal D
	float derivative = measure_value;
	float D = -(pid->Kd * derivative);

	float d_alpha = (pid->N*dt)/(1.0f + pid->N*dt);
	pid->d_filter += (D - pid->d_filter)*d_alpha;

	float output = P + I + pid->d_filter;

	if(output > pid->output_lim) output = pid->output_lim;
	if(output < -pid->output_lim) output = - pid->output_lim;

	pid->previous_error = error;
	return output;
}

// M1 (CW) rear , M4 (CCW) rear
volatile float u_roll = 0,u_pitch = 0,u_yaw = 0;
volatile float u_roll_rate = 0, u_pitch_rate = 0, u_yaw_rate = 0;
volatile float u_roll_pos = 0,u_pitch_pos = 0,u_yaw_pos = 0;
int16_t M1 = 0,M2 = 0,M3 = 0,M4 = 0;

float base_pwm = 0;

float rate_dt = 0.005f, angle_dt = 0.02f;

extern osThreadId_t Motor_Thread;

// State of drone
volatile int drone_state = 0;

void Control_PWM(float roll_sp, float pitch_sp, float yaw_sp){

	static uint8_t pid_div = 0;
	pid_div++;
	if(pid_div > 3){
		roll_pid.sp = roll_sp;
		pitch_pid.sp = pitch_sp;
		yaw_pid.sp = yaw_sp;

		// The outer circle - The angle of IMU
		u_roll_rate = Update_PID(&roll_pid, drone_angle.Roll_angle, angle_dt); // 50Hz
		u_pitch_rate = Update_PID(&pitch_pid, drone_angle.Pitch_angle,angle_dt);
		u_yaw_rate = Update_PID(&yaw_pid, drone_angle.Yaw_angle, angle_dt);

		// The inner circle
		roll_pid_rate.sp = u_roll_rate;
		pitch_pid_rate.sp = u_pitch_rate;
		yaw_pid_rate.sp = u_yaw_rate;

		pid_div = 0;
	}

	u_roll = Update_PID(&roll_pid_rate, final.gx, rate_dt); // 200Hz
	u_pitch = Update_PID(&pitch_pid_rate, final.gy, rate_dt);
	u_yaw = Update_PID(&yaw_pid_rate, final.gz, rate_dt);

	if(fabs(drone_angle.Roll_angle) > 30 || fabs(drone_angle.Pitch_angle) > 30){
		osThreadFlagsSet(Motor_Thread, MOTOR_FLAG_EMERGENCY); // Stop motor
	}

	if(drone_state == 1){
		M1 = (int16_t)(base_pwm - u_roll + u_pitch - u_yaw); // M1 (CW) rear - right
		M2 = (int16_t)(base_pwm + u_roll + u_pitch + u_yaw); // M2 (CCW) rear - left
		M3 = (int16_t)(base_pwm + u_roll - u_pitch - u_yaw); // M3 (CW) front - left
		M4 = (int16_t)(base_pwm - u_roll - u_pitch + u_yaw); // M4 (CCW) front - right
	}

	else if(drone_state == 2) {
		base_pwm = 0;
		M1 = 0; M2 = 0; M3 = 0; M4 = 0;
		roll_pid.integral = 0;
		pitch_pid.integral = 0;
		yaw_pid.integral = 0;

		roll_pid_rate.integral = 0;
		pitch_pid_rate.integral = 0;
		yaw_pid_rate.integral = 0;

	}

	TIM2->CCR4 = M1;
	TIM2->CCR2 = M2;
	TIM4->CCR4 = M3;
	TIM2->CCR1 = M4;
}

void Stop_Motor(void){
	base_pwm = 0;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR4 = 0;
	TIM4->CCR4 = 0;
}

