#include "PWM_Control.h"
#include "BMI088_Library/bmi088.h"

PID_Controller roll_pid,pitch_pid,yaw_pid;
PID_Controller roll_pid_rate, pitch_pid_rate, yaw_pid_rate;

void Setup_PID(void){
	// Angle & rate of ROLL
	roll_pid.Kp = 2.8f;
	roll_pid.Ki = 0.02f;
	roll_pid.Kd = 0;
	roll_pid.N = 25.0f;

	roll_pid_rate.Kp = 1.2f;
	roll_pid_rate.Ki = 0;
	roll_pid_rate.Kd = 0.0025f;
	roll_pid_rate.N = 15.0f;

	// Angle & rate of PITCH

	pitch_pid.Kp = 2.8f;
	pitch_pid.Ki = 0.02f;
	pitch_pid.Kd = 0;
	pitch_pid.N = 25.0f;

	pitch_pid_rate.Kp = 1.25f;
	pitch_pid_rate.Ki = 0;
	pitch_pid_rate.Kd = 0.03f;
	pitch_pid_rate.N = 15.0f;

	// Angle & rate off YAW

	yaw_pid.Kp = 8.0f;	// yaw angle
	yaw_pid.Ki = 0.01f;
	yaw_pid.Kd = 0;
	yaw_pid.N = 15.0f;

	yaw_pid_rate.Kp = 5.5f; // yaw rate
	yaw_pid_rate.Ki = 0;
	yaw_pid_rate.Kd = 0.005f;
	yaw_pid_rate.N = 12.0f;

 // ============================== // ========================== //

	roll_pid.integral = 0;
	roll_pid.previous_error = 0;
	roll_pid.previous_measure = 0;

	pitch_pid.integral = 0;
	pitch_pid.previous_error = 0;
	pitch_pid.previous_measure = 0;

	yaw_pid.integral = 0;
	yaw_pid.previous_error = 0;
	yaw_pid.previous_measure = 0;

	float limit_angle = 450.0f; // deg/s
	float limit_rate = 220.0f; // pwm

	roll_pid.output_lim = limit_angle;
	pitch_pid.output_lim = limit_angle;
	yaw_pid.output_lim = limit_angle;

	roll_pid_rate.integral = 0;
	roll_pid_rate.previous_error = 0;
	roll_pid_rate.previous_measure = 0;

	pitch_pid_rate.integral = 0;
	pitch_pid_rate.previous_error = 0;
	pitch_pid_rate.previous_measure = 0;

	yaw_pid_rate.integral = 0;
	yaw_pid_rate.previous_error = 0;
	yaw_pid_rate.previous_measure = 0;

	roll_pid_rate.output_lim = limit_rate;
	pitch_pid_rate.output_lim = limit_rate;
	yaw_pid_rate.output_lim = limit_rate;
}

float Update_PID(PID_Controller *pid, float measure_value,float dt){
	float error = pid->sp - measure_value;
	// Cal P
	float P = pid->Kp * error;

	// Cal I and Anti-Windup
	pid->integral += error*dt;
	float i_max = pid->output_lim * 0.5f;

	if(pid->integral > i_max) pid->integral = i_max;
	else if(pid->integral < -i_max) pid->integral = -i_max;

	float I = pid->Ki * pid->integral;

	// Cal D
	float derivative = (measure_value - pid->previous_measure)/dt;
	pid->previous_measure = measure_value;

	float D = pid->Kd * derivative;

	float d_alpha = (pid->N*dt)/(1.0f + pid->N*dt); // < 1
	pid->d_filter += (D - pid->d_filter)*d_alpha;

	float output = P + I - pid->d_filter;

	if(output > pid->output_lim) output = pid->output_lim;
	if(output < -pid->output_lim) output = - pid->output_lim;

	pid->previous_error = error;
	return output;
}

// M1 (CW) rear , M4 (CCW) rear
volatile float u_roll = 0,u_pitch = 0,u_yaw = 0;
volatile float u_roll_rate = 0, u_pitch_rate = 0, u_yaw_rate = 0;
int16_t M1 = 0, M2 = 0, M3 = 0, M4 = 0;

float base_pwm = 0;
float rate_dt = 0.005f, angle_dt = 0.005f;

// State of drone
volatile int drone_state = 0;
float yaw_bias = 0,roll_bias = 0,pitch_bias = 0;
volatile float compensation_yaw = 0,compensation_pitch = 0;

void Control_PWM(float roll_sp, float pitch_sp, float yaw_sp){

	roll_pid.sp = roll_sp;
	pitch_pid.sp = pitch_sp;

	// The outer circle - The angle of IMU

	u_roll_rate = Update_PID(&roll_pid, drone_angle.Roll_angle, angle_dt); // 200Hz
	u_pitch_rate = Update_PID(&pitch_pid, drone_angle.Pitch_angle,angle_dt);

	// The inner circle

	roll_pid_rate.sp = u_roll_rate;
	pitch_pid_rate.sp = u_pitch_rate;

	yaw_pid.sp = yaw_sp;
	u_yaw_rate = Update_PID(&yaw_pid, drone_angle.Yaw_angle, angle_dt); // 200Hz
	yaw_pid_rate.sp = u_yaw_rate;


	u_roll = Update_PID(&roll_pid_rate, final.gx, rate_dt); // 200Hz
	u_pitch = Update_PID(&pitch_pid_rate, final.gy, rate_dt);
	u_yaw = Update_PID(&yaw_pid_rate, final.gz, rate_dt);

	if(fabs(drone_angle.Roll_angle) > 15 || fabs(drone_angle.Pitch_angle) > 15 || fabs(drone_angle.Yaw_angle) > 20){
		GPIOC->BSRR = (1 << 0);
	}

	if(drone_state == 1){
		if(base_pwm < 600.0f) {
		   roll_pid.integral = 0;
		   pitch_pid.integral = 0;
		   yaw_pid.integral = 0;
		   roll_pid_rate.integral = 0;
		   pitch_pid_rate.integral = 0;
		   yaw_pid_rate.integral = 0;
		 }

		u_yaw *= 0.92f;

		M1 = (int16_t)(base_pwm - u_roll + u_pitch + u_yaw); // CW front
		M2 = (int16_t)(base_pwm + u_roll + u_pitch - u_yaw); // CCW front
		M3 = (int16_t)(base_pwm + u_roll - u_pitch + u_yaw); // CW rear
		M4 = (int16_t)(base_pwm - u_roll - u_pitch - u_yaw); // CCW rear

		compensation_yaw = (base_pwm - 640.0f) * 0.5f; // Chỉ bù khi ga vượt ngưỡng hover
		    if(compensation_yaw > 0) {
		        M1 += (int16_t)compensation_yaw;
		        M4 += (int16_t)compensation_yaw;
		    }

		 if(M1 > 1600 || M2 > 1600 || M3 > 1600 || M4 > 1600) {
			drone_state = 2;
			base_pwm = 0;
			M1 = 0;M2 = 0;M3 = 0; M4 = 0;
		}
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

	TIM4->CCR4 = M1;
	TIM2->CCR1 = M2;

	TIM2->CCR4 = M3;
	TIM2->CCR2 = M4;
}

void Stop_Motor(void){
	base_pwm = 0;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR4 = 0;
	TIM4->CCR4 = 0;
}