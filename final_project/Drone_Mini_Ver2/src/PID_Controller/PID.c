#include "PID.h"
#include <math.h>
#include <sys/_stdint.h>
#include "INA226.h"
#include "Radio_communication/Radio_Communication.h"
#include "bmi088.h"
#include "pmw3901.h"
#include "PWM_Control.h"

Drone_PID_GAINS drone_pid;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float N, float output_limit){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->N = N;
	pid->output_lim = output_limit;
	pid->integral_lim = output_limit;

	// Reset 
	pid->integral = 0.0f;
	pid->previous_error = 0.0f;
	pid->previous_measure = 0.0f;
	pid->d_filter = 0.0f;
	pid->sp = 0;
}

void Setup_PID_For_Closed_Loop(void) {
    // 1. RATE PID (Inner-most loop) - Output: Torque/PWM
    PID_Init(&drone_pid.rate.roll,  1.2f,  0.0f, 0.0025f, 15.0f, 220.0f);
    PID_Init(&drone_pid.rate.pitch, 1.25f, 0.0f, 0.03f,   15.0f, 220.0f);
    PID_Init(&drone_pid.rate.yaw,   5.5f,  0.0f, 0.005f,  12.0f, 220.0f);

    // 2. ANGLE PID (Attitude loop) - Output: Target Rate (deg/s)
    PID_Init(&drone_pid.angle.roll,  2.8f, 0.02f, 0.0f, 25.0f, 450.0f);
    PID_Init(&drone_pid.angle.pitch, 2.8f, 0.02f, 0.0f, 25.0f, 450.0f);
    PID_Init(&drone_pid.angle.yaw,   8.0f, 0.01f, 0.0f, 15.0f, 450.0f);

    // 3. VELOCITY PID - Output: Target Tilt Angle / Acceleration
    PID_Init(&drone_pid.vel.x, 1.5f, 0.05f, 0.01f, 10.0f, 30.0f); // Max 30 deg tilt
    PID_Init(&drone_pid.vel.y, 1.5f, 0.05f, 0.01f, 10.0f, 30.0f);
    PID_Init(&drone_pid.vel.z, 2.0f, 0.1f,  0.05f, 10.0f, 80.0f); // Max thrust adjust

    // 4. POSITION PID - Output: Target Velocity (m/s)
    PID_Init(&drone_pid.pos.x, 1.0f, 0.0f, 0.0f, 10.0f, 2.0f); // Max 2 m/s
    PID_Init(&drone_pid.pos.y, 1.0f, 0.0f, 0.0f, 10.0f, 2.0f);
    PID_Init(&drone_pid.pos.z, 1.2f, 0.0f, 0.0f, 10.0f, 1.5f);
}

float Update_PID(PID_Controller *pid, float measure_value,float dt){
	float error = pid->sp - measure_value;
	// Cal P
	float P = pid->Kp * error;
	// Cal I and Anti-Windup
	pid->integral += error*dt;

	if(pid->integral > pid->integral_lim) pid->integral = pid->integral_lim;
	else if(pid->integral < -pid->integral_lim) pid->integral = -pid->integral_lim;

	float I = pid->Ki * pid->integral;

	// Cal D
	float derivative = (measure_value - pid->previous_measure)/dt; // rate of change
	float d_alpha = (pid->N*dt)/(1.0f + pid->N*dt); // < 1
	pid->d_filter += (derivative - pid->d_filter)*d_alpha;
	
	float D = pid->Kd * pid->d_filter;

	float output = P + I - D;

	if(output > pid->output_lim) output = pid->output_lim;
	if(output < -pid->output_lim) output = - pid->output_lim;

	pid->previous_measure = measure_value;
	pid->previous_error = error;
	return output;
}

static float Pos_dt = 0.02f; // 50Hz
static float Angle_dt = 0.001f; // 1KHz

Velocity_t current_state;
void Update_Sensor_Velocity(float dt){
	float roll_rad = ((float)packet.roll_tsf) * (M_PI/180.0f);
	float pitch_rad = ((float)packet.pitch_tsf) * (M_PI/180.0f);

	float distance_meter = (float)packet.z_pos_tsf/1000.0f;
	float z_current = distance_meter * cosf(roll_rad) * cosf(pitch_rad);

	// Find Vz (m/s)
	static float z_previuous = 0;
	float Vz_raw = (z_current - z_previuous) / dt;
	current_state.vz += 0.2f * (Vz_raw - current_state.vz); // low pass filter
	z_previuous = z_current;
	current_state.z_height = z_current;

	// Find Vx, Vy  - Optical Flow - 35 pixel - 42 FOV - resolution = 42(rad)/35
	float gyro_to_pixel_factor = (dt * (35/(42 * M_PI/180.0f)));
	static float optical_resolution_factor = (42 * (M_PI/180.0f))/35.0f;

	float compensation_X = delta_x - ((final.gy * (M_PI/180.0f)) * gyro_to_pixel_factor);
	float compensation_Y = delta_y - ((final.gx * (M_PI/180.0f)) * gyro_to_pixel_factor);

	current_state.vx = (compensation_X * z_current * optical_resolution_factor) / dt;
	current_state.vy = (compensation_Y * z_current * optical_resolution_factor) / dt;
}

#define MAX_TILT_ANGLE_DEG 15.0f
#define HOOVER_THUST_BASE 1800 // 50% PWM

struct Desired_Input local_desired;

void Position_Loop_PID(int8_t X_Desired, int8_t Y_Desired, int8_t Z_Desired) { 
	// Setpoint
	drone_pid.pos.x.sp = X_Desired;
	drone_pid.pos.y.sp = Y_Desired;
	drone_pid.pos.z.sp = Z_Desired;

	// Find velocity
	Update_Sensor_Velocity(Pos_dt);

	drone_pid.vel.x.sp = Update_PID(&drone_pid.pos.x, packet.x_pos_tsf, Pos_dt); // 50Hz
	drone_pid.vel.y.sp = Update_PID(&drone_pid.pos.y, packet.y_pos_tsf, Pos_dt);
	drone_pid.vel.z.sp = Update_PID(&drone_pid.pos.z, packet.z_pos_tsf, Pos_dt);

	// Output

	float roll_sp_raw = Update_PID(&drone_pid.vel.y, current_state.vy, Pos_dt);
	float pitch_sp_raw = -Update_PID(&drone_pid.vel.x, current_state.vx, Pos_dt);

	drone_pid.angle.roll.sp = fmaxf(-MAX_TILT_ANGLE_DEG, fminf(MAX_TILT_ANGLE_DEG,roll_sp_raw));
	drone_pid.angle.pitch.sp = fmaxf(-MAX_TILT_ANGLE_DEG, fminf(MAX_TILT_ANGLE_DEG,pitch_sp_raw));

	// Vz = thrust compensation
	float thrust_correction = Update_PID(&drone_pid.vel.z, current_state.vz, Pos_dt);
	drone_pid.thrust_final = HOOVER_THUST_BASE + thrust_correction;
}

void Angle_Loop_PID(void){

	// Angle PID - output angular rate
	drone_pid.rate.roll.sp = Update_PID(&drone_pid.angle.roll,packet.roll_tsf, Angle_dt);
	drone_pid.rate.pitch.sp = Update_PID(&drone_pid.angle.pitch,packet.pitch_tsf, Angle_dt);
	drone_pid.rate.yaw.sp = Update_PID(&drone_pid.angle.yaw, packet.yaw_tsf, Angle_dt);

	// Angular Rate - output : Torque

	float roll_torque = Update_PID(&drone_pid.rate.roll, final.gx, Angle_dt);
	float pitch_torque = Update_PID(&drone_pid.rate.pitch, final.gy, Angle_dt);
	float yaw_torque = Update_PID(&drone_pid.rate.yaw, final.gz, Angle_dt);

	// Desire Torque
	float u_thrust = drone_pid.thrust_final;

	// Desire PWM M1 (CW) - M2 (CCW) front , M3 (CW) - M4 (CCW) rear
	pwm_motor.M1 = (int16_t)(u_thrust - roll_torque + pitch_torque + yaw_torque); // CW front
	pwm_motor.M2 = (int16_t)(u_thrust + roll_torque + pitch_torque - yaw_torque); // CCW front
	pwm_motor.M3 = (int16_t)(u_thrust - roll_torque - pitch_torque - yaw_torque); // CW rear
	pwm_motor.M4 = (int16_t)(u_thrust + roll_torque - pitch_torque + yaw_torque); // CCW rear

	pwm_motor.M1_Thrust_comp = Thrust_To_PWM_Compensation(pwm_motor.M1, Current_voltage);
	pwm_motor.M2_Thrust_comp = Thrust_To_PWM_Compensation(pwm_motor.M2, Current_voltage);
	pwm_motor.M3_Thrust_comp = Thrust_To_PWM_Compensation(pwm_motor.M3, Current_voltage);
	pwm_motor.M4_Thrust_comp = Thrust_To_PWM_Compensation(pwm_motor.M4, Current_voltage);

    Motor(); 
}
