#ifndef PID_H
#define PID_H
#include "stdio.h"
#include "stdint.h"

typedef struct {
	float Kp,Ki,Kd;
	float output_lim;
	float integral_lim;
	float N;
	// Internal State Variables
	float sp;
	float integral;
	float derivative;
	float previous_error;
	float previous_measure;
	float d_filter;

} PID_Controller;

typedef struct {
	float vx,vy,vz;
	float z_height;
} Velocity_t;

typedef struct {
	PID_Controller roll;
	PID_Controller pitch;
	PID_Controller yaw;
} Axis3D_PID;

typedef struct {
	PID_Controller x;
	PID_Controller y;
	PID_Controller z;
} Pos3D_PID;

// All

typedef struct {
	Pos3D_PID pos; // outer loop 1
	Pos3D_PID vel; // outer loop2
	Axis3D_PID angle; // inner loop1
	Axis3D_PID rate; // inner loop2
	int16_t thrust_final;
} Drone_PID_GAINS;

struct Desired_Input{
	int16_t x,y,z;
};

extern struct Desired_Input local_desired;

void Setup_PID_For_Closed_Loop(void);
void Position_Loop_PID(int8_t X_Desired, int8_t Y_Desired, int8_t Z_Desired);
void Angle_Loop_PID(void);

#endif
