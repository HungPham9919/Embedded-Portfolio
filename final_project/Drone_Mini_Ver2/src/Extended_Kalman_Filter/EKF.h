#ifndef EKF_H
#define EKF_H

#include "PID_Controller/PID.h"
#include <math.h>
#include <sys/_stdint.h>
#include "INA226.h"
#include "Radio_communication/Radio_Communication.h"
#include "bmi088.h"
#include "pmw3901.h"
#include "stdio.h"
#include "stdint.h"


typedef struct {
    float z_predict,x_predict, y_predict;
    float vz_predict,vx_predict,vy_predict;

    // Covarience matrix (2x2)
    float P[2][2];
    float Q_accel; // noise from IMU
    float R_tof; // Noise from TOF
    float R_bmp; // noise from baro

} Extended_Kalman_t;

void kalman_init(void);
void kalman_predict_vl53(float acce_z_earth , float dt);
void kalman_update_vl53(float z_measure_tof, float roll, float pitch, float baro);
void kalman_update_baro(float baro);

#endif 
