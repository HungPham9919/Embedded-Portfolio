/*
 * PWM_Control.h
 *
 *  Created on: Apr 26, 2026
 *      Author: hung
 */

#ifndef INC_PWM_CONTROL_H_
#define INC_PWM_CONTROL_H_

extern volatile float u_roll,u_pitch,u_yaw ;
extern int16_t M1,M2,M3,M4;
void Setup_PID(void);

void Control_PWM(float roll_sp, float pitch_sp, float yaw_sp);

#endif /* INC_PWM_CONTROL_H_ */
