/*
 * Attitudecontroller.h
 *
 *  Created on: 29.12.2014
 *      Author: maan
 */

#ifndef ATTITUDECONTROLLER_H_
#define ATTITUDECONTROLLER_H_
#define MIN_SPEED 15

#ifndef DEBUG_ON_PC
#include <arm_math.h>
#endif

void AngleController(float *r, float *y, int n, const float *a, const float *b, float *x, float *u);
void AngleControllerNew(float *D, float *YPR, float *P, float *u, float *pqr, float *P_Stab);
void AngleRateController(float *r, float *y, const float *P, float *u);
void CalculateActuatorSpeed_Percent(float *u_phi, float *u_deta, float *u_psi_dot, float *u_hover, float *PWM_width, float *anglePitch, float *angleRoll);
void PID(float *command, float *YPR, float *pqr, const float *P_rate, const float *I_rate, const float *D_rate, const float *P_angle, float *u, const float *N_rate);

#endif /* ATTITUDECONTROlLER_H_ */
