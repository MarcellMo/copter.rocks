/*
 * AttitudeController.c
 *
 *  Created on: 29.12.2014
 *      Author: maan
 */
#include "AttitudeController.h"
#include "../_HAL/Delay/util.h"

float FLY = 0.0;

float dt = 0.0;
float rate_reference = 0.0;
float error = 0.0;
float error_old = 0.0;
float P_u = 0.0;
float D_u = 0.0;
float I_u = 0.0;
float filter_coef = 0.0;
uint32_t Prev = 0;

void PID(float *command, float *YPR, float *pqr, const float *P_rate, const float *I_rate, const float *D_rate, const float *P_angle, float *u, const float *N_rate)
{
	//PID-Controller

	uint32_t Now = millis();
	dt = ((Now - Prev)/1000.0f); // should be 100Hz / 0.01s
	Prev = Now;

	//P
	rate_reference = (*command - *YPR) * M_PI/180 * *P_angle;
	error = rate_reference - *pqr;
	P_u = error * *P_rate;

	//D Filtered
	D_u = ((error * *D_rate) - filter_coef) * *N_rate;
	filter_coef = filter_coef + (D_u * dt);

	//I
	I_u = I_u + (error * dt  * *I_rate);

	/* TODO: Anti wind up method */

	//PID
	*u = P_u + D_u + I_u;

}

void AngleRateController(float *r, float *y, const float *P, float *u)
{
	//P-Controller
	*u=(*r - *y)*M_PI/180 * *P;
}

void CalculateActuatorSpeed_Percent(float *u_phi, float *u_deta, float *u_psi_dot, float *u_hover, float *PWM_width, float *anglePitch, float *angleRoll)
{
	float saturation=100;

	//*u_psi_dot = 0;
	//*u_phi = 0;
	//*u_deta = 0;

	if (*u_hover > 5.0)
	{
		PWM_width[0]=-*u_phi+*u_deta-*u_psi_dot+*u_hover;
		PWM_width[1]=-*u_phi-*u_deta+*u_psi_dot+*u_hover;
		PWM_width[2]=*u_phi+*u_deta+*u_psi_dot+*u_hover;
		PWM_width[3]=*u_phi-*u_deta-*u_psi_dot+*u_hover;
	}

	else
	{
		PWM_width[0]=*u_hover;
		PWM_width[1]=*u_hover;
		PWM_width[2]=*u_hover;
		PWM_width[3]=*u_hover;
	}


	// Calculate vertical PWM rate of each rotor
	// Calculate the hover force of all rotors
	FLY=(PWM_width[0]+PWM_width[1]+PWM_width[2]+PWM_width[3])*cos(*anglePitch)*cos(*angleRoll);

	if (PWM_width[0]>saturation)
		PWM_width[0]=saturation;

	if (PWM_width[1]>saturation)
		PWM_width[1]=saturation;

	if (PWM_width[2]>saturation)
		PWM_width[2]=saturation;

	if (PWM_width[3]>saturation)
		PWM_width[3]=saturation;

	if (FLY < 4*MIN_SPEED)
	{
		if (PWM_width[0]<0)
			PWM_width[0]=0;

		if (PWM_width[1]<0)
			PWM_width[1]=0;

		if (PWM_width[2]<0)
			PWM_width[2]=0;

		if (PWM_width[3]<0)
			PWM_width[3]=0;
	}
	else
	{
		// preventing that motors turn off during a flight in case of tilting
		if (PWM_width[0]<MIN_SPEED)
			PWM_width[0]=MIN_SPEED;

		if (PWM_width[1]<MIN_SPEED)
			PWM_width[1]=MIN_SPEED;

		if (PWM_width[2]<MIN_SPEED)
			PWM_width[2]=MIN_SPEED;

		if (PWM_width[3]<MIN_SPEED)
			PWM_width[3]=MIN_SPEED;
	}

}
