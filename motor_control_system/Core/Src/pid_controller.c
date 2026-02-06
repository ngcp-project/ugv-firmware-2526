/*
 * pid_controller.c
 *
 *  Created on: Oct 23, 2025
 *      Author: dkim5
 */

#include "stdint.h"
#include "pid_controller.h"


/*************************************** FUNCTIONS ********************************************/

/**
 * @brief Function to initialize a ugv_pid instance.
 */

float PID_controller_1(float set_value, float process_value, float Kp, float Ki, float Kd, float limit)
{
	float error;
	static float error_old;
	float P_value;
	float I_value;
	float D_value;
	static float I_sum;
	float PID_value;
	float Time = 0.001;

	error = (set_value - process_value);
	P_value = Kp * error;
	if (Ki > 0)
	{
		if((Ki * I_sum) < limit && (Ki * I_sum) > -limit)
		{
			I_value = I_sum += Time * error;
		}
	}
	else
	{
		I_value = 0;
		I_sum = 0;
	}
	if(Kd > 0)
	{
		if((error - error_old)/ Time < limit && (error - error_old)/ Time > -limit)
		{
			D_value = (error - error_old) / Time;
		}
	}
	else
	{
		D_value = 0;
	}
	PID_value = P_value + Ki * I_value + Kd * D_value;
	if (PID_value < -limit)
	{
		PID_value = -limit;
	}
	else if (PID_value > limit)
	{
		PID_value = limit;
	}
	error_old = error;

	if(set_value == 0)
		PID_value = 0;

	return PID_value;
}

