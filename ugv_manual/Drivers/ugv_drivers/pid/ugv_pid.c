
/**
 * @file ugv_pid.h
 * @author your name
 * @brief This file contains all functions for ugv_pid.
 * 
 */


/************************************* INCLUDE FILES ******************************************/
#include "ugv_pid.h"
#include "stdint.h"


/*************************************** FUNCTIONS ********************************************/

/**
 * @brief Function to initialize a ugv_pid instance.
 * 
 * @param pid is a pointer to a ugv_pid instance to initialize.
 */
void ugv_pid_initialize(ugv_pid *inst)
{
    inst->error = 0.0f;
    inst->integrator = 0.0f;
    inst->prevError = 0.0f;
    inst->differentiator = 0.0f;
    inst->prevMeasurement = 0.0f;
    inst->out = 0.0f;
}


/**
 * @brief Function to update the output of a ugv_pid instance.
 * 
 * @param inst is a pointer to a ugv_pid instance to update.
 * @param setpoint is the desired setpoint of the ugv_pid.
 * @param measurement is the measured value for PID calculation.
 * @return float is the output of the PID.
 */
float ugv_pid_update(ugv_pid *inst, float setpoint, float measurement)
{
    float error;
    float proportional_error;

    error = setpoint - measurement;
    proportional_error = inst->kp * error;

    inst->integrator = inst->integrator + 0.5f * inst->ki * inst->samplePeriod * (error + inst->prevError);

    if (inst->integrator > inst->integratorMax) {
        inst->integrator = inst->integratorMax;
    } else if (inst->integrator < inst->integratorMin) {
        inst->integrator = inst->integratorMin;
    }

    if (inst -> kd > 0)
    {
    	inst->differentiator = -(2.0f * inst->kd * (measurement - inst->prevMeasurement)
                    + (2.0f * inst->tau - inst->samplePeriod) * inst->differentiator)
                    / (2.0f * inst->tau + inst->samplePeriod);
    }

    inst->out = proportional_error + inst->integrator + inst->differentiator;

    if (inst->out > inst->outMax) {
        inst->out = inst->outMax;
    } else if (inst->out < inst->outMin) {
        inst->out = inst->outMin;
    }

	/* Store error and measurement for later use */
    inst->prevError       = error;
    inst->prevMeasurement = measurement;

	/* Return controller output */
    return inst->out;
}

short int ugv_pid_updateV2(ugv_pid *inst, short int setpoint, short int measurement){
   short int error;
   short int proportional_error;

    error = setpoint - measurement;
    proportional_error = inst->kp * error;

    inst->integrator = inst->integrator + 0.5f * inst->ki * inst->samplePeriod * (error + inst->prevError);

    if (inst->integrator > inst->integratorMax) {
        inst->integrator = inst->integratorMax;
    } else if (inst->integrator < inst->integratorMin) {
        inst->integrator = inst->integratorMin;
    }

    if (inst -> kd > 0)
    {
    	inst->differentiator = -(2.0f * inst->kd * (measurement - inst->prevMeasurement)
                    + (2.0f * inst->tau - inst->samplePeriod) * inst->differentiator)
                    / (2.0f * inst->tau + inst->samplePeriod);
    }

    inst->out = proportional_error + inst->integrator + inst->differentiator;

    if (inst->out > inst->outMax) {
        inst->out = inst->outMax;
    } else if (inst->out < inst->outMin) {
        inst->out = inst->outMin;
    }

	/* Store error and measurement for later use */
    inst->prevError       = error;
    inst->prevMeasurement = measurement;

	/* Return controller output */
    return (short int) inst->out;
}

int ugv_pid_updateV3(ugv_pid *pid, int setpoint, int measurement) {

	/*
	* Error signal
	*/
    float error = (float) (setpoint - measurement)/(pid->Gain);

	/*
	* Proportional
	*/
    float proportional = pid->kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5 * pid->ki * pid->samplePeriod * (error + pid->prevError);


    if (pid->integrator > pid->integratorMax) {
		pid->integrator = pid->integratorMax;
	} else if (pid->integrator < pid->integratorMin) {
		pid->integrator = pid->integratorMin;
	}

	/*
	* Derivative (band-limited differentiator)
	*/
    if(pid->kd != 0){
    	pid->differentiator = -(2.0 * pid->kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
    	                        + (2.0 * pid->tau - pid->samplePeriod) * pid->differentiator)
    	                        / (2.0 * pid->tau + pid->samplePeriod);
    }

	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->outMax) {
		pid->out = pid->outMax;
	} else if (pid->out < pid->outMin) {
		pid->out = pid->outMin;
	}

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return (int) (pid->out*pid->Gain);

}
