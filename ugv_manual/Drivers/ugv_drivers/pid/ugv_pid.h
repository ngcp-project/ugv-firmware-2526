/**
 * @file pid.h
 * @author George Yu
 * @brief This file contains all function prototypes for the pid.c driver.
 * 
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/************************************* INCLUDE FILES ******************************************/
/*************************************** CONSTANTS ********************************************/
/************************************ TYPE DEFINITIONS ****************************************/
typedef struct {
    
    /* Measurement */
    float measurement;
    float setPoint;

    /* Controller Gains */
    float kp;
    float ki;
    float kd;
    float Gain;

    /* Deriviative low-pass filter time constant */
    float tau;

    /* Output limits */
    float outMin;
    float outMax;

    /* Integrator limits */
    float integratorMin;
    float integratorMax;

    /* Sample time (seconds) */
    float samplePeriod;

    /* Utility */
    float error;
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    /* Controller output */
    float out;

} ugv_pid;


/*************************************** FUNCTIONS ********************************************/

/**
 * @brief Function to initialize a ugv_pid instance.
 * 
 * @param pid is a pointer to a ugv_pid instance to initialize.
 */
void ugv_pid_initialize(ugv_pid *inst);


/**
 * @brief Function to update the output of a ugv_pid instance.
 * 
 * @param inst is a pointer to a ugv_pid instance to update.
 * @param setpoint is the desired setpoint of the ugv_pid.
 * @param measurement is the measured value for PID calculation.
 * @return float is the output of the PID.
 */
float ugv_pid_update(ugv_pid *inst, float setpoint, float measurement);

short int ugv_pid_updateV2(ugv_pid *inst, short int setpoint, short int measurement);

int ugv_pid_updateV3(ugv_pid *pid, int setpoint, int measurement);

#endif 
