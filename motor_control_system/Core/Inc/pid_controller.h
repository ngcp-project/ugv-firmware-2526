/*
 * pid_controller.h
 *
 *  Created on: Oct 23, 2025
 *      Author: dkim5
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include "main.h"

/************************************* INCLUDE FILES ******************************************/
/*************************************** CONSTANTS ********************************************/
/************************************ TYPE DEFINITIONS ****************************************/


/*************************************** FUNCTIONS ********************************************/

/**
 * @brief Function to initialize a ugv_pid instance.
 */
float PID_controller_1(float set_value, float process_value, float Kp, float Ki, float Kd, float limit);

#endif /* INC_PID_CONTROLLER_H_ */
