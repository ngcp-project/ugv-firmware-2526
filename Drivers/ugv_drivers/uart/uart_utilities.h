/**
 * @file uart_utilities.h
 * @author Gerg Yu
 * @brief This file contains all function prototypes for uart_utilities.c
 * 
 */

#ifndef UART_UTILITIES_H
#define UART_UTILITIES_H

/**************************************** CONFIG **********************************************/
#define BIGBOY  1
//#define SMOLBOY 0

/************************************* INCLUDE FILES ******************************************/

#if defined(BIGBOY)
#include "stm32f7xx_hal.h"
#elif defined(SMOLBOY)
#include "stm32f7xx_hal.h"
#endif

#include "ctype.h"

/*************************************** CONSTANTS ********************************************/
/************************************ TYPE DEFINITIONS ****************************************/


/*************************************** FUNCTIONS ********************************************/

/**
 * @brief Function to get an integer of user-specified number length via polled UART. This 
 *        function is blocking such that it may never return if the user does not input data.
 * 
 * @param huart is a pointer to the UART instance to use.
 * @param digits is the max number of digits the desired input should be. 
 * @return int is the value the user entered.
 */
int ugv_uart_getIntegerPolled(UART_HandleTypeDef *huart, int digits);



#endif 
