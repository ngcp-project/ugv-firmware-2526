/**
 * @file uart_utilities.h
 * @author Gerg Yu
 * @brief This file contains all functions for uart_utilities.
 *
 */


/************************************* INCLUDE FILES ******************************************/
#include "uart_utilities.h"


/*************************************** FUNCTIONS ********************************************/

/**
 * @brief Function to get an integer of user-specified number length via polled UART. This
 *        function is blocking such that it may never return if the user does not input data.
 *
 * @param huart is a pointer to the UART instance to use.
 * @param digits is the max number of digits the desired input should be.
 * @return int is the value the user entered.
 */
int ugv_uart_getIntegerPolled(UART_HandleTypeDef *huartInst, int digits)
{
    int     usrValue   = 0;
    int     digitCount = 0;
    _Bool   negative   = 0;
    uint8_t recvByte;


    while(digitCount < digits)
    {
        HAL_UART_Receive(huartInst, &recvByte, 1, HAL_MAX_DELAY);

        if (recvByte == '\r' || recvByte == '\n') break;

        if (digitCount == 0 && recvByte == '-') {
            negative = 1;
            HAL_UART_Transmit(huartInst, &recvByte, 1, HAL_MAX_DELAY);
        }
        else if (!isalpha(recvByte)) {
            HAL_UART_Transmit(huartInst, &recvByte, 1, HAL_MAX_DELAY);
            recvByte -= '0';
            usrValue = usrValue * 10 + recvByte;
            digitCount++;
        }
    }
    if(negative) usrValue = -usrValue;
    return usrValue;
}


