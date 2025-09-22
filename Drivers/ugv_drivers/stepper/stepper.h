/*
 * stepper.h
 *
 *  Created on: Mar 21, 2023
 *      Author: chris
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"

/* Macros related to the CTRL register */

#define CTRL_ADDRESS       (0x00U) //CTRL_ADDRESS byte must be used when written data to the IC's CTRL register
#define TORQUE_ADDRESS		(0x01U) //Torque Add address

#define CTRL_FULL_STEP_CW  (0x05U)
#define CTRL_1_2_STEP_CW   (0x0DU)
#define CTRL_1_4_STEP_CW   (0x15U)
#define CTRL_1_8_STEP_CW   (0x1DU)
#define CTRL_1_16_STEP_CW  (0x25U)
#define CTRL_1_32_STEP_CW  (0x2DU)
#define CTRL_1_64_STEP_CW  (0x35U)
#define CTRL_1_128_STEP_CW (0x3DU)
#define CTRL_1_256_STEP_CW (0x45U)


#define CTRL_FULL_STEP_CCW   (0x07U)
#define CTRL_1_2_STEP_CCW    (0x0FU)
#define CTRL_1_4_STEP_CCW    (0x17U)
#define CTRL_1_8_STEP_CCW    (0x1FU)
#define CTRL_1_16_STEP_CCW   (0x27U)
#define CTRL_1_32_STEP_CCW   (0x2FU)
#define CTRL_1_64_STEP_CCW   (0x37U)
#define CTRL_1_128_STEP_CCW  (0x3FU)
#define CTRL_1_256_STEP_CCW  (0x47U)

/* Macros related to the STATUS register */
#define READ_STATUS_REG  (0xF0U)

/* Macros related to step size */
#define STEP_SIZE_PER_REV (200)
#define CIRCLE_DEGREES    (360.0)

/* Macros related to torque */
#define TORQUE_ADDRESS_1 (0x10U)
#define TORQUE_ADDRESS_4 (0x14U)
#define TORQUE_ADDRESS_8 (0x1FU)



/* Enum encapsulates all of the different motor's step sizes
 * WARNING: ONLY STEP_256 and STEP_128 work consistently
 */

typedef enum
{
	STEP_1 =   1,
	STEP_2 =   2,
	STEP_4 =   4,
	STEP_8 =   8,
	STEP_16 =  16,
	STEP_32 =  32,
	STEP_64 =  64,
	STEP_128 = 128,
	STEP_256 = 256

}STEP_SIZE;


struct Stepper_Controller_t
{
	/* SCS pin is set to the a GPIO output pin's port
	 * Ex:  SCS = GPIOB  (for GPIOB Pin 9)
	 */
	GPIO_TypeDef* SCS;

	/* SCS pin is set to the a GPIO output pin number
	 * Ex:  gpio_pin_num = GPIO_PIN_9 (for GPIOB Pin 9)
	 */
	uint16_t gpio_pin_num;

	//SPI_HandleTypeDef *SPI_handle;

	UART_HandleTypeDef *UART_handle;

	/* MOSI buffer contains a 16 bit data message for MOSI operation */
	uint8_t MOSI_buffer[2];

	/* MISO buffer will receive a 12 bit data message from MISO operation */
	uint8_t MISO_buffer[2];

	/* commanded angle of the stepper */
	float commandAngle;

	/* current delta Angle of the stepper */
	float currentAngle;

	/* Current position of the stepper in # of steps */
	int currentPos;

	/* Setpoint of the current position */
	int setpointPos;

	/* Instance variable that keeps track of the motor's step size */
	STEP_SIZE step;

	/* This instance variable will be deleted. Do not worry about this one */
	uint8_t stepBeh;

	float gearRatio;

	// Expressed in Deg per step of Motor
	float stepSize;

};

typedef struct Stepper_Controller_t Stepper_Controller_t;

/* Initializes the SCS GPIO pin for the stepper motor controller IC
 * The GPIO's port and pin number must each be passed as arguments Ex: (&myStepper, &hspi1, GPIOB, GPIO_PIN_9)
 * Sets the *SPI_Handle instance variable to a configured SPI handle
 */
//void init_Stepper(Stepper_Controller_t *my_stepper, SPI_HandleTypeDef *SPI_instance, GPIO_TypeDef* GPIO_SCS, uint16_t pin_num);

/* Transmits a 16 bit data message to the stepper motor IC via SPI that will control the stepper motor behavior */
uint8_t SPI_Transfer(Stepper_Controller_t *my_stepper, uint8_t first_half, uint8_t second_half);


/* Receives data from the stepper IC's registers that have read permission
 * The received data is stored in the MISO_buffer instance variable of the Stepper_Controller_t struct
 */
uint8_t SPI_Receive(Stepper_Controller_t *my_stepper);

/* Commands the stepper motor to commanded angle using a commanded step size */
uint8_t commandAngle(Stepper_Controller_t *stepper, int angle, STEP_SIZE stepsPer);


/* Replacement for command Angle*/
void stepperGoToAngle(Stepper_Controller_t *stepper, float angle, STEP_SIZE stepsPer);

/* Update in interrupt */
void stepperMoveUpdate(Stepper_Controller_t *stepper);

/* Change Step Size selected on the Motor if applicable */
uint8_t stepperChangeStepSize(Stepper_Controller_t *stepper, STEP_SIZE stepsPer);


/* Calculate the Current Angle of Stepper */
void stepperCurrentAngleCalc(Stepper_Controller_t *stepper);

/*
 * Pre: The desired step size and direction of rotation are passed as arguments
 * Post: Returns a boolean value indicating if the data message was properly transmitted via SPI
 */
uint8_t chooseBehavior(STEP_SIZE check_step, uint8_t direction);

/* Toggle SCS pin LOW */
void SCS_Low(Stepper_Controller_t *stepper);

/* Toggle SCS pin HIGH */
void SCS_High(Stepper_Controller_t *stepper);

/* Reads the STATUS	register of the stepper motor driver a dedicated MISO pin must be set up */
void getStatus(Stepper_Controller_t *stepper);


#endif /* INC_STEPPER_H_ */
