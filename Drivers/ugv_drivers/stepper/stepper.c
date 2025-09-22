/*
 * stepper.c
 *
 *  Created on: Mar 21, 2023
 *      Author: chris
 */

#include "stepper.h"


/*
void init_Stepper(Stepper_Controller_t *my_stepper, SPI_HandleTypeDef *SPI_instance, GPIO_TypeDef *GPIO_SCS, uint16_t pin_num)
{
	char successFlag;
	my_stepper->SCS = GPIO_SCS;
	my_stepper->gpio_pin_num = pin_num;

	my_stepper->SPI_handle = SPI_instance;

	my_stepper->currentPos = 0;
	my_stepper->setpointPos = 0;
	my_stepper->step = STEP_1;

	successFlag = SPI_Transfer(my_stepper, TORQUE_ADDRESS, 0xF0);
	//my_stepper->UART_handle = UART_instance;
}*/

void SCS_High(Stepper_Controller_t *stepper)
{
	HAL_GPIO_WritePin(stepper->SCS, stepper->gpio_pin_num, GPIO_PIN_SET);
}

void SCS_Low(Stepper_Controller_t *stepper)
{
	HAL_GPIO_WritePin(stepper->SCS, stepper->gpio_pin_num, GPIO_PIN_RESET);
}

/*
uint8_t SPI_Transfer(Stepper_Controller_t *my_stepper, uint8_t first_half, uint8_t second_half)
{
	//transferSuccess local variable should be treated as a boolean flag
	uint8_t transferSuccess = 0;

	//MOSI buffer contains a 16 bit data message for MOSI operation

	my_stepper->MOSI_buffer[0] = second_half;
	my_stepper->MOSI_buffer[1] = first_half;

	SCS_High(my_stepper);

	transferSuccess = HAL_SPI_Transmit(my_stepper->SPI_handle, my_stepper->MOSI_buffer, 2, 100);

	SCS_Low(my_stepper);

	return transferSuccess;
}*/

void getStatus(Stepper_Controller_t *stepper)
{
	uint8_t isSuccessful = 0;

	char uart_buf[50];
	int uart_buf_len;

	isSuccessful = SPI_Receive(stepper);

	if (isSuccessful != HAL_OK)
	{
	  uart_buf_len = sprintf(uart_buf, "SPI Receive failed");
	  HAL_UART_Transmit(stepper->UART_handle, (uint8_t *)uart_buf, uart_buf_len, 100);
	}
	else
	{
	 uart_buf_len = sprintf(uart_buf, "Val: %u, %u\n\r", stepper->MISO_buffer[0], stepper->MISO_buffer[1]);
	 HAL_UART_Transmit(stepper->UART_handle, (uint8_t *)uart_buf, uart_buf_len, 100);
	 memset(uart_buf, '0', 50);
	}

}

uint8_t commandAngle(Stepper_Controller_t *stepper, int angle, STEP_SIZE stepsPer)
{

	/* Boolean flag that signals if the stepper was commanded to the proper angle successfully */
	uint8_t successFlag = 0;

	/* Boolean flag that indicates if the commanded angle will rotate the stepper motor in the CCW direction */
	uint8_t isCCW = 1;

	/* Local variable that will receive the stepper behavior (Step Size) from the chooseBehavior method */
	uint8_t controlStep;


//	if (angle > 360)
//		angle = 360;
//
//	if (angle < -360)
//		angle = -360;

	/* If angle is negative, the stepper motor will rotate in the CW direction */
	if (angle < 0)
	{
		isCCW = 0;

		angle =  abs(angle);
	}

	controlStep = chooseBehavior(stepsPer, isCCW);

	float ratio = angle / CIRCLE_DEGREES;

	stepper->commandAngle =  ratio * (stepsPer) * STEP_SIZE_PER_REV;

	stepper->currentPos = 0;

	while (stepper->currentPos != (stepper->commandAngle))
	{
		successFlag = SPI_Transfer(stepper, CTRL_ADDRESS, controlStep);

		stepper->currentPos++;
	}

	/* Create an instance field in the struct that can control the delay time */
	HAL_Delay(1000);

	/* reverse the direction of the angle */
	angle *= (-1);

	/* If is CCW == 1 initially then the flag will be set to 0 and vice versa
	 * This bit of code has the purpose of restoring the stepper motor to its original position
	 */
	isCCW = (isCCW == 0) ? 1 : 0;

	controlStep = chooseBehavior(stepsPer, isCCW);


	while (stepper->currentPos != 0)
	{
		successFlag = SPI_Transfer(stepper, CTRL_ADDRESS, controlStep);

		stepper->currentPos--;
	}

	HAL_Delay(1000); //Perhaps create a field that can control the delay time

	return successFlag;
}

void stepperGoToAngle(Stepper_Controller_t *stepper, float angle,
		STEP_SIZE stepsPer) {
	if (stepper->commandAngle != angle) {
		if (stepperChangeStepSize(stepper, stepsPer)) {
			stepper->setpointPos = (int) (angle
					/ (stepper->stepSize / stepper->step));
		} else {
			stepper->setpointPos = (int) (angle
					/ (stepper->stepSize / stepper->step));
		}
		stepper->commandAngle = angle;
	}
}

void stepperMoveUpdate(Stepper_Controller_t *stepper) {
	uint8_t dir;
	uint8_t command;
	uint8_t successFlag = 0;
	if (stepper->currentPos != stepper->setpointPos) {

		if (stepper->currentPos <= stepper->setpointPos) {
			dir = 1;
			stepper->currentPos++;
		}
		if (stepper->currentPos > stepper->setpointPos) {
			dir = 0;
			stepper->currentPos--;
		}

		command = chooseBehavior(stepper->step, dir);

		successFlag = SPI_Transfer(stepper, CTRL_ADDRESS, command);
		stepperCurrentAngleCalc(stepper);
	}

}


uint8_t stepperChangeStepSize(Stepper_Controller_t *stepper,
		STEP_SIZE stepsPer) {
	uint8_t stepSizesMatch = 0;

	if (stepper->step == stepsPer) {
		stepSizesMatch = 1;
	} else {
		if (stepper->step < stepsPer) {
			stepper->currentPos *= (int) (stepsPer / stepper->step);
			stepper->setpointPos *= (int) (stepsPer / stepper->step);
			stepper->step = stepsPer;
		} else {
			stepper->currentPos *= (int) (stepper->step / stepsPer);
			stepper->setpointPos *= (int) (stepper->step / stepsPer);
			stepper->step = stepsPer;
		}

		stepSizesMatch = 0;
	}

	return stepSizesMatch;
}

void stepperCurrentAngleCalc(Stepper_Controller_t *stepper){
	stepper->currentAngle = (float) (stepper->stepSize/stepper->step)*(stepper->currentPos);
}


/*
uint8_t SPI_Receive(Stepper_Controller_t *my_stepper)
{
	// receiveSuccess local variable should be treated as a boolean flag
	uint8_t receiveSuccess = 0;

	my_stepper->MOSI_buffer[0] = 0x00U;
	my_stepper->MOSI_buffer[1] = READ_STATUS_REG;

	my_stepper->MISO_buffer[0] = 0x01U;
	my_stepper->MISO_buffer[1] = 0x01U;

	SCS_High(my_stepper);

	// Must write 4 bits to the motor controller IC in order to read data

	HAL_SPI_Transmit(my_stepper->SPI_handle, my_stepper->MOSI_buffer, 2, 100); //Check the return value of HAL_SPI_Transfer()

	//Reads 12 bits from status register

	receiveSuccess = HAL_SPI_Receive(my_stepper->SPI_handle, my_stepper->MISO_buffer, 2, 100);

	SCS_Low(my_stepper);

	//Clearing the 4 bits that were transfered to the motor driver IC in order to read data

	my_stepper->MOSI_buffer[1] = 0x00U;

	return receiveSuccess;
}*/

uint8_t chooseBehavior(STEP_SIZE check_step, uint8_t direction)
{
	uint8_t stepBehavior;

	if (direction)  //Direction is CCW
	{
		switch(check_step)
		{
			case STEP_1:
				stepBehavior = CTRL_FULL_STEP_CCW;
			break;

			case STEP_2:
				stepBehavior = CTRL_1_2_STEP_CCW;
			break;

			case STEP_4:
				stepBehavior = CTRL_1_4_STEP_CCW;
			break;

			case STEP_8:
				stepBehavior = CTRL_1_8_STEP_CCW;
			break;

			case STEP_16:
				stepBehavior = CTRL_1_16_STEP_CCW;
			break;

			case STEP_32:
				stepBehavior = CTRL_1_32_STEP_CCW;
			break;

			case STEP_64:
				stepBehavior = CTRL_1_64_STEP_CCW;
			break;

			case STEP_128:
				stepBehavior = CTRL_1_128_STEP_CCW;
			break;

			case STEP_256:
				stepBehavior = CTRL_1_256_STEP_CCW;
			break;

			default: //Make something happen for the default case

				break;
		}

	}

	else //Angle is CW
	{
		switch(check_step)
		{
			case STEP_1:
				stepBehavior = CTRL_FULL_STEP_CW;
			break;

			case STEP_2:
				stepBehavior = CTRL_1_2_STEP_CW;
			break;

			case STEP_4:
				stepBehavior = CTRL_1_4_STEP_CW;
			break;

			case STEP_8:
				stepBehavior = CTRL_1_8_STEP_CW;
			break;

			case STEP_16:
				stepBehavior = CTRL_1_16_STEP_CW;
			break;

			case STEP_32:
				stepBehavior = CTRL_1_32_STEP_CW;
			break;

			case STEP_64:
				stepBehavior = CTRL_1_64_STEP_CW;
			break;

			case STEP_128:
				stepBehavior = CTRL_1_128_STEP_CW;
			break;

			case STEP_256:
				stepBehavior = CTRL_1_256_STEP_CW;
			break;

			default: //Make something happen for the default case

				break;

		}
	}

	return stepBehavior;
}





