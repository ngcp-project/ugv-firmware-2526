#include "linearactuator.h"

void ugv_initPWMLA(ugvlinearActuatorPWM_t *linearActuator,
		TIM_HandleTypeDef *htimerx, uint16_t channel_x, uint16_t channel_y) {

	linearActuator->state = 0;
	//initialize PWM
	HAL_TIM_Base_Start(htimerx);
	HAL_TIM_PWM_Start(htimerx, channel_x);
	HAL_TIM_PWM_Start(htimerx, channel_y);

}

void ugv_movePWMLA(ugvlinearActuatorPWM_t *linearActuator, uint8_t state,
		uint16_t speed) {
	if (state != linearActuator->state) {
		if (speed > linearActuator->max_speed) {
			linearActuator->speed = linearActuator->max_speed;
		}
		if (speed < linearActuator->max_speed) {
			linearActuator->speed = speed;
		}
		if (state == 0) {
			*linearActuator->CCRXCH1 = linearActuator->speed;
			*linearActuator->CCRXCH2 = 0;
		}
		if (state == 2) {
			*linearActuator->CCRXCH1 = 0;
			*linearActuator->CCRXCH2 = linearActuator->speed;
		}
	}
}

void ugv_checkPWMLA(ugvlinearActuatorPWM_t *linearActuator) {
	if(!HAL_GPIO_ReadPin(linearActuator->EGPIOPort, linearActuator->EGPIOPinNum)){
		linearActuator->state = 2;
		linearActuator->speed = 0;
		*linearActuator->CCRXCH1 = 0;
		*linearActuator->CCRXCH2 = 0;
	}

	if(!HAL_GPIO_ReadPin(linearActuator->RGPIOPort, linearActuator->RGPIOPinNum)){
		linearActuator->state = 0;
		linearActuator->speed = 0;
		*linearActuator->CCRXCH1 = 0;
		*linearActuator->CCRXCH2 = 0;
	}
}

void ugv_moveGPIOLA(ugvlinearActuatorGPIO_t *linearActuator, uint8_t state) {
	if (state != linearActuator->state) {
		if (state == 0) {
			HAL_GPIO_WritePin(linearActuator->MTR_P_GPIOPort,
					linearActuator->MTR_P_GPIOPinNum, GPIO_PIN_SET);
			HAL_GPIO_WritePin(linearActuator->MTR_N_GPIOPort,
					linearActuator->MTR_N_GPIOPinNum, GPIO_PIN_RESET);
		}
		if (state == 1) {
			HAL_GPIO_WritePin(linearActuator->MTR_P_GPIOPort,
					linearActuator->MTR_P_GPIOPinNum, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(linearActuator->MTR_N_GPIOPort,
					linearActuator->MTR_N_GPIOPinNum, GPIO_PIN_SET);

		}
	}
}

void ugv_checkGPIOLA(ugvlinearActuatorGPIO_t *linearActuator) {
	if (!HAL_GPIO_ReadPin(linearActuator->RGPIOPort,
			linearActuator->RGPIOPinNum)) {
		linearActuator->state = 0;
	}
	else{
		linearActuator->state = 1;
	}
}
