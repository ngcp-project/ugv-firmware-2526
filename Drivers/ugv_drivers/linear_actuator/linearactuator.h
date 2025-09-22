#ifndef INC_LINEARACTUATOR_H_
#define INC_LINEARACTUATOR_H_

#include "../drive_motor/ugv_drive_motor_utilities.h"
#include "stm32f7xx.h"

typedef struct {
	volatile long unsigned int* CCRXCH1; //Period register for pwm ch 1
	volatile long unsigned int* CCRXCH2; //Period register for pwm ch 2

	uint8_t state; //Tri-State 0: Retracted, In-Movement, Extended

	uint16_t max_speed; //Max speed to travel [DC]
	uint16_t speed; //How fast to move the motor

	//Limit Switches Section
	//Retracted
	GPIO_TypeDef* RGPIOPort; //Retracted limit switch GPIO port
	uint16_t RGPIOPinNum; //Retracted limit switch GPIO pin

	//Extended
	GPIO_TypeDef* EGPIOPort; //Extended limit switch GPIO port
	uint16_t EGPIOPinNum; //Extended limit switch GPIO pin




}ugvlinearActuatorPWM_t;

typedef struct {
	uint8_t state; //Booleon 0: Retracted, 1: Extended/Extending

	//Limit Switches Section
	//Retracted
	GPIO_TypeDef *RGPIOPort; //Retracted limit switch GPIO port
	uint16_t RGPIOPinNum; //Retracted limit switch GPIO pin

	//Motor Positive Lead
	GPIO_TypeDef *MTR_P_GPIOPort;
	uint16_t MTR_P_GPIOPinNum;

	//Motor Negative Lead
	GPIO_TypeDef *MTR_N_GPIOPort;
	uint16_t MTR_N_GPIOPinNum;

}ugvlinearActuatorGPIO_t;


//PWM LA Init Function
void ugv_initPWMLA(ugvlinearActuatorPWM_t* linearActuator,TIM_HandleTypeDef *htimerx, uint16_t channel_x, uint16_t channel_y);

void ugv_movePWMLA(ugvlinearActuatorPWM_t* linearActuator, uint8_t state, uint16_t speed);

void ugv_checkPWMLA(ugvlinearActuatorPWM_t* linearActuator);

//GPIO Functions
void ugv_moveGPIOLA(ugvlinearActuatorGPIO_t* linearActuator, uint8_t state);

void ugv_checkGPIOLA(ugvlinearActuatorGPIO_t* linearActuator);

#endif /* INC_LINEARACTUATOR_H_ */
