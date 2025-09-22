#ifndef DRIVEMOTOR_UTILS
#define DRIVEMOTOR_UTILS

#include "stm32f7xx.h"
#include "stdlib.h"


struct PWM_t
{
	TIM_HandleTypeDef *timer;

	uint16_t CW_channel;
	uint16_t CCW_channel;
	uint16_t CW_duty;
	uint16_t CCW_duty;
};

struct DriveMotor_t
{
	/*
	 * Create necessary members that are pertinent to the drive motor
	 */

	struct PWM_t *drivePWM;
};

/*
 * 32 bit register address of timer channels ranging from timer[1,6]
 */
typedef enum
{
	CHANNEL_1 = 0x00000000U,
	CHANNEL_2 = 0x00000004U,
	CHANNEL_3 = 0x00000008U,
	CHANNEL_4 = 0x0000000CU,
	CHANNEL_5 = 0x00000010U,
	CHANNEL_6 = 0x00000014U

}TIM_CHANNELS;


typedef struct PWM_t ugv_PWM_t;
typedef struct DriveMotor_t ugv_DriveMotor_t;

/*
 * Initializes a PWM signal for two channels of a timer object
 * *drivePtr is a pointer to the DriveMotor_t struct which contains all of the necessary members to control the drive motor
 * *htimerx is a pointer to the timer object that is passed from the main function
 * channel_x is one of the timer channels that is generating PWM signal
 * channel_y is the second timer channel that is generating a PWM signal
 */

void ugv_init_PWM(ugv_DriveMotor_t *drivePtr, TIM_HandleTypeDef *htimerx, uint16_t channel_x, uint16_t channel_y);

/*
 * Controls the speed and direction of the motor
 * *drivePtr is a pointer to the DriveMotor_t struct which contains all of the necessary members to control the drive motor
 * dutyCycle controls the duty cycle of the PWM signal. If a negative value is passed the motor will spin the wheels counter clock wise
 * Else, the motor will spin the wheels in the clockwise direction
 */

void ugv_drive_direction_control(ugv_DriveMotor_t *drivePtr, short int dutyCycle);

#endif //DRIVEMOTOR_UTILS
