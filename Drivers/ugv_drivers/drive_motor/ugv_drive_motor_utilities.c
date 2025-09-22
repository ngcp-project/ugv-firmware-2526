/*
 * drive_motor_utilities.c
 *
 *  Created on: Oct 12, 2022
 *      Author: Chris
 */

#include "ugv_drive_motor_utilities.h"
#include <math.h>

void ugv_init_PWM(ugv_DriveMotor_t *drivePtr, TIM_HandleTypeDef *htimerx, uint16_t channel_x, uint16_t channel_y)
{
	drivePtr -> drivePWM -> timer = htimerx;
	drivePtr -> drivePWM -> CW_channel = channel_x;
	drivePtr -> drivePWM -> CCW_channel = channel_y;

	HAL_TIM_Base_Start(drivePtr -> drivePWM -> timer);
	HAL_TIM_PWM_Start(drivePtr -> drivePWM -> timer, drivePtr -> drivePWM -> CW_channel);
	HAL_TIM_PWM_Start(drivePtr -> drivePWM -> timer, drivePtr -> drivePWM -> CCW_channel);
}


void ugv_drive_direction_control(ugv_DriveMotor_t *drivePtr, short int dutyCycle)
{

	if (dutyCycle >= 0) // The motor will go CCW or CW (Depends on the wiring of the motor)
	{
		dutyCycle = abs(dutyCycle);

		switch(drivePtr -> drivePWM -> CW_channel)
		{
			case CHANNEL_1:
				drivePtr -> drivePWM -> timer -> Instance -> CCR1 = dutyCycle;
				break;
			case CHANNEL_2:
				drivePtr -> drivePWM -> timer -> Instance -> CCR2 = dutyCycle;
				break;
			case CHANNEL_3:
				drivePtr -> drivePWM -> timer -> Instance -> CCR3 = dutyCycle;
				break;
			case CHANNEL_4:
				drivePtr -> drivePWM -> timer -> Instance -> CCR4 = dutyCycle;
				break;
			case CHANNEL_5:
				drivePtr -> drivePWM -> timer -> Instance -> CCR5 = dutyCycle;
				break;
			case CHANNEL_6:
				drivePtr -> drivePWM -> timer -> Instance -> CCR6 = dutyCycle;
				break;
			default:
				break;
		}

		switch(drivePtr -> drivePWM -> CCW_channel)
		{
			case CHANNEL_1:
				drivePtr -> drivePWM -> timer -> Instance -> CCR1 = 0;
				break;
			case CHANNEL_2:
				drivePtr -> drivePWM -> timer -> Instance -> CCR2 = 0;
				break;
			case CHANNEL_3:
				drivePtr -> drivePWM -> timer -> Instance -> CCR3 = 0;
				break;
			case CHANNEL_4:
				drivePtr -> drivePWM -> timer -> Instance -> CCR4 = 0;
				break;
			case CHANNEL_5:
				drivePtr -> drivePWM -> timer -> Instance -> CCR5 = 0;
				break;
			case CHANNEL_6:
				drivePtr -> drivePWM -> timer -> Instance -> CCR6 = 0;
				break;
			default:
				break;
		  }
		}

		else //if(dutyCycle < 0) // The motor will go in the opposite direction of the first case
		{
			dutyCycle = abs(dutyCycle);

			switch(drivePtr -> drivePWM -> CCW_channel)
			{
				case CHANNEL_1:
					drivePtr -> drivePWM -> timer -> Instance -> CCR1 = dutyCycle;

					break;
				case CHANNEL_2:
					drivePtr -> drivePWM -> timer -> Instance -> CCR2 = dutyCycle;
					break;
				case CHANNEL_3:
					drivePtr -> drivePWM -> timer -> Instance -> CCR3 = dutyCycle;
					break;
				case CHANNEL_4:
					drivePtr -> drivePWM -> timer -> Instance -> CCR4 = dutyCycle;
					break;
				case CHANNEL_5:
					drivePtr -> drivePWM -> timer -> Instance -> CCR5 = dutyCycle;
					break;
				case CHANNEL_6:
					drivePtr -> drivePWM -> timer -> Instance -> CCR6 = dutyCycle;
					break;
				default:
					break;


				switch(drivePtr -> drivePWM -> CW_channel)
				{
					case CHANNEL_1:
						drivePtr -> drivePWM -> timer -> Instance -> CCR1 = 0;
						break;
					case CHANNEL_2:
						drivePtr -> drivePWM -> timer -> Instance -> CCR2 = 0;
						break;
					case CHANNEL_3:
						drivePtr -> drivePWM -> timer -> Instance -> CCR3 = 0;
						break;
					case CHANNEL_4:
						drivePtr -> drivePWM -> timer -> Instance -> CCR4 = 0;
						break;
					case CHANNEL_5:
						drivePtr -> drivePWM -> timer -> Instance -> CCR5 = 0;
						break;
					case CHANNEL_6:
						drivePtr -> drivePWM -> timer -> Instance -> CCR6 = 0;
						break;
					default:
						break;
				}
			}
		}
}
