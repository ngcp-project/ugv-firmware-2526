/**
  ******************************************************************************
  * @file           : ugv_turret.c
  * @brief          : turret subsystem main file
  ******************************************************************************
  */

#include "ugv_turret.h"

void ugv_initTurret(ugvTurret_t* turret, EncoderVals* enc, TIM_HandleTypeDef *htim_enc,
		ugv_PWM_t* pwm, ugv_DriveMotor_t* motor, TIM_HandleTypeDef *htim_mtr, uint16_t channel_x, uint16_t channel_y){

	//set turret motor and encoder structs
	turret->turretEnc = enc;
	motor->drivePWM = pwm;
	turret->turretMotor = motor;

	//reset turret control variables
	turret->turretHeading = 0.0;
	turret->turretTravel = 0.0;
	turret->setHeading = 0.0;
	turret->headingError = 0.0;

	//initialize encoder
	initEncoders(htim_enc, turret->turretEnc);

	//initialize motor
	ugv_init_PWM(turret->turretMotor, htim_mtr, channel_x, channel_y);

}

void ugv_updateTurret(ugvTurret_t* turret){
	int direction = (int) (turret->turretHeading - turret->setHeading);

	// update the turret encoder
	readEncoder(turret->turretEnc, direction);

	// calculate the amount of travel done by the turret motor
	turret->turretTravel = ((float)(turret->turretEnc->travelRots) * turret->turretEnc->CNTROT) + turret->turretEnc->travel;
	// calculate the angle from the travel done by the motor
	turret->turretHeading = (turret->turretTravel/turret->turretEnc->CNTROT)*360.0;

	/***************
	 * Control
	***************/
	turret->headingError = turret->setHeading - turret->turretHeading;

	if(fabs(turret->headingError) > 2){

		//move motor in direction of angle
		ugv_turret_dir_cntrl(turret, (int)(turret->headingError/fabs(turret->headingError))*32000);

	}
	else{
		//stop the motor when it is within a good error
		ugv_turret_dir_cntrl(turret, 0);
	}

}

void ugv_turret_dir_cntrl(ugvTurret_t *turret, int dutyCycle){

	if (dutyCycle >= 0) // The motor will go CCW or CW (Depends on the wiring of the motor)
	{
		//change direction of the motor to one direction


		dutyCycle = abs(dutyCycle);

		*turret->CCRXCH1 = dutyCycle;
		*turret->CCRXCH2 = 0;

	}
	else	// The motor will go in the opposite direction of the other ^^^ (Depends on the wiring of the motor)
	{
		//change direction of the motor to the other direction

		dutyCycle = abs(dutyCycle);

		*turret->CCRXCH1 = 0;
		*turret->CCRXCH2 = dutyCycle;

	}
}

void ugv_turret_moveToAngle(ugvTurret_t *turret, float setAngle){

	turret->setHeading = setAngle;	//change the desired angle to the angle you want to move to

}

void ugv_turret_setOrigin(ugvTurret_t *turret){

	*turret->turretEnc->CNT = 0;
	turret->turretEnc->travelRots = 0;
	turret->turretEnc->travel = 0;
}

