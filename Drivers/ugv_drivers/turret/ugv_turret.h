/**
  ******************************************************************************
  * @file           : ugv_turret.h
  * @brief          : turret subsystem header
  ******************************************************************************
  */

#ifndef UGV_TURRET_H
#define UGV_TURRET_H

#include "../encoder/ugv_encoder.h"
#include "../drive_motor/ugv_drive_motor_utilities.h"
#include "../pid/ugv_pid.h"
#include "math.h"


/**
 * Struct to handle turret operations and variables
 */
typedef struct {

	EncoderVals* turretEnc;
	ugv_PWM_t* turretPWM;
	ugv_DriveMotor_t* turretMotor;

	float turretHeading; //Current Heading
	float turretTravel;
	float setHeading; //Set point heading
	float headingError;

	int maxDegrees;
	int minDegrees;

	volatile long unsigned int* CCRXCH1; //Period register for pwm ch 1
	volatile long unsigned int* CCRXCH2; //Period register for pwm ch 2

}ugvTurret_t;

/**
 *
 * Initialize turret subsystem (motor, encoder, and control systems)
 *
 * @param turret: pointer to the turret struct
 * @param enc: pointer to the turret's encoder struct
 * @param htim_enc: pointer to the htim timer struct that the encoder is using
 * @param pwm: pointer to the motor pwm struct that the turret motor is using
 * @param motor: pointer to the motor struct that the turret motor is using
 * @param htim_mtr: pointer to the htim timer struct that the turret motor is using
 * @param channel_x: the channel number of the timer that the motor is being controlled from
 * @param channel_y: the 2nd channel number of the timer that the motor is being controlled from
 */
void ugv_initTurret(ugvTurret_t* turret, EncoderVals* enc, TIM_HandleTypeDef *htim_enc,
		ugv_PWM_t* pwm, ugv_DriveMotor_t* motor, TIM_HandleTypeDef *htim_mtr, uint16_t channel_x, uint16_t channel_y);

/**
 *
 * Update motor encoder and control systems
 * 		- update encoder values
 * 		- update turret heading
 * 		- update turret control system to move to specific heading
 * This should be placed in an interrupt
 *
 * @param turret: pointer to the turret struct
 */
void ugv_updateTurret(ugvTurret_t* turret);

/**
 *
 * Move turret motor by sending a duty cycle value
 * 		- another version of the motor driver function because the turret uses a different
 * 		motor controller
 *
 * @param turret: pointer to the turret struct
 * @param dutyCycle: duty cycle value being set to the motor (-32767 - 32767)
 */
void ugv_turret_dir_cntrl(ugvTurret_t *turret, int dutyCycle);

/**
 *
 * Command the turret to move to a specific angle
 *
 * @param turret: pointer to the turret struct
 * @param setAngle: angle in degrees to move the turret to
 */
void ugv_turret_moveToAngle(ugvTurret_t *turret, float setAngle);

void ugv_turret_setOrigin(ugvTurret_t *turret);

#endif
