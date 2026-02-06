#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "string.h"

typedef struct {

    TIM_HandleTypeDef* htim; // Timer for PWM control
    uint8_t channel_1;
    uint8_t channel_2;// PWM channel for motor control
} MotorControl;

void MotorControl_Init(MotorControl* motor, TIM_HandleTypeDef* timer, uint8_t channel_1, uint8_t channel_2);
void MotorControl_SetSpeed(MotorControl* motor, TIM_HandleTypeDef* htim, float input_speed);

#endif /* MOTOR_CONTROL_H */
