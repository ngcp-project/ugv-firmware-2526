/*
 * ENCODER.h
 *
 *  Created on: Oct 20, 2022
 *      Author: Luke Gutierrez
 *
 *      Encoder Driver Tailored for NGCP
 *
 *		Ver. 1.1
 */
#ifndef UGV_ENCODER_H
#define UGV_ENCODER_H

#include "stm32f7xx_hal.h"
#include <stdlib.h>


typedef struct {
   int motorCPR;
   float gearRatio;
   float sampleRate;
   float wheelC;
   int noLoadRPM;


   int travelRots; //total amount of travel in rotations
   int travel;	//total mount of travel in counts
   float deltaT;	//Delta  in travel in feet
   int deltaR;	//Delta in rotation in counts
   float avgV;	//Velocity Delta in feet/sec
   float RPM;		//Current RPM
   float CNTROT;	//Number of counts per rotation
   int prevCNT; //previous timer CNT
   int preSETPOINT;

   volatile long unsigned int* CNT; //CNT of timer register

} EncoderVals;


void initEncoders(TIM_HandleTypeDef *htim, EncoderVals *Vals);

void readEncoder(EncoderVals *Vals, int SETPOINT);

void updateTravel(EncoderVals *Vals);

#endif
