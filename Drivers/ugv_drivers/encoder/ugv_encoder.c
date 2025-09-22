/*
 * ENCODER.c
 *
 *  Created on: Oct 20, 2022
 *      Author: Luke Gutierrez
 *
 *      Ver. 1.1
 */

#include "ugv_encoder.h"

void initEncoders(TIM_HandleTypeDef *htim, EncoderVals *Vals){
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

	Vals->travel = 0;
	Vals->deltaT = 0;
	Vals->deltaR = 0;
	Vals->avgV = 0;
	Vals->RPM = 0;
	Vals->prevCNT =0;
	Vals->CNTROT = Vals->motorCPR*Vals->gearRatio;

}


//Reads data from quadature encoder and processes must be placed in ISR with appropiate sample rate
void readEncoder(EncoderVals *Vals, int SETPOINT){

	uint16_t MTR_CNT = *Vals->CNT;

	/*Set Current delta R
	if (SETPOINT > 0) {
		if (MTR_CNT < Vals->prevCNT) {
			Vals->deltaR = MTR_CNT + (0xFFFF - Vals->prevCNT);
		} else {
			Vals->deltaR = MTR_CNT - Vals->prevCNT;
		}
	} else if (SETPOINT < 0) {
		if (MTR_CNT > Vals->prevCNT) {
			Vals->deltaR = -((0xFFFF - MTR_CNT) + Vals->prevCNT);
		} else {
			Vals->deltaR = MTR_CNT - Vals->prevCNT;
		}
		Vals->preSETPOINT = SETPOINT;
	} else if (SETPOINT == 0){

		Vals->deltaR = MTR_CNT - Vals->prevCNT;

	}*/

	Vals->deltaR = MTR_CNT - Vals->prevCNT;


	updateTravel(Vals);

	Vals->deltaT = ((float) Vals->deltaR / (float) Vals->CNTROT)
			* Vals->wheelC;

	Vals->avgV = Vals->deltaT / Vals->sampleRate;

	float RPM = ((float) Vals->deltaR / (4*(float) Vals->CNTROT)) / (Vals->sampleRate / 60);


	if(RPM > Vals->noLoadRPM){
		Vals->RPM = Vals->noLoadRPM;
	} else if (RPM < -Vals->noLoadRPM) {
		Vals->RPM = -Vals->noLoadRPM;
	} else {
		Vals->RPM = RPM;
	}

	Vals->prevCNT = MTR_CNT;


}



void updateTravel(EncoderVals *Vals) {
	if (abs(Vals->deltaR) < Vals->CNTROT) {
		Vals->travel += Vals->deltaR;
		if ((Vals->travel > Vals->CNTROT) && (Vals->travel > 0)) {
			Vals->travelRots += 1;
			Vals->travel -= Vals->CNTROT;
		}
		if ((Vals->travel < -Vals->CNTROT) && (Vals->travel < 0)) {
			Vals->travelRots -= 1;
			Vals->travel += Vals->CNTROT;
		}
	}
}
