/*
 * PD.h
 *
 *  Created on: Nov 6, 2020
 *      Author: nowik
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

extern double velocity;

typedef struct encoder{

	TIM_HandleTypeDef* timer;
	uint32_t channel;

	uint16_t actual_impulse;
	uint16_t max_impulse;

}Encoder_HandleTypeDef;


void Encoder_Start(Encoder_HandleTypeDef* encoder);

double Get_encoder_readings(Encoder_HandleTypeDef* encoder, double time);

#endif /* INC_ENCODER_H_ */
