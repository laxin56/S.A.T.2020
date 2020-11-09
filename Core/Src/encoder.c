/*
 * PD.c
 *
 *  Created on: Nov 4, 2020
 *      Author: nowik
 */

//Plik zawiera implementację obslugi enkodera


#include "encoder.h"

double velocity = 0;

/*
 * This function start timer encoder mode
 *
 */
void Encoder_Start(Encoder_HandleTypeDef* encoder)
	{

		HAL_TIM_Encoder_Start(encoder->timer, encoder->channel);
	}


/*
 * This function is getting and calculating data from encoder
 * Output is velocity in rotations per minute
 * In param - time must be in seconds
 */
double Get_encoder_readings(Encoder_HandleTypeDef* encoder, double time)
	{

		//Getting impulses from TIM counter register
		encoder->actual_impulse = encoder->timer->Instance->CNT;

		//Calculating a velocity in rotations per second
		velocity = encoder->actual_impulse/(encoder->max_impulse*time);

		//Calculating a velocity in rotations per minute
		velocity = velocity*60;

		//Restarting counter register
		encoder->timer->Instance->CNT = 0;

		return velocity;

	}



