/*
 * pd_algorithm.c
 *
 *  Created on: 6 lis 2020
 *      Author: nowik
 */

#include "pd_algorithm.h"

double out = 0;

/*
 * This function is calculating the correct output
 * Out - corrected value for the output
 * In param
 * intended_val - value that we want achieve
 * real_val - value that comes from the sensor
 * dt - time that passed since last calculation
 * m - mistake
 * last_m - remembering previous mistake
 * derivative - variable for calculating derivative in algorithm equations
 */
double Correct(float intended_val, float real_val, uint16_t dt, double* m, double *last_m, double *derivative)
	{


		*m = intended_val - real_val;

		*derivative = (*m - *last_m)*dt;

		out = (*m)*KP + (*derivative)*KD;

		last_m = m;

		return out;

	}


