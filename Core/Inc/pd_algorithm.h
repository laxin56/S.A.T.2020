/*
 * pd_algorithm.h
 *
 *  Created on: 6 lis 2020
 *      Author: nowik
 */



#ifndef INC_PD_ALGORITHM_H_
#define INC_PD_ALGORITHM_H_

#include "main.h"

//Variables to correctly set regulator PD
#define KP 11
#define KD 12

extern double out;

//Declaration of pd algorithm function
double Correct(float intended_val, float real_val, uint16_t dt, double* blad, double *wczesniejszy_blad, double *pochodna);

#endif /* INC_PD_ALGORITHM_H_ */
