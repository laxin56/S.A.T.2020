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
#define KP 1000
#define KD 500



//Declaration of pd algorithm function
double Correct(double intended_val, double real_val, double dt, double*, double*, double*);

#endif /* INC_PD_ALGORITHM_H_ */
