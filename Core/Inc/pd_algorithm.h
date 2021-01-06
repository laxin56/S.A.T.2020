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
#define KP1 150
#define KD1 70

#define KP2 130
#define KD2 50

#define KP3 200
#define KD3 70

//Declaration of pd algorithm functions
double Correct1(double intended_val, double real_val, double dt, double*, double*, double*);
double Correct2(double intended_val, double real_val, double dt, double*, double*, double*);
double Correct3(double intended_val, double real_val, double dt, double*, double*, double*);
#endif /* INC_PD_ALGORITHM_H_ */
