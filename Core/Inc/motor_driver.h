/*
 * motor_driver.h
 *
 *  Created on: 20 paź 2020
 *      Author: nowik
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "main.h"

typedef struct Motor{

	TIM_HandleTypeDef* timer; 	//uchwyt do timera
	uint32_t channel;  			//uchwyt do kanału

	uint16_t pin_gpio;			//uchwyt do pinu GPIO - zmiana kierunku rotacji
	GPIO_TypeDef* gpio_port;	//uchwyt do portu GPIO - zmiana kierunku rotacji

	//unsigned int duty;			//Wypelnienie PWM

}Motor_HandleTypeDef;

void Start_PWM_Motor_Z(Motor_HandleTypeDef* motor1);
void Speed_Motor(Motor_HandleTypeDef* motor, unsigned int rotation, int duty);

#endif /* INC_MOTOR_DRIVER_H_ */
