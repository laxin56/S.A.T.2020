/*
 * motor_driver.c
 *
 *  Created on: 20 paź 2020
 *      Author: nowik
 */

#include "motor_driver.h"

void Start_PWM_Motor_Z(Motor_HandleTypeDef* motor){

	//Begin PWM
	HAL_TIM_PWM_Start(motor->timer, motor->channel);

}

/*
 * Will be used for setting direction of rotation and speed for all 3 motors.
 * ROLL, PITCH and YAW
 */
void Speed_Motor(Motor_HandleTypeDef* motor, unsigned int rotation, int duty){

	//Rotations on the left
	if(rotation == 1)
	{
		HAL_GPIO_WritePin(motor->gpio_port, motor->pin_gpio, GPIO_PIN_SET);
	}
	//rotation on the right
	else if(rotation == 0)
	{
		HAL_GPIO_WritePin(motor->gpio_port, motor->pin_gpio, GPIO_PIN_RESET);
	}

	//Set speed
	// duty can be set like 0-1000
	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel, duty);

}

