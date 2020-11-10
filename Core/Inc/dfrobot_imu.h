#ifndef INC_DFROBOT_IMU_H_
#define INC_DFROBOT_IMU_H_

#include "bno055.h"

/*
extern uint8_t* eul_roll_x;
extern uint8_t* eul_pitch_y;
extern uint8_t* eul_heading_z;

extern int16_t	 gyro_x, gyro_y, gyro_z;
*/
void IMU_Initialize(I2C_HandleTypeDef*);

uint8_t Gyro_Data(I2C_HandleTypeDef*, uint8_t*);

void Euler_Data(I2C_HandleTypeDef*, double *, double *, double *);

//void Euler_Data(I2C_HandleTypeDef *hi2c_d, uint8_t* eul_roll_x, uint8_t* eul_pitch_y, uint8_t* eul_heading_z);

#endif /* INC_DFROBOT_IMU_H_ */
