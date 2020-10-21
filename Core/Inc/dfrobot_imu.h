#ifndef INC_DFROBOT_IMU_H_
#define INC_DFROBOT_IMU_H_

#include "bno055.h"


void IMU_Initialize(I2C_HandleTypeDef*);

uint8_t Gyro_Data(I2C_HandleTypeDef*, uint8_t*);

uint8_t Euler_Data(I2C_HandleTypeDef*, uint8_t*, uint8_t*, uint8_t*);



#endif /* INC_DFROBOT_IMU_H_ */
