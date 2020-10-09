#include "main.h"
#include "dfrobot_imu.h"
#include "bno055.h"

#define IMU_ADDRESS (BNO055_I2C_ADDR_LO << 1)

//Variables for GYRO Settings
uint8_t GYRO_Bandwith = GBW_230Hz;
uint8_t GYRO_Range = GFS_2000DPS;
uint8_t GYRO_Mode = NormalG;

//Variables for BNO055 Settings
uint8_t Power_Mode = Normalpwr;
uint8_t Operation_Mode = GYROONLY;  //Only GYRO is working

void IMU_Initialize(I2C_HandleTypeDef *hi2c_d)
{

	//Getting in Config Mode
	uint8_t config_mode[2] = {BNO055_OPR_MODE, CONFIGMODE};
	HAL_I2C_Master_Transmit(&hi2c_d, IMU_ADDRESS, config_mode, sizeof(config_mode), 10);
	HAL_Delay(10);

	//Now Page 1 to configure sensors
	uint8_t page_1[2] = {BNO055_PAGE_ID, 0x01};
	HAL_I2C_Master_Transmit(&hi2c_d, IMU_ADDRESS, page_1, sizeof(page_1), 10);
	HAL_Delay(10);

	//Now configuring Gyroscope

	    // Configuring Bandwith and Range
		uint8_t config_gyro_0[2] = {BNO055_GYRO_CONFIG_0, GYRO_Bandwith << 3 | GYRO_Range };
		HAL_I2C_Master_Transmit(&hi2c_d, IMU_ADDRESS, config_gyro_0, sizeof(config_gyro_0), 10);
		HAL_Delay(10);

		//Configuring Operation Mode
		uint8_t config_gyro_1[2] = {BNO055_GYRO_CONFIG_1, GYRO_Mode };
		HAL_I2C_Master_Transmit(&hi2c_d, IMU_ADDRESS, config_gyro_1, sizeof(config_gyro_1), 10);
		HAL_Delay(10);

	//Now changing Page to 0 to configure Mode of BNO055
	uint8_t page_0[2] = {BNO055_PAGE_ID, 0x00};
	HAL_I2C_Master_Transmit(&hi2c_d, IMU_ADDRESS, page_0, sizeof(page_0), 10);
	HAL_Delay(10);

	//Changing Power Mode of BNO055
	uint8_t power_mode[2] = {BNO055_PWR_MODE, Power_Mode};
	HAL_I2C_Master_Transmit(&hi2c_d, IMU_ADDRESS, power_mode, sizeof(power_mode), 10);
	HAL_Delay(10);

	//Changing Operation Mode of BNO055
	uint8_t operation_mode[2] = {BNO055_OPR_MODE, Operation_Mode};
	HAL_I2C_Master_Transmit(&hi2c_d, IMU_ADDRESS, operation_mode, sizeof(operation_mode), 10);
	HAL_Delay(100);

}
