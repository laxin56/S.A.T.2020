#include "main.h"
#include "dfrobot_imu.h"
#include "bno055.h"

#define IMU_ADDRESS (BNO055_I2C_ADDR_LO << 1)

//Variables for GYRO Settings
uint8_t GYRO_Bandwith = GBW_116Hz;
uint8_t GYRO_Range = GFS_2000DPS;
uint8_t GYRO_Mode = NormalG;

//Variables for BNO055 Settings
uint8_t Power_Mode = Normalpwr;
uint8_t Operation_Mode = NDOF;  //Only GYRO is working


//Global variables for angle values

void IMU_Initialize(I2C_HandleTypeDef *hi2c_d)
{

	//Getting in Config Mode
	uint8_t config_mode[2] = {BNO055_OPR_MODE, CONFIGMODE};
	HAL_I2C_Master_Transmit(hi2c_d, IMU_ADDRESS, config_mode, sizeof(config_mode), 10);
	HAL_Delay(10);

	//Now Page 1 to configure sensors
	uint8_t page_1[2] = {BNO055_PAGE_ID, 0x01};
	HAL_I2C_Master_Transmit(hi2c_d, IMU_ADDRESS, page_1, sizeof(page_1), 10);
	HAL_Delay(10);

	//Now configuring Gyroscope

	    // Configuring Bandwith and Range
		uint8_t config_gyro_0[2] = {BNO055_GYRO_CONFIG_0, GYRO_Bandwith << 3 | GYRO_Range };
		HAL_I2C_Master_Transmit(hi2c_d, IMU_ADDRESS, config_gyro_0, sizeof(config_gyro_0), 10);
		HAL_Delay(10);

		//Configuring Operation Mode
		uint8_t config_gyro_1[2] = {BNO055_GYRO_CONFIG_1, GYRO_Mode };
		HAL_I2C_Master_Transmit(hi2c_d, IMU_ADDRESS, config_gyro_1, sizeof(config_gyro_1), 10);
		HAL_Delay(10);

	//Now changing Page to 0 to configure Mode of BNO055
	uint8_t page_0[2] = {BNO055_PAGE_ID, 0x00};
	HAL_I2C_Master_Transmit(hi2c_d, IMU_ADDRESS, page_0, sizeof(page_0), 10);
	HAL_Delay(10);

	//Changing Power Mode of BNO055
	uint8_t power_mode[2] = {BNO055_PWR_MODE, Power_Mode};
	HAL_I2C_Master_Transmit(hi2c_d, IMU_ADDRESS, power_mode, sizeof(power_mode), 10);
	HAL_Delay(10);

	//Changing Operation Mode of BNO055
	uint8_t operation_mode[2] = {BNO055_OPR_MODE, Operation_Mode};
	HAL_I2C_Master_Transmit(hi2c_d, IMU_ADDRESS, operation_mode, sizeof(operation_mode), 10);
	HAL_Delay(100);

}

//UNFINISHED FUNCTION
uint8_t Gyro_Data(I2C_HandleTypeDef *hi2c_d, uint8_t* gyro_data)
{
	uint8_t status;

	//Read Gyro Data in Multipleread starting with x LSB
	// 6 for: X - L i M, Y - L i M, Z - L i M
	//Katy eulera
	status = HAL_I2C_Mem_Read(hi2c_d, IMU_ADDRESS, BNO055_EUL_ROLL_LSB, I2C_MEMADD_SIZE_8BIT, gyro_data, 6, 100);

	return status;
}


/*
 * Functions that gets Euler Angles
 * 6 for: X - L i M, Y - L i M, Z - L i M
 *	all six bites - lower and higher
 *	For ROLL, PITCH AND YAW
 */


void Euler_Data(I2C_HandleTypeDef *hi2c_d, double *x, double *y, double *z){


	uint8_t eul_roll_x[2];
	uint8_t eul_pitch_y[2];
	uint8_t eul_heading_z[2];

	int16_t	 gyro_x, gyro_y, gyro_z;


	//roll
	HAL_I2C_Mem_Read(hi2c_d, IMU_ADDRESS, BNO055_EUL_ROLL_LSB, I2C_MEMADD_SIZE_8BIT, eul_roll_x, 2, 100);
	//pitch
	HAL_I2C_Mem_Read(hi2c_d, IMU_ADDRESS, BNO055_EUL_PITCH_LSB, I2C_MEMADD_SIZE_8BIT, eul_pitch_y, 2, 100);
	//yaw - heading
	HAL_I2C_Mem_Read(hi2c_d, IMU_ADDRESS, BNO055_EUL_HEADING_LSB, I2C_MEMADD_SIZE_8BIT, eul_heading_z, 2, 100);

	//Connecting Lower ang higher bites of data
	gyro_x = (int16_t)(((uint8_t)eul_roll_x[1] << 8)  | (uint8_t)eul_roll_x[0]);

	gyro_y = (int16_t)(((uint8_t)eul_pitch_y[1] << 8)  | (uint8_t)eul_pitch_y[0]);

	gyro_z = (int16_t)(((uint8_t)eul_heading_z[1] << 8)  | (uint8_t)eul_heading_z[0]);

	//Transfering data to correct unit - angle
	*x = ((double)(gyro_x))/16.0f;
	*y = ((double)(gyro_y))/16.0f;
	*z = ((double)(gyro_z))/16.0f;

}


/*
void Euler_Data(I2C_HandleTypeDef *hi2c_d, uint8_t* eul_roll_x, uint8_t* eul_pitch_y, uint8_t* eul_heading_z){

	//roll
	HAL_I2C_Mem_Read(hi2c_d, IMU_ADDRESS, BNO055_EUL_ROLL_LSB, I2C_MEMADD_SIZE_8BIT, eul_roll_x, 2, 100);
	//pitch
	HAL_I2C_Mem_Read(hi2c_d, IMU_ADDRESS, BNO055_EUL_PITCH_LSB, I2C_MEMADD_SIZE_8BIT, eul_pitch_y, 2, 100);
	//yaw - heading
	HAL_I2C_Mem_Read(hi2c_d, IMU_ADDRESS, BNO055_EUL_HEADING_LSB, I2C_MEMADD_SIZE_8BIT, eul_heading_z, 2, 100);


}
*/
