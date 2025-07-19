/*
 * HMC5883L.c
 *
 *  Created on: Sep 2, 2024
 *      Author: Normalny
 */


#include "main.h"


static I2C_HandleTypeDef *hi2c_HMC5883L;
extern HMC5883L_Struct HMC5883L;

uint8_t HMC5883L_Init(I2C_HandleTypeDef*hi2c){

	hi2c_HMC5883L = hi2c;

	HMC5883L.Directions.Heading = 0;
	HMC5883L.Directions.Old_X = 0;
	HMC5883L.Directions.Old_Y = 0;
	HMC5883L.Directions.Old_Z = 0;
	HMC5883L.Directions.X = 0;
	HMC5883L.Directions.Y = 0;
	HMC5883L.Directions.Z = 0;
	HMC5883L.Directions.X_max = 0.0394495428*1090;
	HMC5883L.Directions.X_min = -0.298165143*1090;
	HMC5883L.Directions.Y_max = 0.250458717*1090;
	HMC5883L.Directions.Y_min = -0.103669725*1090;
	HMC5883L.Directions.Z_max = 0.072477065*1090;
	HMC5883L.Directions.Z_min = -0.223853216*1090;
	HMC5883L.Directions.X_fac = (HMC5883L.Directions.X_max + HMC5883L.Directions.X_min)/2;
	HMC5883L.Directions.Y_fac = (HMC5883L.Directions.Y_max + HMC5883L.Directions.Y_min)/2;
	HMC5883L.Directions.Z_fac = (HMC5883L.Directions.Z_max + HMC5883L.Directions.Z_min)/2;
	HMC5883L.Directions.X_normalized = 0;
	HMC5883L.Directions.Y_normalized = 0;
	HMC5883L.Directions.Z_normalized = 0;
	HMC5883L.HMC583L_IRQ = 0;
	HMC5883L.Off_Set_Values.X = 0;
	HMC5883L.Off_Set_Values.Y = 0;
	HMC5883L.Off_Set_Values.Z = 0;

	uint8_t data = 0;

	/*
	 * bit7 = 1
	 * bit6 = 1		( 8 samples
	 * bit5 = 1		)
	 * bit4 = 1 	{
	 * bit3 = 0			15 HzOutput rate
	 * bit2 = 0		}
	 * bit1 = 0
	 * bit0 = 0
	 */
	data = 0x70;

	HAL_I2C_Mem_Write(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Configuration_Register_A, 1, &data, 1, 1);
	/*
	 *
	 * bit7 = 0	{
	 * bit6 = 0		+-1.3 Ga (1090 LSB/Gauss)
	 * bit5 = 1 }
	 * bit4 = 0
	 * bit3 = 0
	 * bit2 = 0
	 * bit1 = 0
	 * bit0 = 0
	 */
	data = 0xA0;

	HAL_I2C_Mem_Write(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Configuration_Register_B, 1, &data, 1, 1);
	/*
	 *
	 * bit7 = 1
	 * bit6 = 0
	 * bit5 = 0
	 * bit4 = 0
	 * bit3 = 0
	 * bit2 = 0
	 * bit1 = 0	{ Continuous-Measurement Mode
	 * bit0 = 0 }
	 */
	data = 0x00;

	HAL_I2C_Mem_Write(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Mode_Register, 1, &data, 1, 1);


	uint8_t status = 0;
	HAL_Delay(10);

	HAL_I2C_Mem_Read(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Identifaction_Register_A, 1, &status, 1, 1);

	if(status != 0x48){
		status = 0;
	}
	else{
		status = 1;
	}
	HAL_Delay(10);

	return status;
}


int16_t HMC5883L_Get_Z_Start(void){
	int16_t fulldata = 182;
	uint8_t data[6];

	HAL_I2C_Mem_Read(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Data_Output_X_MSB_Register, 1, data, 6, 1);

	fulldata = ((int16_t)data[2]<<8) | data[3];

	HMC5883L.Directions.X = (int16_t)((data[0]<<8) | data[1]);
	HMC5883L.Directions.Z = (int16_t)((data[2]<<8) | data[3]);
	HMC5883L.Directions.Y = (int16_t)((data[4]<<8) | data[5]);

	HMC5883L.Directions.X -= HMC5883L.Directions.X_fac; //offset
	HMC5883L.Directions.Y -= HMC5883L.Directions.Y_fac;
	HMC5883L.Directions.Z -= HMC5883L.Directions.Z_fac;

	HMC5883L.Directions.X /= 1090; //scaling
	HMC5883L.Directions.Y /= 1090;
	HMC5883L.Directions.Z /= 1090;

	return fulldata;
}

void HMC5883L_Get_Z_Start_IT(void){
	HAL_I2C_Mem_Read_IT(hi2c_HMC5883L, HMC5883L_I2C_Address<<1, HMC5883L_Data_Output_X_MSB_Register, 1, (uint8_t *)HMC5883L.I2C_Data, 6);
	HMC5883L.HMC583L_IRQ = 1;
}

void HMC5883L_Get_Z_End_IT(void){
		HMC5883L.Directions.X = (int16_t)((HMC5883L.I2C_Data[0]<<8) | HMC5883L.I2C_Data[1]);
		HMC5883L.Directions.Z = (int16_t)((HMC5883L.I2C_Data[2]<<8) | HMC5883L.I2C_Data[3]);
		HMC5883L.Directions.Y = (int16_t)((HMC5883L.I2C_Data[4]<<8) | HMC5883L.I2C_Data[5]);

		HMC5883L.Directions.X -= HMC5883L.Directions.X_fac; //offset
		HMC5883L.Directions.Y -= HMC5883L.Directions.Y_fac;
		HMC5883L.Directions.Z -= HMC5883L.Directions.Z_fac;

		HMC5883L.Directions.X /= 1090; //scaling
		HMC5883L.Directions.Y /= 1090;
		HMC5883L.Directions.Z /= 1090;

//		HMC5883L.Directions.X -= HMC5883L.Directions.Initial_X;
//		HMC5883L.Directions.Y -= HMC5883L.Directions.Initial_Y;
//		HMC5883L.Directions.Z -= HMC5883L.Directions.Initial_Z;

		//HMC5883L.Directions.X = HMC5883L.Directions.X*cos(HMC5883L.Directions.Initial_Yaw) + HMC5883L.Directions.Y*sin(HMC5883L.Directions.Initial_Yaw);
		//HMC5883L.Directions.Y = -HMC5883L.Directions.X*sin(HMC5883L.Directions.Initial_Yaw) + HMC5883L.Directions.Y*cos(HMC5883L.Directions.Initial_Yaw);

		//HMC5883L_Cal();

		HMC5883L.Directions.Heading = (atan2(HMC5883L.Directions.X, HMC5883L.Directions.Y))*180/M_PI;

		float magnitude = sqrt(HMC5883L.Directions.X*HMC5883L.Directions.X + HMC5883L.Directions.Y*HMC5883L.Directions.Y + HMC5883L.Directions.Z*HMC5883L.Directions.Z);

		HMC5883L.Directions.X_normalized = HMC5883L.Directions.X/magnitude;
		HMC5883L.Directions.Y_normalized = HMC5883L.Directions.Y/magnitude;
		HMC5883L.Directions.Z_normalized = HMC5883L.Directions.Z/magnitude;
}


int16_t HMC5883L_Calibration(void){
	int64_t mes_data = 0;
	for(int i = 0; i < 10; i++){
		mes_data += HMC5883L_Get_Z_Start();
		HAL_Delay(100); //Output = 30Hz
	}

	HMC5883L.Directions.Initial_X = HMC5883L.Directions.X;
	HMC5883L.Directions.Initial_Y = HMC5883L.Directions.Y;
	HMC5883L.Directions.Initial_Z = HMC5883L.Directions.Z;

	return (mes_data/10);
}

void HMC5883L_Cal(void){
	HMC5883L.Directions.X_max = HMC5883L.Directions.X > HMC5883L.Directions.X_max ? HMC5883L.Directions.X : HMC5883L.Directions.X_max;
	HMC5883L.Directions.Y_max = HMC5883L.Directions.Y > HMC5883L.Directions.Y_max ? HMC5883L.Directions.Y : HMC5883L.Directions.Y_max;
	HMC5883L.Directions.Z_max = HMC5883L.Directions.Z > HMC5883L.Directions.Z_max ? HMC5883L.Directions.Z : HMC5883L.Directions.Z_max;

	HMC5883L.Directions.X_min = HMC5883L.Directions.X < HMC5883L.Directions.X_min ? HMC5883L.Directions.X : HMC5883L.Directions.X_min;
	HMC5883L.Directions.Y_min = HMC5883L.Directions.Y < HMC5883L.Directions.Y_min? HMC5883L.Directions.Y : HMC5883L.Directions.Y_min;
	HMC5883L.Directions.Z_min = HMC5883L.Directions.Z < HMC5883L.Directions.Z_min ? HMC5883L.Directions.Z : HMC5883L.Directions.Z_min;
}
