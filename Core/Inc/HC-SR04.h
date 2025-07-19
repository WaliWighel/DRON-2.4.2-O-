/*
 * HC-SR04.h
 *
 *  Created on: Jan 18, 2025
 *      Author: Normalny
 */
#include  "main.h"

#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_


#define TRIG_1 HAL_GPIO_WritePin(HC_SR04_Triger_GPIO_Port, HC_SR04_Triger_Pin, GPIO_PIN_SET)
#define TRIG_0 HAL_GPIO_WritePin(HC_SR04_Triger_GPIO_Port, HC_SR04_Triger_Pin, GPIO_PIN_RESET)


typedef struct HC_SR04_Struct{
	uint8_t Step;// 1 - ongoing 0 - non
	uint8_t Triger;
	uint8_t Echo;
	uint32_t Time;
	float Distance;
	float Start_Distance;
}HC_SR04_Struct;

void HC_SR04_Start_Messurment(HC_SR04_Struct *a);
void HC_SR04_End_Messurment(HC_SR04_Struct *a);
void HC_SR04_Get_Echo(HC_SR04_Struct *a);
void HC_SR04_Get_Distance(HC_SR04_Struct *a);
void HC_SR04_Init(HC_SR04_Struct *a, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2);

#endif /* INC_HC_SR04_H_ */
