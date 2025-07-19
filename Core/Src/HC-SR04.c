/*
 * HC-SR04.c
 *
 *  Created on: Jan 18, 2025
 *      Author: Normalny
 */
#include "HC-SR04.h"
#include "tim.h"

static TIM_HandleTypeDef *htim_1_HC_SR04;
static TIM_HandleTypeDef *htim_2_HC_SR04;

void HC_SR04_Start_Messurment(HC_SR04_Struct *a){
	TRIG_1;
	HAL_TIM_Base_Start_IT(htim_1_HC_SR04);
	a->Triger = 1;
	a->Step = 1;
}

void HC_SR04_End_Messurment(HC_SR04_Struct *a){
	TRIG_0;
	HAL_TIM_Base_Stop_IT(htim_1_HC_SR04);
	a->Triger = 0;
	a->Step = 2;
}

void HC_SR04_Get_Echo(HC_SR04_Struct *a){
	switch (a->Echo){
		case 0:
			HAL_TIM_Base_Start(htim_2_HC_SR04);
			a->Echo = 1;
			a->Step = 3;
			break;


		case 1:
			HAL_TIM_Base_Stop(htim_2_HC_SR04);
			a->Time = htim_2_HC_SR04->Instance->CNT/275;//-> us
			htim_2_HC_SR04->Instance->CNT = 0;
			HC_SR04_Get_Distance(a);
			a->Step = 0;
			a->Echo = 0;
			break;
	}
}

void HC_SR04_Get_Distance(HC_SR04_Struct *a){
	a->Distance = (a->Time/58.0f)/100;// [m]
}

/*
 * htim_1_HC_SR04 interupt 10ms
 * htim_2_HC_SR04 counting everu us
 */
void HC_SR04_Init(HC_SR04_Struct *a, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2){
	htim_1_HC_SR04 = htim1;
	htim_2_HC_SR04 = htim2;

	a->Distance = 0;
	a->Echo = 0;
	a->Time = 0;
	a->Triger = 0;
	a->Start_Distance = 0;
	a->Step = 0;


	for(int i = 0; i < 10; i++){
		HC_SR04_Start_Messurment(a);
		HAL_Delay(10);
		HC_SR04_End_Messurment(a);

		while(a->Distance == 0){

		}
	}
	a->Start_Distance = a->Distance;
}
