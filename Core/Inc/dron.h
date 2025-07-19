/*
 * dron.h
 *
 *  Created on: Aug 29, 2023
 *      Author: Normalny
 */

#ifndef INC_DRON_H_
#define INC_DRON_H_
#include "MadgwickAHRS.h"
#include "HC-SR04.h"
#include "main.h"


#define thrust_increse_factor 	0.0006 // w procentach od 0-100% np. 10% = 0.1
#define thrust_stability_factor 1
#define DEBUG_USER 0


struct Dronstatus{
	uint8_t Connection;
	uint8_t Battery;
	uint8_t Code;
	uint8_t Wobble;
	uint8_t On_Grund;
};



struct Dron_Pitch{
	int16_t Wanted_rx;
	int16_t Wanted_v;
	float Last_Wanted_rx;
	float Wanted;
	float Wanted_Factor;
	float Now;
	float Angle_Error;
	float Angular_Rate_Error;
	float Old_Angle_Error;
	float Old_Angular_Rate_Error;
	double Angle_Error_Sum;
	double Angular_Rate_Error_Sum;
	int16_t Value;
};
struct Dron_Rool{
	int16_t Wanted_rx;
	int16_t Wanted_v;
	float Last_Wanted_rx;
	float Wanted;
	float Wanted_Factor;
	float Now;
	float Angle_Error;
	float Angular_Rate_Error;
	float Old_Angle_Error;
	float Old_Angular_Rate_Error;
	double Angle_Error_Sum;
	double Angular_Rate_Error_Sum;
	int16_t Value;
};
struct Dron_Yaw{
	int16_t Wanted_rx;
	int16_t Wanted_v;
	float Last_Wanted_rx;
	float Wanted;
	float Wanted_Factor;
	float Now;
	float Old;
	int32_t Count;
	float Angle_Error;
	float Angular_Rate_Error;
	float Old_Angle_Error;
	float Old_Angular_Rate_Error;
	double Angle_Error_Sum;
	double Angular_Rate_Error_Sum;
	int16_t Value;
};
struct Dron_Thrust{
	uint16_t Now;
	int16_t Wanted;
	int16_t Thrust_Limit;
	double Values;
	uint16_t Speed_1;
	uint16_t Speed_2;
	uint16_t Speed_3;
	uint16_t Speed_4;
	uint16_t Old_Speed_1;
	uint16_t Old_Speed_2;
	uint16_t Old_Speed_3;
	uint16_t Old_Speed_4;
	uint8_t Max_Flag;
	float Increse_factor;
};
struct PID_Pitch{
	float Angle_Value;
	float Angular_Rate_Value;
	float Angle_Factors[6];
	float Angular_Rate_Factors[6];
	uint8_t Status;
#if DEBUG_USER == 1
	float P_A;
	float I_A;
	float D_A;
	float P_AR;
	float I_AR;
	float D_AR;
#endif
};
struct PID_Rool{
	float Angle_Value;
	float Angular_Rate_Value;
	float Angle_Factors[6];
	float Angular_Rate_Factors[6];
	uint8_t Status;
#if DEBUG_USER == 1
	float P_A;
	float I_A;
	float D_A;
	float P_AR;
	float I_AR;
	float D_AR;
#endif
};
struct PID_Yaw{
	float Angle_Value;
	float Angular_Rate_Value;
	float Angle_Factors[6];
	float Angular_Rate_Factors[6];
	uint8_t Status;
#if DEBUG_USER == 1
	float P_A;
	float I_A;
	float D_A;
	float P_AR;
	float I_AR;
	float D_AR;
#endif
};

struct Take_Off_Mode{
	uint8_t Status; // 1 - True; 0 - False
	struct PID_Rool Rool;
	struct PID_Pitch Pitch;
	struct PID_Yaw Yaw;
};

typedef struct Dron{
	struct Dron_Pitch Pitch;
	struct Dron_Rool Rool;
	struct Dron_Yaw Yaw;
	struct Dron_Thrust Thrust;
	struct PID_Pitch PID_Pitch;
	struct PID_Rool PID_Rool;
	struct PID_Yaw PID_Yaw;
	struct Dronstatus Status;
	struct Take_Off_Mode TOM;
	uint16_t batterysize;
	float barometrick_height;
	float to_ground_height;
	double TEAES;// Thrust error angle error sum
}Dron;

enum DRON_StateTypeDef{
	DRON_CONNECTED = 0x01,
	DRON_DISCONNECTED = 0x02,
	DRON_CONNECTION_ERROR = 0x03,

    DRON_CODE_HARDFAULT_ERROR = 0x04,
	DRON_CODE_OK = 0x05,
	DRON_CODE_STUCK = 0x06,

	DRON_BATTERY_RUN_OUT = 0x07,
	DRON_BATTERY_OK = 0x08,
	DRON_BATTERY_CRIT_VAL = 0x09,

	DRON_ENGINE_ERROR = 0x00,
	DRON_ENGINE_OK = 0x0A,
	DRON_ENGINE_BIG_CURRENT = 0x0B,

	DRON_POSITION_OK = 0x0C,
	DRON_POSITION_WRONG = 0x0D,

	THRUST_MAX = 0x10,

	NO_WOBBLE = 0x0E,
	WOBBLE_PITCH = 0x0F,
	WOBBLE_ROOL = 0x11,
	WOBBLE_PITCHandROOL = 0x12
};

typedef struct Stack{   // sapisanie danych z ostatniej sekundy
	float olddata[4010];
	uint16_t start_pointer;
	uint16_t end_pointer;
}Stack;

void PID_Values_Save_Check(Dron *a);
void PID_call_2(Dron *Paramiters);
void PID_cal(float *PID_var, float *PID_FAC, uint8_t pry);
void PID_call(Dron *Paramiters);
void Angle_Filter(Dron *a, Madgwick_Struct c);
void Get_batteryvalue(void);
void IS_DRON_ON_GROUND(Dron *a, HC_SR04_Struct c);
void Thrust_filter(double factor);
void Dron_Init(void);
void acceleration_stabilizer(float *g_ax, float *PID_FAC);
float APV(float P_factor, float factor, uint8_t mov);
float ADV(float D_factor, float factor, uint8_t mov);
void Thrust_Angle_error_Filter(Dron *a);
double  Thrust_filter_2(float *PID_FAC_rool, float *PID_FAC_pitch, float *PID_FAC_yaw);
void convert_value_to_array(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend);
void convert_array_to_value(uint8_t arrayfrom[], int16_t *value , uint8_t rangebegin, uint8_t rangeend);
uint32_t potenga(int a, int b);

float WartoscBezwgledna(float a);

void RGB_LED_Set_color(uint8_t R, uint8_t G, uint8_t B);
void RGB_LED_For_BAT(uint8_t batval);
void convert_value_to_array2(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend);
void convert_value_to_array3(float value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend);

int16_t ROOL_MAX_VAL(void);
int16_t ROOL_MIN_VAL(void);
int16_t PITCH_MAX_VAL(void);
int16_t PITCH_MIN_VAL(void);
int16_t YAW_MAX_VAL(void);
int16_t YAW_MIN_VAL(void);
int16_t YAW_GOOD_VAL(void);
int16_t PITCH_GOOD_VAL(void);
int16_t ROOL_GOOD_VAL(void);


#endif /* INC_DRON_H_ */
