#include "main.h"
extern Dron MYDRON;
extern MPU6050_Struct MPU6050;
extern Madgwick_Struct Madgwick;
extern HMC5883L_Struct HMC5883L;

extern uint32_t analogmess;
extern float looptime;
extern TIM_HandleTypeDef htim1;




extern uint16_t FDP_D_Gain_AR;
extern uint16_t FDP_D_Gain;


void Dron_Init(void){
	MYDRON.barometrick_height = 0;
	MYDRON.to_ground_height = 0;
	MYDRON.batterysize = 0;

	MYDRON.Pitch.Angle_Error_Sum = 0;
	MYDRON.Pitch.Angle_Error = 0;
	MYDRON.Pitch.Angular_Rate_Error = 0;
	MYDRON.Pitch.Angular_Rate_Error_Sum = 0;
	MYDRON.Pitch.Last_Wanted_rx = 0;
	MYDRON.Pitch.Now = 0;
	MYDRON.Pitch.Old_Angle_Error = 0;
	MYDRON.Pitch.Old_Angular_Rate_Error = 0;
	MYDRON.Pitch.Value = 0;
	MYDRON.Pitch.Wanted = 0;
	MYDRON.Pitch.Wanted_Factor = 0.2167;
	MYDRON.Pitch.Wanted_rx = 0;
	MYDRON.Pitch.Wanted_v = 0;

	MYDRON.Rool.Angle_Error_Sum = 0;
	MYDRON.Rool.Angle_Error = 0;
	MYDRON.Rool.Angular_Rate_Error = 0;
	MYDRON.Rool.Angular_Rate_Error_Sum = 0;
	MYDRON.Rool.Last_Wanted_rx = 0;
	MYDRON.Rool.Now = 0;
	MYDRON.Rool.Old_Angle_Error = 0;
	MYDRON.Rool.Old_Angular_Rate_Error = 0;
	MYDRON.Rool.Value = 0;
	MYDRON.Rool.Wanted = 0;
	MYDRON.Rool.Wanted_Factor = 0.22;
	MYDRON.Rool.Wanted_rx = 0;
	MYDRON.Rool.Wanted_v = 0;

	MYDRON.Yaw.Angle_Error_Sum = 0;
	MYDRON.Yaw.Angle_Error = 0;
	MYDRON.Yaw.Angular_Rate_Error = 0;
	MYDRON.Yaw.Angular_Rate_Error_Sum = 0;
	MYDRON.Yaw.Last_Wanted_rx = 0;
	MYDRON.Yaw.Now = 0;
	MYDRON.Yaw.Count = 0;
	MYDRON.Yaw.Old = 0;
	MYDRON.Yaw.Old_Angle_Error = 0;
	MYDRON.Yaw.Old_Angular_Rate_Error = 0;
	MYDRON.Yaw.Value = 0;
	MYDRON.Yaw.Wanted = 0;
	MYDRON.Yaw.Wanted_Factor = 0.2;
	MYDRON.Yaw.Wanted_rx = 0;
	MYDRON.Yaw.Wanted_v = 0;

	MYDRON.PID_Pitch.Angle_Factors[0] = 6;//15
	MYDRON.PID_Pitch.Angle_Factors[1] = 5;//20
	MYDRON.PID_Pitch.Angle_Factors[2] = 0;//25
	MYDRON.PID_Pitch.Angle_Factors[3] = 0;
	MYDRON.PID_Pitch.Angle_Factors[4] = 0;
	MYDRON.PID_Pitch.Angular_Rate_Factors[0] = 16;//6
	MYDRON.PID_Pitch.Angular_Rate_Factors[1] = 0;//0
	MYDRON.PID_Pitch.Angular_Rate_Factors[2] = 30;//70
	MYDRON.PID_Pitch.Angular_Rate_Factors[3] = 0;
	MYDRON.PID_Pitch.Angular_Rate_Factors[4] = 0;
	MYDRON.PID_Pitch.Angle_Value = 0;
	MYDRON.PID_Pitch.Angular_Rate_Value = 0;

	MYDRON.PID_Rool.Angle_Factors[0] = 6;//4
	MYDRON.PID_Rool.Angle_Factors[1] = 5;//10
	MYDRON.PID_Rool.Angle_Factors[2] = 0;//30
	MYDRON.PID_Rool.Angle_Factors[3] = 0;
	MYDRON.PID_Rool.Angle_Factors[4] = 0;
	MYDRON.PID_Rool.Angular_Rate_Factors[0] = 16;//6
	MYDRON.PID_Rool.Angular_Rate_Factors[1] = 0;//0
	MYDRON.PID_Rool.Angular_Rate_Factors[2] = 30;//8
	MYDRON.PID_Rool.Angular_Rate_Factors[3] = 0;
	MYDRON.PID_Rool.Angular_Rate_Factors[4] = 0;
	MYDRON.PID_Rool.Angle_Value = 0;
	MYDRON.PID_Rool.Angular_Rate_Value = 0;

	MYDRON.PID_Yaw.Angle_Factors[0] = 10;//15
	MYDRON.PID_Yaw.Angle_Factors[1] = 0;//0
	MYDRON.PID_Yaw.Angle_Factors[2] = 0;//10
	MYDRON.PID_Yaw.Angle_Factors[3] = 0;
	MYDRON.PID_Yaw.Angle_Factors[4] = 0;
	MYDRON.PID_Yaw.Angular_Rate_Factors[0] = 10;//8
	MYDRON.PID_Yaw.Angular_Rate_Factors[1] = 0;//0
	MYDRON.PID_Yaw.Angular_Rate_Factors[2] = 0;//15
	MYDRON.PID_Yaw.Angular_Rate_Factors[3] = 0;
	MYDRON.PID_Yaw.Angular_Rate_Factors[4] = 0;
	MYDRON.PID_Yaw.Angle_Value = 0;
	MYDRON.PID_Yaw.Angular_Rate_Value = 0;

	MYDRON.Thrust.Now = 0;
	MYDRON.Thrust.Old_Speed_1 = min_speed;
	MYDRON.Thrust.Old_Speed_2 = min_speed;
	MYDRON.Thrust.Old_Speed_3 = min_speed;
	MYDRON.Thrust.Old_Speed_4 = min_speed;
	MYDRON.Thrust.Speed_1 = min_speed;
	MYDRON.Thrust.Speed_2 = min_speed;
	MYDRON.Thrust.Speed_3 = min_speed;
	MYDRON.Thrust.Speed_4 = min_speed;
	MYDRON.Thrust.Thrust_Limit = 10000;
	MYDRON.Thrust.Values = 0;
	MYDRON.Thrust.Wanted = 0;
	MYDRON.Thrust.Increse_factor = 0.0006;

	MYDRON.Status.Battery = DRON_BATTERY_OK;
	MYDRON.Status.Code = DRON_CODE_OK;
	MYDRON.Status.Connection = DRON_CONNECTED;
	MYDRON.Status.Wobble = NO_WOBBLE;
	MYDRON.TEAES = 0;


	MYDRON.TOM.Status = 1;
	MYDRON.TOM.Pitch.Angle_Factors[0] = MYDRON.PID_Pitch.Angle_Factors[0];
	MYDRON.TOM.Pitch.Angle_Factors[1] = MYDRON.PID_Pitch.Angle_Factors[1];
	MYDRON.TOM.Pitch.Angle_Factors[2] = MYDRON.PID_Pitch.Angle_Factors[2];
	MYDRON.TOM.Pitch.Angle_Factors[3] = MYDRON.PID_Pitch.Angle_Factors[3];
	MYDRON.TOM.Pitch.Angle_Factors[4] = MYDRON.PID_Pitch.Angle_Factors[4];

	MYDRON.TOM.Rool.Angle_Factors[0] = MYDRON.PID_Rool.Angle_Factors[0];
	MYDRON.TOM.Rool.Angle_Factors[1] = MYDRON.PID_Rool.Angle_Factors[1];
	MYDRON.TOM.Rool.Angle_Factors[2] = MYDRON.PID_Rool.Angle_Factors[2];
	MYDRON.TOM.Rool.Angle_Factors[3] = MYDRON.PID_Rool.Angle_Factors[3];
	MYDRON.TOM.Rool.Angle_Factors[4] = MYDRON.PID_Rool.Angle_Factors[4];

	MYDRON.TOM.Yaw.Angle_Factors[0] = MYDRON.PID_Yaw.Angle_Factors[0];
	MYDRON.TOM.Yaw.Angle_Factors[1] = MYDRON.PID_Yaw.Angle_Factors[1];
	MYDRON.TOM.Yaw.Angle_Factors[2] = MYDRON.PID_Yaw.Angle_Factors[2];
	MYDRON.TOM.Yaw.Angle_Factors[3] = MYDRON.PID_Yaw.Angle_Factors[3];
	MYDRON.TOM.Yaw.Angle_Factors[4] = MYDRON.PID_Yaw.Angle_Factors[4];

	MYDRON.TOM.Pitch.Angular_Rate_Factors[0] = MYDRON.PID_Pitch.Angular_Rate_Factors[0];
	MYDRON.TOM.Pitch.Angular_Rate_Factors[1] = MYDRON.PID_Pitch.Angular_Rate_Factors[1];
	MYDRON.TOM.Pitch.Angular_Rate_Factors[2] = MYDRON.PID_Pitch.Angular_Rate_Factors[2];
	MYDRON.TOM.Pitch.Angular_Rate_Factors[3] = MYDRON.PID_Pitch.Angular_Rate_Factors[3];
	MYDRON.TOM.Pitch.Angular_Rate_Factors[4] = MYDRON.PID_Pitch.Angular_Rate_Factors[4];

	MYDRON.TOM.Rool.Angular_Rate_Factors[0] = MYDRON.PID_Rool.Angular_Rate_Factors[0];
	MYDRON.TOM.Rool.Angular_Rate_Factors[1] = MYDRON.PID_Rool.Angular_Rate_Factors[1];
	MYDRON.TOM.Rool.Angular_Rate_Factors[2] = MYDRON.PID_Rool.Angular_Rate_Factors[2];
	MYDRON.TOM.Rool.Angular_Rate_Factors[3] = MYDRON.PID_Rool.Angular_Rate_Factors[3];
	MYDRON.TOM.Rool.Angular_Rate_Factors[4] = MYDRON.PID_Rool.Angular_Rate_Factors[4];

	MYDRON.TOM.Yaw.Angular_Rate_Factors[0] = MYDRON.PID_Yaw.Angular_Rate_Factors[0];//Disables Yaw control
	MYDRON.TOM.Yaw.Angular_Rate_Factors[1] = MYDRON.PID_Yaw.Angular_Rate_Factors[1];
	MYDRON.TOM.Yaw.Angular_Rate_Factors[2] = MYDRON.PID_Yaw.Angular_Rate_Factors[2];
	MYDRON.TOM.Yaw.Angular_Rate_Factors[3] = MYDRON.PID_Yaw.Angular_Rate_Factors[3];
	MYDRON.TOM.Yaw.Angular_Rate_Factors[4] = MYDRON.PID_Yaw.Angular_Rate_Factors[4];

//	MYDRON.TOM.Yaw.Angular_Rate_Factors[0] = 0;//Disables Yaw control
//	MYDRON.TOM.Yaw.Angular_Rate_Factors[1] = 0;
//	MYDRON.TOM.Yaw.Angular_Rate_Factors[2] = 0;
//	MYDRON.TOM.Yaw.Angular_Rate_Factors[3] = 0;
//	MYDRON.TOM.Yaw.Angular_Rate_Factors[4] = 0;
}

/*
 * Thrust_filter
 *
 *  anti angle error
 *
 */
void Thrust_filter(double factor){
	double error_pitch, error_rool, error_sum;
	double thrust_add = 0;
	int16_t thrust_error;
	double thrust_function;
	uint8_t negflag = 0;
	error_pitch = MYDRON.Pitch.Angle_Error;
	error_rool 	= MYDRON.Rool.Angle_Error;

	error_pitch = WartoscBezwgledna(error_pitch);
	error_rool 	= WartoscBezwgledna(error_rool);


	error_sum = pow(error_pitch + error_rool + 1, 5);
	if(MYDRON.Thrust.Wanted > 10500){
		MYDRON.Thrust.Wanted = 10500;
	}

	thrust_error = MYDRON.Thrust.Wanted - MYDRON.Thrust.Now;
	if(thrust_error < 0){
		thrust_error = WartoscBezwgledna(thrust_error);
		negflag = 1;
	}


	if((thrust_error) != 0){
		//if(negflag == 0){
			thrust_function = 2000*(sqrt(sqrt((double)(thrust_error))));
			//MYDRON.TEAES = 0;
			thrust_add = ((double)(factor * MYDRON.Thrust.Increse_factor * thrust_function/error_sum));
		/*} else{*/
			if(negflag == 1){
//			MYDRON.TEAES += error_pitch + error_rool;
//			thrust_add = (thrust_error * (MYDRON.TEAES + 1)/((error_pitch + error_rool + 1)*10000)) * MYDRON.Thrust.Increse_factor;
			thrust_add *= -1;
		}


		if(thrust_add < 1 && thrust_add > -1){
			MYDRON.Thrust.Values = MYDRON.Thrust.Values + thrust_add;
		}
		else{
			MYDRON.Thrust.Now = MYDRON.Thrust.Now + thrust_add;
		}

		if(MYDRON.Thrust.Values >= 1 || MYDRON.Thrust.Values <= -1){
			MYDRON.Thrust.Now = MYDRON.Thrust.Now + MYDRON.Thrust.Values;
			MYDRON.Thrust.Values = 0;
		}
		if(MYDRON.Thrust.Now > 10000){//ograniczenie THRUST
			MYDRON.Thrust.Now = 10000;
		}
	}
}
/*
 * Thrust_filter
 *
 * niepozwala zwiększac jezeli część D jest zbyt durza;
 * dopuszczaly 1 o/s
 *
 *	d_fac = 2000 -> d = 4
 *
 *
 */
double Thrust_filter_2(float *PID_FAC_rool, float *PID_FAC_pitch, float *PID_FAC_yaw){
	double D_Factor_sum;
	double factor = 1;// zakres od 0 do 1;

	D_Factor_sum = WartoscBezwgledna(PID_FAC_rool[4])/* + WartoscBezwgledna(PID_FAC_pitch[4]) + WartoscBezwgledna(PID_FAC_yaw[4])*/;//todo odkomentowac

	factor = (10 / ((D_Factor_sum+1)));
	if(factor > 1){
		factor = 1;
	}

	return factor;
}

void Thrust_Angle_error_Filter(Dron *a){
	float factor, c, d;

	c = a->Rool.Wanted_rx == 0 ? 1 : a->Rool.Wanted_rx;
	d = a->Pitch.Wanted_rx == 0 ? 1 : a->Pitch.Wanted_rx;

	factor = WartoscBezwgledna((a->Rool.Wanted/(c/10))*(a->Pitch.Wanted/(d/10)));
	factor = factor > 1 ? 1: factor;

	a->Thrust.Wanted -= factor*(WartoscBezwgledna(a->Pitch.Angle_Error*10) + WartoscBezwgledna(a->Rool.Angle_Error*10));
}


/*
*	zmienia wartosc P w zaleznosci od thrust
 */
float APV(float P_factor, float factor, uint8_t mov){
	float new_factor = P_factor;
	if(factor < 1){
		factor = 1;
	}
	switch(mov){
		case 1:

			new_factor = P_factor - MYDRON.Thrust.Now/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;

		case 2:

			new_factor = P_factor - MYDRON.Thrust.Now/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;
	}
	return new_factor;
}

/*
*	zmienia wartosc D w zaleznosci od thrust
 */
float ADV(float D_factor, float factor, uint8_t mov){
	float new_factor = D_factor;
	if(factor < 1){
		factor = 1;
	}
	switch(mov){
		case 1:

			new_factor = D_factor - MYDRON.Thrust.Now/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;

		case 2:

			new_factor = D_factor - MYDRON.Thrust.Now/factor;
			if(new_factor < 1){
				new_factor = 1;
			}
			return new_factor;
	}
	return new_factor;
}

void acceleration_stabilizer(float *g_ax, float *PID_FAC){//todo
//
//	*g_ax = PID_FAC[0]*(wanted_pitch - *g_ax);
//
//	*g_ax = *g_ax + PID_FAC[1]*error_sum_pitch*looptime;
//
//	*g_ax = *g_ax + PID_FAC[2]*((wanted_pitch - *g_ax) - old_error_pitch);

}

void PID_Values_Save_Check(Dron *a){
	a->PID_Pitch.Angle_Factors[0] = a->PID_Pitch.Angle_Factors[0] < 0 ? 0 : a->PID_Pitch.Angle_Factors[0] > 20 ? 20 : a->PID_Pitch.Angle_Factors[0];
	a->PID_Pitch.Angle_Factors[1] = a->PID_Pitch.Angle_Factors[1] < 0 ? 0 : a->PID_Pitch.Angle_Factors[1] > 50 ? 50 : a->PID_Pitch.Angle_Factors[1];
	a->PID_Pitch.Angle_Factors[2] = a->PID_Pitch.Angle_Factors[2] < 0 ? 0 : a->PID_Pitch.Angle_Factors[2] > 1000 ? 1000 : a->PID_Pitch.Angle_Factors[2];

	a->PID_Rool.Angle_Factors[0] = a->PID_Rool.Angle_Factors[0] < 0 ? 0 : a->PID_Rool.Angle_Factors[0] > 20 ? 20 : a->PID_Rool.Angle_Factors[0];
	a->PID_Rool.Angle_Factors[1] = a->PID_Rool.Angle_Factors[1] < 0 ? 0 : a->PID_Rool.Angle_Factors[1] > 50 ? 50 : a->PID_Rool.Angle_Factors[1];
	a->PID_Rool.Angle_Factors[2] = a->PID_Rool.Angle_Factors[2] < 0 ? 0 : a->PID_Rool.Angle_Factors[2] > 1000 ? 1000 : a->PID_Rool.Angle_Factors[2];

	a->PID_Yaw.Angle_Factors[0] = a->PID_Yaw.Angle_Factors[0] < 0 ? 0 : a->PID_Yaw.Angle_Factors[0] > 20 ? 20 : a->PID_Yaw.Angle_Factors[0];
	a->PID_Yaw.Angle_Factors[1] = a->PID_Yaw.Angle_Factors[1] < 0 ? 0 : a->PID_Yaw.Angle_Factors[1] > 50 ? 50 : a->PID_Yaw.Angle_Factors[1];
	a->PID_Yaw.Angle_Factors[2] = a->PID_Yaw.Angle_Factors[2] < 0 ? 0 : a->PID_Yaw.Angle_Factors[2] > 1000 ? 1000 : a->PID_Yaw.Angle_Factors[2];



	a->PID_Pitch.Angular_Rate_Factors[0] = a->PID_Pitch.Angular_Rate_Factors[0] < 0 ? 0 : a->PID_Pitch.Angular_Rate_Factors[0] > 20 ? 20 : a->PID_Pitch.Angular_Rate_Factors[0];
	a->PID_Pitch.Angular_Rate_Factors[1] = a->PID_Pitch.Angular_Rate_Factors[1] < 0 ? 0 : a->PID_Pitch.Angular_Rate_Factors[1] > 20 ? 20 : a->PID_Pitch.Angular_Rate_Factors[1];
	a->PID_Pitch.Angular_Rate_Factors[2] = a->PID_Pitch.Angular_Rate_Factors[2] < 0 ? 0 : a->PID_Pitch.Angular_Rate_Factors[2] > 1000 ? 1000 : a->PID_Pitch.Angular_Rate_Factors[2];

	a->PID_Rool.Angular_Rate_Factors[0] = a->PID_Rool.Angular_Rate_Factors[0] < 0 ? 0 : a->PID_Rool.Angular_Rate_Factors[0] > 20 ? 20 : a->PID_Rool.Angular_Rate_Factors[0];
	a->PID_Rool.Angular_Rate_Factors[1] = a->PID_Rool.Angular_Rate_Factors[1] < 0 ? 0 : a->PID_Rool.Angular_Rate_Factors[1] > 20 ? 20 : a->PID_Rool.Angular_Rate_Factors[1];
	a->PID_Rool.Angular_Rate_Factors[2] = a->PID_Rool.Angular_Rate_Factors[2] < 0 ? 0 : a->PID_Rool.Angular_Rate_Factors[2] > 1000 ? 1000 : a->PID_Rool.Angular_Rate_Factors[2];

	a->PID_Yaw.Angular_Rate_Factors[0] = a->PID_Yaw.Angular_Rate_Factors[0] < 0 ? 0 : a->PID_Yaw.Angular_Rate_Factors[0] > 20 ? 20 : a->PID_Yaw.Angular_Rate_Factors[0];
	a->PID_Yaw.Angular_Rate_Factors[1] = a->PID_Yaw.Angular_Rate_Factors[1] < 0 ? 0 : a->PID_Yaw.Angular_Rate_Factors[1] > 20 ? 20 : a->PID_Yaw.Angular_Rate_Factors[1];
	a->PID_Yaw.Angular_Rate_Factors[2] = a->PID_Yaw.Angular_Rate_Factors[2] < 0 ? 0 : a->PID_Yaw.Angular_Rate_Factors[2] > 1000 ? 1000 : a->PID_Yaw.Angular_Rate_Factors[2];
}

void PID_call(Dron *Paramiters){
	//PID_Values_Save_Check(Paramiters);

	//wanted angular rate
#if DEBUG_USER == 1
	Paramiters->PID_Pitch.P_A = Paramiters->Pitch.Angle_Error * Paramiters->PID_Pitch.Angle_Factors[0];
	Paramiters->PID_Pitch.I_A = Paramiters->Pitch.Angle_Error_Sum * Paramiters->PID_Pitch.Angle_Factors[1] * looptime;
	Paramiters->PID_Pitch.D_A = (Paramiters->Pitch.Angle_Error - Paramiters->Pitch.Old_Angle_Error) * Paramiters->PID_Pitch.Angle_Factors[2];
	Paramiters->PID_Pitch.Angle_Value = Paramiters->PID_Pitch.P_A + Paramiters->PID_Pitch.I_A + Paramiters->PID_Pitch.D_A;

	Paramiters->PID_Rool.P_A = Paramiters->Rool.Angle_Error * Paramiters->PID_Rool.Angle_Factors[0];
	Paramiters->PID_Rool.I_A = Paramiters->Rool.Angle_Error_Sum * Paramiters->PID_Rool.Angle_Factors[1] * looptime;
	Paramiters->PID_Rool.D_A = (Paramiters->Rool.Angle_Error - Paramiters->Rool.Old_Angle_Error) * Paramiters->PID_Rool.Angle_Factors[2];
	Paramiters->PID_Rool.Angle_Value = Paramiters->PID_Rool.P_A + Paramiters->PID_Rool.I_A + Paramiters->PID_Rool.D_A;

	Paramiters->PID_Yaw.P_A = Paramiters->Yaw.Angle_Error * Paramiters->PID_Yaw.Angle_Factors[0];
	Paramiters->PID_Yaw.I_A = Paramiters->Yaw.Angle_Error_Sum * Paramiters->PID_Yaw.Angle_Factors[1] * looptime;
	Paramiters->PID_Yaw.D_A = (Paramiters->Yaw.Angle_Error - Paramiters->Yaw.Old_Angle_Error) * Paramiters->PID_Yaw.Angle_Factors[2];
	Paramiters->PID_Yaw.Angle_Value = Paramiters->PID_Yaw.P_A + Paramiters->PID_Yaw.I_A + Paramiters->PID_Yaw.D_A;
#else
	Paramiters->PID_Pitch.Angle_Value = (Paramiters->Pitch.Angle_Error * Paramiters->PID_Pitch.Angle_Factors[0]) + (Paramiters->Pitch.Angle_Error_Sum * Paramiters->PID_Pitch.Angle_Factors[1] * looptime) + ((Paramiters->Pitch.Angle_Error - Paramiters->Pitch.Old_Angle_Error) * Paramiters->PID_Pitch.Angle_Factors[2]);
	Paramiters->PID_Rool.Angle_Value = (Paramiters->Rool.Angle_Error * Paramiters->PID_Rool.Angle_Factors[0]) + (Paramiters->Rool.Angle_Error_Sum * Paramiters->PID_Rool.Angle_Factors[1] * looptime) + ((Paramiters->Rool.Angle_Error - Paramiters->Rool.Old_Angle_Error) * Paramiters->PID_Rool.Angle_Factors[2]);
	Paramiters->PID_Yaw.Angle_Value = (Paramiters->Yaw.Angle_Error * Paramiters->PID_Yaw.Angle_Factors[0]) + (Paramiters->Yaw.Angle_Error_Sum * Paramiters->PID_Yaw.Angle_Factors[1] * looptime) + ((Paramiters->Yaw.Angle_Error - Paramiters->Yaw.Old_Angle_Error) * Paramiters->PID_Yaw.Angle_Factors[2]);
#endif


  	MYDRON.Pitch.Angular_Rate_Error = MYDRON.PID_Pitch.Angle_Value - MPU6050.Gyr.gx;
  	MYDRON.Rool.Angular_Rate_Error = MYDRON.PID_Rool.Angle_Value - MPU6050.Gyr.gy;
  	MYDRON.Yaw.Angular_Rate_Error = MYDRON.PID_Yaw.Angle_Value - MPU6050.Gyr.gz;


#if DEBUG_USER == 1
	Paramiters->PID_Pitch.P_AR = Paramiters->Pitch.Angular_Rate_Error * Paramiters->PID_Pitch.Angular_Rate_Factors[0];
	Paramiters->PID_Pitch.I_AR = Paramiters->Pitch.Angular_Rate_Error_Sum * Paramiters->PID_Pitch.Angular_Rate_Factors[1] * looptime;
	Paramiters->PID_Pitch.D_AR = (Paramiters->Pitch.Angular_Rate_Error - Paramiters->Pitch.Old_Angular_Rate_Error) * Paramiters->PID_Pitch.Angular_Rate_Factors[2];
	Paramiters->PID_Pitch.Angular_Rate_Value = Paramiters->PID_Pitch.P_AR + Paramiters->PID_Pitch.I_AR + Paramiters->PID_Pitch.D_AR;

	Paramiters->PID_Rool.P_AR = Paramiters->Rool.Angular_Rate_Error * Paramiters->PID_Rool.Angular_Rate_Factors[0];
	Paramiters->PID_Rool.I_AR = Paramiters->Rool.Angular_Rate_Error_Sum * Paramiters->PID_Rool.Angular_Rate_Factors[1] * looptime;
	Paramiters->PID_Rool.D_AR = (Paramiters->Rool.Angular_Rate_Error - Paramiters->Rool.Old_Angular_Rate_Error) * Paramiters->PID_Rool.Angular_Rate_Factors[2];
	Paramiters->PID_Rool.Angular_Rate_Value = Paramiters->PID_Rool.P_AR + Paramiters->PID_Rool.I_AR + Paramiters->PID_Rool.D_AR;

	Paramiters->PID_Yaw.P_AR = Paramiters->Yaw.Angular_Rate_Error * Paramiters->PID_Yaw.Angular_Rate_Factors[0];
	Paramiters->PID_Yaw.I_AR = Paramiters->Yaw.Angular_Rate_Error_Sum * Paramiters->PID_Yaw.Angular_Rate_Factors[1] * looptime;
	Paramiters->PID_Yaw.D_AR = (Paramiters->Yaw.Angular_Rate_Error - Paramiters->Yaw.Old_Angular_Rate_Error) * Paramiters->PID_Yaw.Angular_Rate_Factors[2];
	Paramiters->PID_Yaw.Angular_Rate_Value = Paramiters->PID_Yaw.P_AR + Paramiters->PID_Yaw.I_AR + Paramiters->PID_Yaw.D_AR;
#else
	Paramiters->PID_Pitch.Angular_Rate_Value = (Paramiters->Pitch.Angular_Rate_Error * Paramiters->PID_Pitch.Angular_Rate_Factors[0]) + (Paramiters->Pitch.Angular_Rate_Error_Sum * Paramiters->PID_Pitch.Angular_Rate_Factors[1] * looptime) + ((Paramiters->Pitch.Angular_Rate_Error - Paramiters->Pitch.Old_Angular_Rate_Error) * Paramiters->PID_Pitch.Angular_Rate_Factors[2]);
	Paramiters->PID_Rool.Angular_Rate_Value = (Paramiters->Rool.Angular_Rate_Error * Paramiters->PID_Rool.Angular_Rate_Factors[0]) + (Paramiters->Rool.Angular_Rate_Error_Sum * Paramiters->PID_Rool.Angular_Rate_Factors[1] * looptime) + ((Paramiters->Rool.Angular_Rate_Error - Paramiters->Rool.Old_Angular_Rate_Error) * Paramiters->PID_Rool.Angular_Rate_Factors[2]);
	Paramiters->PID_Yaw.Angular_Rate_Value = (Paramiters->Yaw.Angular_Rate_Error * Paramiters->PID_Yaw.Angular_Rate_Factors[0]) + (Paramiters->Yaw.Angular_Rate_Error_Sum * Paramiters->PID_Yaw.Angular_Rate_Factors[1] * looptime) + ((Paramiters->Yaw.Angular_Rate_Error - Paramiters->Yaw.Old_Angular_Rate_Error) * Paramiters->PID_Yaw.Angular_Rate_Factors[2]);
#endif
}

void PID_call_2(Dron *Paramiters){
	PID_Values_Save_Check(Paramiters);
	float Temp_PID_Pitch[3], Temp_PID_Rool[3], Temp_PID_Yaw[3], Temp_PID_AR_Pitch[3], Temp_PID_AR_Rool[3], Temp_PID_AR_Yaw[3];

	//TOM - Takeoff Mode
	if(Paramiters->TOM.Status == 1){
		Paramiters->TOM.Pitch.Angle_Factors[3] = Paramiters->TOM.Pitch.Angle_Factors[0];
		Paramiters->TOM.Pitch.Angle_Factors[4] = Paramiters->TOM.Pitch.Angle_Factors[1];
		Paramiters->TOM.Pitch.Angle_Factors[5] = Paramiters->TOM.Pitch.Angle_Factors[2];

		Paramiters->TOM.Rool.Angle_Factors[3] = Paramiters->TOM.Rool.Angle_Factors[0];
		Paramiters->TOM.Rool.Angle_Factors[4] = Paramiters->TOM.Rool.Angle_Factors[1];
		Paramiters->TOM.Rool.Angle_Factors[5] = Paramiters->TOM.Rool.Angle_Factors[2];

		Paramiters->TOM.Yaw.Angle_Factors[3] = Paramiters->TOM.Yaw.Angle_Factors[0];
		Paramiters->TOM.Yaw.Angle_Factors[4] = Paramiters->TOM.Yaw.Angle_Factors[1];
		Paramiters->TOM.Yaw.Angle_Factors[5] = Paramiters->TOM.Yaw.Angle_Factors[2];

		Paramiters->TOM.Pitch.Angular_Rate_Factors[3] = Paramiters->TOM.Pitch.Angular_Rate_Factors[0];
		Paramiters->TOM.Pitch.Angular_Rate_Factors[4] = Paramiters->TOM.Pitch.Angular_Rate_Factors[1];
		Paramiters->TOM.Pitch.Angular_Rate_Factors[5] = Paramiters->TOM.Pitch.Angular_Rate_Factors[2];

		Paramiters->TOM.Rool.Angular_Rate_Factors[3] = Paramiters->TOM.Rool.Angular_Rate_Factors[0];
		Paramiters->TOM.Rool.Angular_Rate_Factors[4] = Paramiters->TOM.Rool.Angular_Rate_Factors[1];
		Paramiters->TOM.Rool.Angular_Rate_Factors[5] = Paramiters->TOM.Rool.Angular_Rate_Factors[2];

		Paramiters->TOM.Yaw.Angular_Rate_Factors[3] = Paramiters->TOM.Yaw.Angular_Rate_Factors[0];
		Paramiters->TOM.Yaw.Angular_Rate_Factors[4] = Paramiters->TOM.Yaw.Angular_Rate_Factors[1];
		Paramiters->TOM.Yaw.Angular_Rate_Factors[5] = Paramiters->TOM.Yaw.Angular_Rate_Factors[2];
	}
	if(Paramiters->TOM.Status == 0){
		Temp_PID_Pitch[0] = Paramiters->PID_Pitch.Angle_Factors[0];
		Temp_PID_Pitch[1] = Paramiters->PID_Pitch.Angle_Factors[1];
		Temp_PID_Pitch[2] = Paramiters->PID_Pitch.Angle_Factors[2];

		Temp_PID_Rool[0] = Paramiters->PID_Rool.Angle_Factors[0];
		Temp_PID_Rool[1] = Paramiters->PID_Rool.Angle_Factors[1];
		Temp_PID_Rool[2] = Paramiters->PID_Rool.Angle_Factors[2];

		Temp_PID_Yaw[0] = Paramiters->PID_Yaw.Angle_Factors[0];
		Temp_PID_Yaw[1] = Paramiters->PID_Yaw.Angle_Factors[1];
		Temp_PID_Yaw[2] = Paramiters->PID_Yaw.Angle_Factors[2];

		Temp_PID_AR_Pitch[0] = Paramiters->PID_Pitch.Angular_Rate_Factors[0];
		Temp_PID_AR_Pitch[1] = Paramiters->PID_Pitch.Angular_Rate_Factors[1];
		Temp_PID_AR_Pitch[2] = Paramiters->PID_Pitch.Angular_Rate_Factors[2];

		Temp_PID_AR_Rool[0] = Paramiters->PID_Rool.Angular_Rate_Factors[0];
		Temp_PID_AR_Rool[1] = Paramiters->PID_Rool.Angular_Rate_Factors[1];
		Temp_PID_AR_Rool[2] = Paramiters->PID_Rool.Angular_Rate_Factors[2];

		Temp_PID_AR_Yaw[0] = Paramiters->PID_Yaw.Angular_Rate_Factors[0];
		Temp_PID_AR_Yaw[1] = Paramiters->PID_Yaw.Angular_Rate_Factors[1];
		Temp_PID_AR_Yaw[2] = Paramiters->PID_Yaw.Angular_Rate_Factors[2];

		if(Paramiters->TOM.Pitch.Angle_Factors[3] != Paramiters->PID_Pitch.Angle_Factors[0]){
			Temp_PID_Pitch[0] = (Temp_PID_Pitch[0] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Pitch.Angle_Factors[3] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Pitch.Angle_Factors[3] = Temp_PID_Pitch[0];
		}
		if(Paramiters->TOM.Pitch.Angle_Factors[4] != Paramiters->PID_Pitch.Angle_Factors[1]){
			Temp_PID_Pitch[1] = (Temp_PID_Pitch[1] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Pitch.Angle_Factors[4] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Pitch.Angle_Factors[4] = Temp_PID_Pitch[1];
		}
		if(Paramiters->TOM.Pitch.Angle_Factors[5] != Paramiters->PID_Pitch.Angle_Factors[2]){
			Temp_PID_Pitch[2] = (Temp_PID_Pitch[2] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Pitch.Angle_Factors[5] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Pitch.Angle_Factors[5] = Temp_PID_Pitch[2];
		}

		if(Paramiters->TOM.Rool.Angle_Factors[3] != Paramiters->PID_Rool.Angle_Factors[0]){
			Temp_PID_Rool[0] = (Temp_PID_Rool[0] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Rool.Angle_Factors[3] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Rool.Angle_Factors[3] = Temp_PID_Rool[0];
		}
		if(Paramiters->TOM.Rool.Angle_Factors[4] != Paramiters->PID_Rool.Angle_Factors[1]){
			Temp_PID_Rool[1] = (Temp_PID_Rool[1] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Rool.Angle_Factors[4] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Rool.Angle_Factors[4] = Temp_PID_Rool[1];
		}
		if(Paramiters->TOM.Rool.Angle_Factors[5] != Paramiters->PID_Rool.Angle_Factors[2]){
			Temp_PID_Rool[2] = (Temp_PID_Rool[2] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Rool.Angle_Factors[5] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Rool.Angle_Factors[5] = Temp_PID_Rool[2];
		}

		if(Paramiters->TOM.Yaw.Angle_Factors[3] != Paramiters->PID_Yaw.Angle_Factors[0]){
			Temp_PID_Yaw[0] = (Temp_PID_Yaw[0] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Yaw.Angle_Factors[3] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Yaw.Angle_Factors[3] = Temp_PID_Yaw[0];
		}
		if(Paramiters->TOM.Yaw.Angle_Factors[4] != Paramiters->PID_Yaw.Angle_Factors[1]){
			Temp_PID_Yaw[1] = (Temp_PID_Yaw[1] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Yaw.Angle_Factors[4] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Yaw.Angle_Factors[4] = Temp_PID_Yaw[1];
		}
		if(Paramiters->TOM.Yaw.Angle_Factors[5] != Paramiters->PID_Yaw.Angle_Factors[2]){
			Temp_PID_Yaw[2] = (Temp_PID_Yaw[2] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Yaw.Angle_Factors[5] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Yaw.Angle_Factors[5] = Temp_PID_Yaw[2];
		}

		if(Paramiters->TOM.Pitch.Angular_Rate_Factors[3] != Paramiters->PID_Pitch.Angular_Rate_Factors[0]){
			Temp_PID_AR_Pitch[0] = (Temp_PID_AR_Pitch[0] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Pitch.Angular_Rate_Factors[3] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Pitch.Angular_Rate_Factors[3] = Temp_PID_AR_Pitch[0];
		}
		if(Paramiters->TOM.Pitch.Angular_Rate_Factors[4] != Paramiters->PID_Pitch.Angular_Rate_Factors[1]){
			Temp_PID_AR_Pitch[1] = (Temp_PID_AR_Pitch[1] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Pitch.Angular_Rate_Factors[4] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Pitch.Angular_Rate_Factors[4] = Temp_PID_AR_Pitch[1];
		}
		if(Paramiters->TOM.Pitch.Angular_Rate_Factors[5] != Paramiters->PID_Pitch.Angular_Rate_Factors[2]){
			Temp_PID_AR_Pitch[2] = (Temp_PID_AR_Pitch[2] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Pitch.Angular_Rate_Factors[5] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Pitch.Angular_Rate_Factors[5] = Temp_PID_AR_Pitch[2];
		}

		if(Paramiters->TOM.Rool.Angular_Rate_Factors[3] != Paramiters->PID_Rool.Angular_Rate_Factors[0]){
			Temp_PID_AR_Rool[0] = (Temp_PID_AR_Rool[0] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Rool.Angular_Rate_Factors[3] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Rool.Angular_Rate_Factors[3] = Temp_PID_AR_Rool[0];
		}
		if(Paramiters->TOM.Rool.Angular_Rate_Factors[4] != Paramiters->PID_Rool.Angular_Rate_Factors[1]){
			Temp_PID_AR_Rool[1] = (Temp_PID_AR_Rool[1] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Rool.Angular_Rate_Factors[4] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Rool.Angular_Rate_Factors[4] = Temp_PID_AR_Rool[1];
		}
		if(Paramiters->TOM.Rool.Angular_Rate_Factors[5] != Paramiters->PID_Rool.Angular_Rate_Factors[2]){
			Temp_PID_AR_Rool[2] = (Temp_PID_AR_Rool[2] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Rool.Angular_Rate_Factors[5] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Rool.Angular_Rate_Factors[5] = Temp_PID_AR_Rool[2];
		}

		if(Paramiters->TOM.Yaw.Angular_Rate_Factors[3] != Paramiters->PID_Yaw.Angular_Rate_Factors[0]){
			Temp_PID_AR_Yaw[0] = (Temp_PID_AR_Yaw[0] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Yaw.Angular_Rate_Factors[3] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Yaw.Angular_Rate_Factors[3] = Temp_PID_AR_Yaw[0];
		}
		if(Paramiters->TOM.Yaw.Angular_Rate_Factors[4] != Paramiters->PID_Yaw.Angular_Rate_Factors[1]){
			Temp_PID_AR_Yaw[1] = (Temp_PID_AR_Yaw[1] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Yaw.Angular_Rate_Factors[4] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Yaw.Angular_Rate_Factors[4] = Temp_PID_AR_Yaw[1];
		}
		if(Paramiters->TOM.Yaw.Angular_Rate_Factors[5] != Paramiters->PID_Yaw.Angular_Rate_Factors[2]){
			Temp_PID_AR_Yaw[2] = (Temp_PID_AR_Yaw[2] * (0.5 * looptime) / (1 + (0.5 * looptime))) + (Paramiters->TOM.Yaw.Angular_Rate_Factors[5] * (1 / (1 + (0.5 * looptime))));
			Paramiters->TOM.Yaw.Angular_Rate_Factors[5] = Temp_PID_AR_Yaw[2];
		}
	}

	//wanted angular rate
#if DEBUG_USER == 1
	Paramiters->PID_Pitch.P_A = Paramiters->Pitch.Angle_Error * Paramiters->PID_Pitch.Angle_Factors[0];
	Paramiters->PID_Pitch.I_A = Paramiters->Pitch.Angle_Error_Sum * Paramiters->PID_Pitch.Angle_Factors[1] * looptime;
	Paramiters->PID_Pitch.D_A = (Paramiters->Pitch.Angle_Error - Paramiters->Pitch.Old_Angle_Error) * Paramiters->PID_Pitch.Angle_Factors[2];
	Paramiters->PID_Pitch.Angle_Value = Paramiters->PID_Pitch.P_A + Paramiters->PID_Pitch.I_A + Paramiters->PID_Pitch.D_A;

	Paramiters->PID_Rool.P_A = Paramiters->Rool.Angle_Error * Paramiters->PID_Rool.Angle_Factors[0];
	Paramiters->PID_Rool.I_A = Paramiters->Rool.Angle_Error_Sum * Paramiters->PID_Rool.Angle_Factors[1] * looptime;
	Paramiters->PID_Rool.D_A = (Paramiters->Rool.Angle_Error - Paramiters->Rool.Old_Angle_Error) * Paramiters->PID_Rool.Angle_Factors[2];
	Paramiters->PID_Rool.Angle_Value = Paramiters->PID_Rool.P_A + Paramiters->PID_Rool.I_A + Paramiters->PID_Rool.D_A;

	Paramiters->PID_Yaw.P_A = Paramiters->Yaw.Angle_Error * Paramiters->PID_Yaw.Angle_Factors[0];
	Paramiters->PID_Yaw.I_A = Paramiters->Yaw.Angle_Error_Sum * Paramiters->PID_Yaw.Angle_Factors[1] * looptime;
	Paramiters->PID_Yaw.D_A = (Paramiters->Yaw.Angle_Error - Paramiters->Yaw.Old_Angle_Error) * Paramiters->PID_Yaw.Angle_Factors[2];
	Paramiters->PID_Yaw.Angle_Value = Paramiters->PID_Yaw.P_A + Paramiters->PID_Yaw.I_A + Paramiters->PID_Yaw.D_A;
#else
	Paramiters->PID_Pitch.Angle_Value = (Paramiters->Pitch.Angle_Error * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Pitch.Angle_Factors[0] : Temp_PID_Pitch[0])) + (Paramiters->Pitch.Angle_Error_Sum * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Pitch.Angle_Factors[1] : Temp_PID_Pitch[1]) * looptime) + ((Paramiters->Pitch.Angle_Error - Paramiters->Pitch.Old_Angle_Error) * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Pitch.Angle_Factors[2] : Temp_PID_Pitch[2]));
	Paramiters->PID_Rool.Angle_Value = (Paramiters->Rool.Angle_Error * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Rool.Angle_Factors[0] : Temp_PID_Rool[0])) + (Paramiters->Rool.Angle_Error_Sum * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Rool.Angle_Factors[1] : Temp_PID_Rool[1]) * looptime) + ((Paramiters->Rool.Angle_Error - Paramiters->Rool.Old_Angle_Error) * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Rool.Angle_Factors[2] : Temp_PID_Rool[2]));
	Paramiters->PID_Yaw.Angle_Value = (Paramiters->Yaw.Angle_Error * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Yaw.Angle_Factors[0] : Temp_PID_Yaw[0])) + (Paramiters->Yaw.Angle_Error_Sum * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Yaw.Angle_Factors[1] : Temp_PID_Yaw[1]) * looptime) + ((Paramiters->Yaw.Angle_Error - Paramiters->Yaw.Old_Angle_Error) * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Yaw.Angle_Factors[2] : Temp_PID_Yaw[2]));
#endif


  	MYDRON.Pitch.Angular_Rate_Error = MYDRON.PID_Pitch.Angle_Value - MPU6050.Gyr.gx;
  	MYDRON.Rool.Angular_Rate_Error = MYDRON.PID_Rool.Angle_Value - MPU6050.Gyr.gy;
  	MYDRON.Yaw.Angular_Rate_Error = MYDRON.PID_Yaw.Angle_Value - MPU6050.Gyr.gz;


#if DEBUG_USER == 1
	Paramiters->PID_Pitch.P_AR = Paramiters->Pitch.Angular_Rate_Error * Paramiters->PID_Pitch.Angular_Rate_Factors[0];
	Paramiters->PID_Pitch.I_AR = Paramiters->Pitch.Angular_Rate_Error_Sum * Paramiters->PID_Pitch.Angular_Rate_Factors[1] * looptime;
	Paramiters->PID_Pitch.D_AR = (Paramiters->Pitch.Angular_Rate_Error - Paramiters->Pitch.Old_Angular_Rate_Error) * Paramiters->PID_Pitch.Angular_Rate_Factors[2];
	Paramiters->PID_Pitch.Angular_Rate_Value = Paramiters->PID_Pitch.P_AR + Paramiters->PID_Pitch.I_AR + Paramiters->PID_Pitch.D_AR;

	Paramiters->PID_Rool.P_AR = Paramiters->Rool.Angular_Rate_Error * Paramiters->PID_Rool.Angular_Rate_Factors[0];
	Paramiters->PID_Rool.I_AR = Paramiters->Rool.Angular_Rate_Error_Sum * Paramiters->PID_Rool.Angular_Rate_Factors[1] * looptime;
	Paramiters->PID_Rool.D_AR = (Paramiters->Rool.Angular_Rate_Error - Paramiters->Rool.Old_Angular_Rate_Error) * Paramiters->PID_Rool.Angular_Rate_Factors[2];
	Paramiters->PID_Rool.Angular_Rate_Value = Paramiters->PID_Rool.P_AR + Paramiters->PID_Rool.I_AR + Paramiters->PID_Rool.D_AR;

	Paramiters->PID_Yaw.P_AR = Paramiters->Yaw.Angular_Rate_Error * Paramiters->PID_Yaw.Angular_Rate_Factors[0];
	Paramiters->PID_Yaw.I_AR = Paramiters->Yaw.Angular_Rate_Error_Sum * Paramiters->PID_Yaw.Angular_Rate_Factors[1] * looptime;
	Paramiters->PID_Yaw.D_AR = (Paramiters->Yaw.Angular_Rate_Error - Paramiters->Yaw.Old_Angular_Rate_Error) * Paramiters->PID_Yaw.Angular_Rate_Factors[2];
	Paramiters->PID_Yaw.Angular_Rate_Value = Paramiters->PID_Yaw.P_AR + Paramiters->PID_Yaw.I_AR + Paramiters->PID_Yaw.D_AR;
#else
	Paramiters->PID_Pitch.Angular_Rate_Value = (Paramiters->Pitch.Angular_Rate_Error * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Pitch.Angular_Rate_Factors[0] : Temp_PID_AR_Pitch[0])) + (Paramiters->Pitch.Angular_Rate_Error_Sum * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Pitch.Angular_Rate_Factors[1] : Temp_PID_AR_Pitch[1]) * looptime) + ((Paramiters->Pitch.Angular_Rate_Error - Paramiters->Pitch.Old_Angular_Rate_Error) * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Pitch.Angular_Rate_Factors[2] : Temp_PID_AR_Pitch[2]));
	Paramiters->PID_Rool.Angular_Rate_Value = (Paramiters->Rool.Angular_Rate_Error * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Rool.Angular_Rate_Factors[0] : Temp_PID_AR_Rool[0])) + (Paramiters->Rool.Angular_Rate_Error_Sum * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Rool.Angular_Rate_Factors[1] : Temp_PID_AR_Rool[1]) * looptime) + ((Paramiters->Rool.Angular_Rate_Error - Paramiters->Rool.Old_Angular_Rate_Error) * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Rool.Angular_Rate_Factors[2] : Temp_PID_AR_Rool[2]));
	Paramiters->PID_Yaw.Angular_Rate_Value = (Paramiters->Yaw.Angular_Rate_Error * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Yaw.Angular_Rate_Factors[0] : Temp_PID_AR_Yaw[0])) + (Paramiters->Yaw.Angular_Rate_Error_Sum * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Yaw.Angular_Rate_Factors[1] : Temp_PID_Yaw[1]) * looptime) + ((Paramiters->Yaw.Angular_Rate_Error - Paramiters->Yaw.Old_Angular_Rate_Error) * (Paramiters->TOM.Status == 1 ? Paramiters->TOM.Yaw.Angular_Rate_Factors[2] : Temp_PID_AR_Yaw[2]));
#endif
}

void PID_cal(float *PID_var, float *PID_FAC, uint8_t pry){//pitch = 1, rool = 2, yaw = 3
//	switch(pry){
//		case 1://pitch
//
//			*PID_var = PID_FAC[0]*(pitch_error);
//
//			*PID_var = *PID_var + PID_FAC[1]*error_sum_pitch*looptime;
//
//			PID_FAC[3] = PID_FAC[2]*((pitch_error) - old_error_pitch);//policzenie częsci D
//
//			//FDP
//			if(FDP_D_Gain > 0){
//				PID_FAC[3] = (PID_FAC[3] * (FDP_D_Gain * looptime) / (1 + (FDP_D_Gain * looptime))) + (PID_FAC[4] * (1 / (1 + (FDP_D_Gain * looptime))));
//				PID_FAC[4] = PID_FAC[3];//old d_fac
//			}
//
//			*PID_var = *PID_var + PID_FAC[3];
//
////				if(*PID_var > 400){//PID_var jest w o/s, jezeli bendzei chcailo sie obracac szybciej niz.. przekroczy zakres pomiarowy akcelerometru
////					*PID_var = 400;
////				}
////				if(*PID_var < -400){
////					*PID_var = -400;
////				}
//			break;
//
//		case 2://rool
//
//			*PID_var = PID_FAC[0]*(rool_error);
//
//			*PID_var = *PID_var + PID_FAC[1]*error_sum_rool*looptime;
//
//			PID_FAC[3] =  PID_FAC[2]*((rool_error) - old_error_rool);
//
//			//FDP
//			if(FDP_D_Gain > 0){
//				PID_FAC[3] = (PID_FAC[3] * (FDP_D_Gain * looptime) / (1 + (FDP_D_Gain * looptime))) + (PID_FAC[4] * (1 / (1 + (FDP_D_Gain * looptime))));
//				PID_FAC[4] = PID_FAC[3];//old d_fac
//			}
//
//			*PID_var = *PID_var + PID_FAC[3];
//
////			if(*PID_var > 400){
////				*PID_var = 400;
////			}
////			if(*PID_var < -400){
////				*PID_var = -400;
////			}
//			break;
//
//		case 3:
//
//			*PID_var = PID_FAC[0]*(yaw_error);
//
//			*PID_var = *PID_var + PID_FAC[1]*error_sum_yaw*looptime;
//
//			PID_FAC[3] = PID_FAC[2]*((yaw_error) - old_error_yaw);
//
//			//FDP
//			if(FDP_D_Gain > 0){
//				PID_FAC[3] = (PID_FAC[3] * (FDP_D_Gain * looptime) / (1 + (FDP_D_Gain * looptime))) + (PID_FAC[4] * (1 / (1 + (FDP_D_Gain * looptime))));
//				PID_FAC[4] = PID_FAC[3];//old d_fac
//			}
//
//			*PID_var = *PID_var + PID_FAC[3];
//
////			if(*PID_var > 400){
////				*PID_var = 400;
////			}
////			if(*PID_var < -400){
////				*PID_var = -400;
////			}
//			break;
//
//		case 4:// angular rates pitch
//
//			*PID_var = PID_FAC[0]*(pitch_ar_error);
//
//			*PID_var = *PID_var + PID_FAC[1]*error_sum_angular_rate_pitch*looptime;
//
//
//			PID_FAC[3] = PID_FAC[2]*((pitch_ar_error) - old_error_angular_rate_pitch);//policzenie częsci D
//
//			//FDP
//			if(FDP_D_Gain_AR > 0){
//				PID_FAC[3] = (PID_FAC[3] * (FDP_D_Gain_AR * looptime) / (1 + (FDP_D_Gain_AR * looptime))) + (PID_FAC[4] * (1 / (1 + (FDP_D_Gain_AR * looptime))));
//				PID_FAC[4] = PID_FAC[3];//old d_fac
//			}
//
//			*PID_var = *PID_var + PID_FAC[3];
//
//			break;
//
//		case 5:// angular rates rool
//
//			*PID_var = PID_FAC[0]*(rool_ar_error);
//
//			*PID_var = *PID_var + PID_FAC[1]*error_sum_angular_rate_rool*looptime;
//
//			PID_FAC[3] =  PID_FAC[2]*((rool_ar_error) - old_error_angular_rate_rool);
//
//			//FDP
//			if(FDP_D_Gain_AR > 0){
//				PID_FAC[3] = (PID_FAC[3] * (FDP_D_Gain_AR * looptime) / (1 + (FDP_D_Gain_AR * looptime))) + (PID_FAC[4] * (1 / (1 + (FDP_D_Gain_AR * looptime))));
//				PID_FAC[4] = PID_FAC[3];//old d_fac
//			}
//
//			*PID_var = *PID_var + PID_FAC[3];
//			break;
//
//		case 6:// angular rates yaw
//
//			*PID_var = PID_FAC[0]*(yaw_ar_error);
//
//			*PID_var = *PID_var + PID_FAC[1]*error_sum_angular_rate_yaw*looptime;
//
//			PID_FAC[3] = PID_FAC[2]*((yaw_ar_error) - old_error_angular_rate_yaw);
//
//			//FDP
//			if(FDP_D_Gain_AR > 0){
//				PID_FAC[3] = (PID_FAC[3] * (FDP_D_Gain_AR * looptime) / (1 + (FDP_D_Gain_AR * looptime))) + (PID_FAC[4] * (1 / (1 + (FDP_D_Gain_AR * looptime))));
//				PID_FAC[4] = PID_FAC[3];//old d_fac
//			}
//
//			*PID_var = *PID_var + PID_FAC[3];
//			break;
//		default:
//			break;
	//}
}

void Get_batteryvalue(void){
	/*
	 * Kiedy 9,5V -> DRON_BATTERY_RUN_OUT 9,6V kryyczne napięcie bateri
	 * dzielnik to 3,7
	 * Max napiencie to 11,3V to po dzielniku 3,05v czyli 3786
	 * 8V po dzielniku to 2,16V czyli 2681
	 * nie 8V
	 * 10V musi byc czyli 2,7027V
	 *	2,7027V to 3353
	 *	3400
	*/

	MYDRON.batterysize = (analogmess - 3353)/7.42;
	if(MYDRON.batterysize >= 100){
		MYDRON.batterysize = 100;
	}

	if(MYDRON.batterysize < 25){
		MYDRON.Status.Battery = DRON_BATTERY_RUN_OUT;
	}
	if(MYDRON.batterysize <= 10){
		MYDRON.Status.Battery = DRON_BATTERY_CRIT_VAL;
	}
	if(MYDRON.batterysize >= 25){
		MYDRON.Status.Battery = DRON_BATTERY_OK;
	}
}

void IS_DRON_ON_GROUND(Dron *a, HC_SR04_Struct c){//todo
	a->Status.On_Grund = ((c.Distance - c.Start_Distance) >= 0.01) ? 0 : ((c.Distance - c.Start_Distance) <= 0.01) ? 1 : 0;
	a->TOM.Status = a->Status.On_Grund;
}

void PID_Self_Regulation(uint8_t freedom, float *PID_FAC){//todo
	uint8_t best = 0;
	float overshoot[2];//[0] - teraz [1] poprzedni
	overshoot[0] = 0;
	overshoot[1] = 0;
	 switch(freedom){
	 	 case 1://pitch angular
	 		 for(int i = 0; i < 5; i++){
	 			 PID_FAC[i] = 0;
	 		 }
	 		 while(best != 1){
	 			 PID_FAC[0] = 1;//ustawienie wspulczynika P

	 			 if(overshoot[0] < overshoot[1]){

	 			 }
	 		 }
	 		 break;
	 	 case 2://pitch
	 		 break;
	 	 case 3://rool angular
	 		 break;
	 	 case 4://rool
	 		 break;
	 	 case 5://yaw angular
	 		 break;
	 	 case 6://yaw
	 		 break;
	 }
}

void Angle_Filter(Dron *a, Madgwick_Struct c){
	float temp_yaw;
	a->Pitch.Now = 0.8*(a->Pitch.Now + MPU6050.Gyr.gx*looptime) + 0.2*c.x;
	a->Rool.Now = 0.8*(a->Rool.Now + MPU6050.Gyr.gy*looptime) + 0.2*c.y;


	if(c.z * a->Yaw.Old < 0){
		if(c.z < -170){
			a->Yaw.Count += 2;
		}
		if(c.z > 170){
			a->Yaw.Count -= 2;
		}

//		if(c.z < 10){
//
//		}
//		if(c.z > - 10){
//
//		}
	}
	temp_yaw = c.z + 180*a->Yaw.Count;
	a->Yaw.Now = 0.8*(a->Yaw.Now + MPU6050.Gyr.gz*looptime) + 0.2*temp_yaw;
	a->Yaw.Old = c.z;
}

/*float Calculate_altitude(float ax, float ay, float az, float last_altitude){
	float G_Force = 0;
	float altitude = 0;

	G_Force = (ax + ay + az) - 255;

	altitude =+ last_altitude;

	return altitude;
}*/

/*void Position_Holding(void){
	float G_Force = 0;
	float X_acc = 0, Y_acc = 0, Z_acc = 0;

	//G_Force = (ax + ay + az) - 255;//suma przyspieszeń - przyspieszenie ziemskie(255)\

	X_acc = ax;
	Y_acc = ay;
	Z_acc = G_Force - X_acc - Y_acc;
}*/


void convert_array_to_value(uint8_t arrayfrom[], int16_t *value , uint8_t rangebegin, uint8_t rangeend){
	*value = 0;
	int range = rangeend - rangebegin;

	for(int y = 0; y < range+1; y++){
		*value = *value + arrayfrom[rangebegin+y]*pow(10, range - y);
	}
}


void convert_value_to_array(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend){
	int x = 0;
	int loopnum = 0;
	int range = rangeend - rangebegin;
	for(int i = 0; i < range+1; i++){// 3
		while(value >= (uint16_t)pow(10,range - i)){
			if(value == 0){
				break;
			}
			value -= (uint16_t)potenga(10,range - i);
			x++;
		}
		arraytoputin[rangebegin+loopnum] = (uint8_t)x;
		loopnum++;
		x = 0;
	}
}
uint32_t potenga(int a, int b){
	int32_t c = a;
	if(b == 0){
		return 1;
	}
	if(b == 1){
		return a;
	}
	if(b > 1){
			for(int i = 1; i < b; i++){
			a = a*c;
		}
		return a;
	}
	if(b < 0){
		for(int i = 0; i < b; i++){
			a = a/c;
		}
		return a;
	}
	return a;
}
float WartoscBezwgledna(float a){
	a = (a < 0) ? a*(-1) : a;
	return a;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void RGB_LED_Set_color(uint8_t R, uint8_t G, uint8_t B){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, B);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, R);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, G);
}

void RGB_LED_For_BAT(uint8_t batval){
/*
 * batval == 150  r = 0, g = 255 b = 0
 * batval == 75	  r = 128 g = 128 b = 0
 * batval == 0    r = 255 g = 0 b = 0
 *
 *   r + g = 255
 */
	RGB_LED_Set_color((255 - ((float)batval*2.55)), ((float)batval*2.55), 0);
}

/*convert_value_to_array2
 *
 *
 * value - wartosc konwertowana
 * arraytoputin - adres tablicy do której chcemy zapisac
 * rangebegin - poczontek tablicy w którym zaczynamy zapisywac
 * rangeend - koniec tablicy
 *
 * nalezy pamientac by dodac +1 do rangeend, poniewaz pierwsze miejsce w tablicy moze byc znak -
 *
 *	[0] -
 *	[1] 2
 *	[2] 2
 */

void convert_value_to_array2(int16_t value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend){
	int x = 0;
	int loopnum = 0;
	int range = rangeend - rangebegin;

	if(value < 0){
		arraytoputin[rangebegin] = '-';
	}

	for(int i = 1; i < range+1; i++){// 3
		while(value >= (uint16_t)potenga(10,range - i)){
			if(value == 0){
				break;
			}
			value -= (uint16_t)potenga(10,range - i);
			x++;
		}
		arraytoputin[rangebegin + loopnum] = (uint8_t)x + 48;//zamiana na ASCII
		loopnum++;
		x = 0;
	}
}

/*
 * 12.055
 *
 * 1
 * 2
 * .
 * 0
 * 5
 * 5
 *
 *	1000.1
 *
 *	range = 5
 *
 */
void convert_value_to_array3(float value, uint8_t *arraytoputin, uint8_t rangebegin, uint8_t rangeend){
	int x = 0;
	int loopnum = 0;
	int range = rangeend - rangebegin;
	int power_of_value = 0;
	float a;
	int kropka;


	if(value < 0){
		arraytoputin[rangebegin] = '-';
		value = value * -1;
	}

	//sprawdzenie wagi pierwsazej liczby znaczącej
	//potrzebne do dzielenia
	for(int i = 1; i < range*2; i++){
		a = pow(10,range - i);
		if(value >= a){
			break;
		}
		power_of_value++;
	}

	power_of_value = range - power_of_value - 1;

	//		Sprawdzic gdzie jest kropka 0.00123 123.01 12.12
	if(power_of_value <= 0){
		arraytoputin[rangebegin + 1] = '.';
		kropka = 1;
	}
	if(power_of_value > 0){
		arraytoputin[rangebegin + power_of_value + 1] = '.';
		kropka = power_of_value + 1;
	}


	value = value*pow(10,(range-1) - power_of_value);


	for(int i = 0; i < range-2; i++){// range-2 poniewaz jest znak kropki i ewentualny znak minusa na początku
		while(value >= (uint16_t)pow(10,(range-1) - i)){
			if(value <= 0){
				break;
			}
			value = value - pow(10,(range-1) - i);
			x++;
		}

        if(loopnum == kropka){
            loopnum++;
        }
		arraytoputin[rangebegin + loopnum] = (uint8_t)x + 48;//zamiana na ASCII
		loopnum++;
		x = 0;
	}
}
/*
 *
 * Save to SD :
 *
 * SPEED1	5 6
 * SPEED2 	5
 * SPEED3	5
 * SPEED4	5
 *
 * MYDRON.Pitch.Angle_Error_pitch 7 8
 * MYDRON.Pitch.Angle_Error_rool 7
 * MYDRON.Pitch.Angle_Error_yaw 7
 *
 * data.x 6 7
 * data.y 6
 * data.z 6
 *
 * MYDRON.batterysize 3 4
 * MYDRON.dron_status.Connection 1 2
 *
 * 					 PID_FAC_Pitch[5]; 7 8
 *						 PID_FAC_Rool[5]; 7
 *						 						PID_FAC_Yaw[5]; 7
 *
 * Numer pentli 7 8
 *
 * Mag_Z 3
 */


int16_t ROOL_MAX_VAL(void){
	MYDRON.PID_Rool.Status = 2;
	return 5000;
}
int16_t ROOL_MIN_VAL(void){
	MYDRON.PID_Rool.Status = 1;
	return -5000;
}
int16_t PITCH_MAX_VAL(void){
	MYDRON.PID_Pitch.Status = 2;
	return 5000;
}
int16_t PITCH_MIN_VAL(void){
	MYDRON.PID_Pitch.Status = 1;
	return -5000;
}
int16_t YAW_MAX_VAL(void){
	MYDRON.PID_Yaw.Status = 2;
	return 5000;
}
int16_t YAW_MIN_VAL(void){
	MYDRON.PID_Yaw.Status = 1;
	return -5000;
}
int16_t ROOL_GOOD_VAL(void){
	MYDRON.PID_Rool.Status = 0;
	return MYDRON.PID_Rool.Angular_Rate_Value;
}
int16_t PITCH_GOOD_VAL(void){
	MYDRON.PID_Pitch.Status = 0;
	return MYDRON.PID_Pitch.Angular_Rate_Value;
}
int16_t YAW_GOOD_VAL(void){
	MYDRON.PID_Yaw.Status = 0;
	return MYDRON.PID_Yaw.Angular_Rate_Value;
}
