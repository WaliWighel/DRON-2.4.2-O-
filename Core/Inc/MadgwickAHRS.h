//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration



typedef struct Madgwick_Struct{
	float beta;				// algorithm gain
	float q0;				// quaternion of sensor frame relative to auxiliary frame
	float q1;
	float q2;
	float q3;
	float x;
	float y;
	float z;
	float z_offset;
}Madgwick_Struct;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Madgwick_Get_Angles(void);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
