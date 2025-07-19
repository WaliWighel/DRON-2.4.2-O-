//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>
#include "main.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	1000.0f		// sample frequency in Hz
//---------------------------------------------------------------------------------------------------
// Variable definitions

//volatile float beta = betaDef;								// 2 * proportional gain (Kp)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
extern Madgwick_Struct Madgwick;
//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm = 0;
	float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
	float qDot1 = 0, qDot2 = 0, qDot3 = 0, qDot4 = 0;
	float hx = 0, hy = 0;
	float _2q0mx = 0, _2q0my = 0, _2q0mz = 0, _2q1mx = 0, _2bx = 0, _2bz = 0, _4bx = 0, _4bz = 0, _2q0 = 0, _2q1 = 0, _2q2 = 0, _2q3 = 0, _2q0q2 = 0, _2q2q3 = 0, q0q0 = 0, q0q1 = 0, q0q2 = 0, q0q3 = 0, q1q1 = 0, q1q2 = 0, q1q3 = 0, q2q2 = 0, q2q3 = 0, q3q3 = 0;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-Madgwick.q1 * gx - Madgwick.q2 * gy - Madgwick.q3 * gz);
	qDot2 = 0.5f * (Madgwick.q0 * gx + Madgwick.q2 * gz - Madgwick.q3 * gy);
	qDot3 = 0.5f * (Madgwick.q0 * gy - Madgwick.q1 * gz + Madgwick.q3 * gx);
	qDot4 = 0.5f * (Madgwick.q0 * gz + Madgwick.q1 * gy - Madgwick.q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * Madgwick.q0 * mx;
		_2q0my = 2.0f * Madgwick.q0 * my;
		_2q0mz = 2.0f * Madgwick.q0 * mz;
		_2q1mx = 2.0f * Madgwick.q1 * mx;
		_2q0 = 2.0f * Madgwick.q0;
		_2q1 = 2.0f * Madgwick.q1;
		_2q2 = 2.0f * Madgwick.q2;
		_2q3 = 2.0f * Madgwick.q3;
		_2q0q2 = 2.0f * Madgwick.q0 * Madgwick.q2;
		_2q2q3 = 2.0f * Madgwick.q2 * Madgwick.q3;
		q0q0 = Madgwick.q0 * Madgwick.q0;
		q0q1 = Madgwick.q0 * Madgwick.q1;
		q0q2 = Madgwick.q0 * Madgwick.q2;
		q0q3 = Madgwick.q0 * Madgwick.q3;
		q1q1 = Madgwick.q1 * Madgwick.q1;
		q1q2 = Madgwick.q1 * Madgwick.q2;
		q1q3 = Madgwick.q1 * Madgwick.q3;
		q2q2 = Madgwick.q2 * Madgwick.q2;
		q2q3 = Madgwick.q2 * Madgwick.q3;
		q3q3 = Madgwick.q3 * Madgwick.q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * Madgwick.q3 + _2q0mz * Madgwick.q2 + mx * q1q1 + _2q1 * my * Madgwick.q2 + _2q1 * mz * Madgwick.q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * Madgwick.q3 + my * q0q0 - _2q0mz * Madgwick.q1 + _2q1mx * Madgwick.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * Madgwick.q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * Madgwick.q2 + _2q0my * Madgwick.q1 + mz * q0q0 + _2q1mx * Madgwick.q3 - mz * q1q1 + _2q2 * my * Madgwick.q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * Madgwick.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * Madgwick.q3 + _2bz * Madgwick.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * Madgwick.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * Madgwick.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * Madgwick.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * Madgwick.q2 + _2bz * Madgwick.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * Madgwick.q3 - _4bz * Madgwick.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * Madgwick.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * Madgwick.q2 - _2bz * Madgwick.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * Madgwick.q1 + _2bz * Madgwick.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * Madgwick.q0 - _4bz * Madgwick.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * Madgwick.q3 + _2bz * Madgwick.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * Madgwick.q0 + _2bz * Madgwick.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * Madgwick.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= Madgwick.beta * s0;
		qDot2 -= Madgwick.beta * s1;
		qDot3 -= Madgwick.beta * s2;
		qDot4 -= Madgwick.beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	Madgwick.q0 += qDot1 * (1.0f / sampleFreq);
	Madgwick.q1 += qDot2 * (1.0f / sampleFreq);
	Madgwick.q2 += qDot3 * (1.0f / sampleFreq);
	Madgwick.q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(Madgwick.q0 * Madgwick.q0 + Madgwick.q1 * Madgwick.q1 + Madgwick.q2 * Madgwick.q2 + Madgwick.q3 * Madgwick.q3);
	Madgwick.q0 *= recipNorm;
	Madgwick.q1 *= recipNorm;
	Madgwick.q2 *= recipNorm;
	Madgwick.q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-Madgwick.q1 * gx - Madgwick.q2 * gy - Madgwick.q3 * gz);
	qDot2 = 0.5f * (Madgwick.q0 * gx + Madgwick.q2 * gz - Madgwick.q3 * gy);
	qDot3 = 0.5f * (Madgwick.q0 * gy - Madgwick.q1 * gz + Madgwick.q3 * gx);
	qDot4 = 0.5f * (Madgwick.q0 * gz + Madgwick.q1 * gy - Madgwick.q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * Madgwick.q0;
		_2q1 = 2.0f * Madgwick.q1;
		_2q2 = 2.0f * Madgwick.q2;
		_2q3 = 2.0f * Madgwick.q3;
		_4q0 = 4.0f * Madgwick.q0;
		_4q1 = 4.0f * Madgwick.q1;
		_4q2 = 4.0f * Madgwick.q2;
		_8q1 = 8.0f * Madgwick.q1;
		_8q2 = 8.0f * Madgwick.q2;
		q0q0 = Madgwick.q0 * Madgwick.q0;
		q1q1 = Madgwick.q1 * Madgwick.q1;
		q2q2 = Madgwick.q2 * Madgwick.q2;
		q3q3 = Madgwick.q3 * Madgwick.q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * Madgwick.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * Madgwick.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * Madgwick.q3 - _2q1 * ax + 4.0f * q2q2 * Madgwick.q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= Madgwick.beta * s0;
		qDot2 -= Madgwick.beta * s1;
		qDot3 -= Madgwick.beta * s2;
		qDot4 -= Madgwick.beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	Madgwick.q0 += qDot1 * (1.0f / sampleFreq);
	Madgwick.q1 += qDot2 * (1.0f / sampleFreq);
	Madgwick.q2 += qDot3 * (1.0f / sampleFreq);
	Madgwick.q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(Madgwick.q0 * Madgwick.q0 + Madgwick.q1 * Madgwick.q1 + Madgwick.q2 * Madgwick.q2 +Madgwick.q3 * Madgwick.q3);
	Madgwick.q0 *= recipNorm;
	Madgwick.q1 *= recipNorm;
	Madgwick.q2 *= recipNorm;
	Madgwick.q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void Madgwick_Get_Angles(void){// NaN output
	Madgwick.x = atan2f(Madgwick.q0*Madgwick.q1 + Madgwick.q2*Madgwick.q3, 0.5f - Madgwick.q1*Madgwick.q1 - Madgwick.q2*Madgwick.q2)*57.295779513082320876798154814105f;
	Madgwick.y = asinf(-2.0f * (Madgwick.q1*Madgwick.q3 - Madgwick.q0*Madgwick.q2))*(57.295779513082320876798154814105f);
	Madgwick.z = atan2f(Madgwick.q1*Madgwick.q2 + Madgwick.q0*Madgwick.q3, 0.5f - Madgwick.q2*Madgwick.q2 - Madgwick.q3*Madgwick.q3)*57.295779513082320876798154814105f;
}
//====================================================================================================
// END OF CODE
//====================================================================================================
