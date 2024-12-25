#pragma once
#include "hcQuaternion.h"
#include "Filter.h"
#include "hcSensor.h"
#include <cmath>
namespace Hexacercle {
	void GetEuler(Quaternion Hand, Quaternion Finger[5], float *OutPitch, float* OutYaw);
	void GetBodyEuler(Quaternion Target,Quaternion RelativeTarget,float Body_Pitch_Yaw_Roll[3]);
	float FilterVoltage(float prevValue, float newValue);
	float VoltToPercent(float Volt);
	Quaternion InitQuatTrans(hcSensor* Sensor, int SensorType, Quaternion UnityQuat);
	Quaternion GetUnityQuat(hcSensor* Sensor, int SensorType, Quaternion QuatTrans);
	void switchXYZ(Quaternion* quat);
	void quaternionToRotationMatrix(Quaternion q, double R[3][3]); 
	void yawPitchMatrix(Quaternion quat0, Quaternion quat1, double* x, double* y, double* z);
}