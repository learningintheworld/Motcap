#pragma once
#ifdef NATIVEUTILS_EXPORTS
#define EXPORT_API __declspec(dllexport)   
#else  
#define EXPORT_API __declspec(dllimport)   
#endif
#include "hcQuaternion.h"
namespace Hexacercle
{
	struct Param
	{
		Quaternion prevData;
		Quaternion p;
		Quaternion gain;
		Param():
			prevData(),
			p(),
			gain()
		{}
		Param(const Param& other) :
			prevData(other.prevData),
			p(other.p),
			gain(other.gain)
		{}
	};
	extern "C" EXPORT_API Quaternion ProcessParamHC(Quaternion inQuat, Param * param);
	extern "C" EXPORT_API void SetParam(float value);
}