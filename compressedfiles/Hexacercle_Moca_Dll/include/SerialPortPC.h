#pragma once
#ifdef NATIVEUTILS_EXPORTS
#define EXPORT_API __declspec(dllexport)   
#else  
#define EXPORT_API __declspec(dllimport)   
#endif
#include "pch.h"
#include "StateMachine.h"
#define _CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>

// #ifdef _DEBUG
// #define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
// #define new DEBUG_NEW
// #endif

#define CONFIG_BUFFER_LEN 1024

// 功能的主入口, Unity调用这个方法

namespace Hexacercle
{
	struct ServerPacket
	{
		BYTE SensorType;
		float Voltage;
		float VoltPercent;
		Quaternion QuatSensor;
		Quaternion QuatSecSensor;

		Quaternion originalQuat;
		float EulerAngle[3];

		float Acc[3];
		BYTE FrontPressure;
		BYTE RearPressure;

		// No finger quat output; only euler angles
		//uint8_t FingerInitialized[5];
		//Quaternion Finger[5];
		float FingerPitch[5];
		float FingerYaw[5]; 
		float Body_Pitch_Yaw_Roll[3];
		int GunID;
		BYTE GunType;
		BYTE GunKeys;
		float axisX;
		float axisY;
		float Trigger;

		ServerPacket(
			BYTE sensorType,
			float voltage,
			float voltPercent,
			Quaternion quatSensor,
			Quaternion quatSecSensor,
			Quaternion originalquat,
			float originalEuler[3],
			float acc[3],
			BYTE frontPressure,
			BYTE rearPressure,
			float fingerPitch[5],
			float fingerYaw[5],
			float body_Pitch_Yaw_Roll[3],
			int gunID,
			BYTE gunType,
			BYTE gunKeys,
			float axisx,
			float axisy,
			float trigger):
			SensorType(sensorType),
			Voltage(voltage),
			VoltPercent(voltPercent),
			QuatSensor(quatSensor),
			QuatSecSensor(quatSecSensor),
			originalQuat(originalquat),
			FrontPressure(frontPressure),
			RearPressure(rearPressure),
			GunID(gunID),
			GunType(gunType),
			GunKeys(gunKeys),
			axisX(axisx),
			axisY(axisy),
			Trigger(trigger)
		{
			for (int i = 0; i < 5; i++)
			{
				FingerPitch[i] = fingerPitch[i];
				FingerYaw[i] = fingerYaw[i];
			}
			for (int i = 0; i < 3; i++)
			{
				Acc[i] = acc[i];
			}
			for (int i = 0; i < 3; i++)
			{
				Body_Pitch_Yaw_Roll[i] = body_Pitch_Yaw_Roll[i];
			}
			for (int i = 0; i < 3; i++)
			{
				EulerAngle[i] = originalEuler[i];
			}
		}
	};

	struct ConfigNode
	{
		BYTE Addr[5];
		BYTE Channel;
		BYTE RTUA;
		BYTE MSA;
		ConfigNode(BYTE address[5], BYTE channel, BYTE rtua, BYTE msa):
			Channel(channel),
			RTUA(rtua),
			MSA(msa)
		{
			for (int i = 0; i < 5; i++)
			{
				Addr[i] = address[i];
			}
		}
	};
	typedef void(__stdcall* ServerCallback)(
		ServerPacket data,
		void* sender
		);
	class EXPORT_API hcPortMgr
	{
	public:
		bool StartSerialSend(const char* ComName);
		bool StartSerial(const char* ComName);
		void StopSerial();
		void PowerDown();
		void GetConfig(BYTE data[256]);
		void SetServerCallback(ServerCallback callback);
		void InitSensor(int SensorType, Quaternion QuatUnity);
		bool IsConnected(int SensorType);
		void* caller = NULL;
		void SerialSend(unsigned char pData[], DWORD lenght);
		HANDLE SeriaSendHandle = INVALID_HANDLE_VALUE;
	private:
		volatile ServerCallback serverCallback = NULL;
		std::thread ThrSerial;
		bool serialReading = false;
		HANDLE serialHandle = INVALID_HANDLE_VALUE;
		
		std::map<BYTE, void*> sensors;
		std::string Comname = "";
		std::string SendOutComname = "";
		StateMachine sm;
		static void smCallback(void* sensor, void* sender);
		void initSensorMap();
		void disposeSensorMap();
		void serialRead();
		bool writeCommand(BYTE cmd);
	  
	};
	extern "C" EXPORT_API hcPortMgr * GetInstance();
	extern "C" EXPORT_API void DestroyInstance(hcPortMgr* obj);
	extern "C" EXPORT_API bool StartSerial(hcPortMgr * obj, const char* ComName);
	extern "C" EXPORT_API void StopSerial(hcPortMgr * obj);
	extern "C" EXPORT_API void PowerDown(hcPortMgr * obj);
	extern "C" EXPORT_API void GetConfig(hcPortMgr * obj, BYTE data[CONFIG_BUFFER_LEN]);
	extern "C" EXPORT_API void SetServerCallback(hcPortMgr * obj, ServerCallback callback, void* caller);
	extern "C" EXPORT_API void InitSensor(hcPortMgr * obj, int SensorType, Quaternion QuatUnity);
	extern "C" EXPORT_API bool IsConnected(hcPortMgr * obj, int SensorType);
}