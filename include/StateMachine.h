#pragma once

#include <vector>
#include <functional>
#include "hcQuaternion.h"
#include "hcSensor.h"
#include "spsc_queue.h"


namespace Hexacercle
{
	//typedef void(__stdcall* MachineCallback)(void*);
	class EXPORT_API StateMachine
	{
	public:
		void Process(uint8_t * buffer, int size);
		//void SetMachineCallback(MachineCallback callback);
		void fSetMachineCallback(void param(void*, void*), void* sender);
		void DisposeMachine();
	private:
		void* caller = NULL;
		void (*_callback)(void*, void*) = NULL;
		//MachineCallback _callback;
		std::thread ThrDecode;
		bool processing = false;
		spsc_queue<BYTE> buffer;
		void Decode();
	};
}