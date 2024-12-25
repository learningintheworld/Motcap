// TestCpp.cpp : This file contains the 'main' function. Program execution begins and ends there.
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
#include "hand_task.h"
#include "MovingAverageFilter.h"
#include "Quaternion.h"
#include "wit_sync.h"
#include "wit_task.h"
#include "SerialPortPC.h"
#include <stdlib.h>  
#include <windows.h>
#include <iostream>
#include <debugapi.h>
#include <Math.h>
#include <cstdlib>

#define _CRTDBG_MAP_ALLOC  
#include <crtdbg.h>  
#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

#include <set>
#include <stack>
#include <list>
#include <algorithm>
#include <numeric>
#include <ctype.h>
#include <map>
#include <array>
#include <deque>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <conio.h> 
#include <cctype> 

//**********************************
#include <stdint.h>
#include<string.h>
// #include "C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/VC/Tools/MSVC/14.29.30133/include/iomanip"
//**********************************

using namespace Hexacercle;
using namespace std;

std::map<BYTE,std::array<float, 4>> NodeMap;

hcPortMgr mgr;
bool CanSend=false;

void __stdcall callback(ServerPacket data, void* sender)
{

    if(data.SensorType==255)
      return;
     auto node= NodeMap.find(data.SensorType);
     if(node!=NodeMap.end())
     {

        node->second[0]=data.originalQuat.x;
        node->second[1]=data.originalQuat.y;
        node->second[2]=data.originalQuat.z;
        node->second[3]=data.originalQuat.w;
     }
     else
     {
         //float nodeValue[4]={data.originalQuat.x,data.originalQuat.y,data.originalQuat.z,data.originalQuat.w};
         NodeMap[data.SensorType]={data.originalQuat.x,data.originalQuat.y,data.originalQuat.z,data.originalQuat.w};
     }


    // if (data.SensorType == SensorTypes::Head)
    // {
        
    //   /*  std::cout << data.FingerPitch[0]<< " " << data.FingerPitch[1]<< " "
    //         << data.FingerPitch[2] << " " << data.FingerPitch[3] << std::endl;*/

    //     std::cout << data.originalQuat.x << "\t" << data.originalQuat.y << "\t" << data.originalQuat.z << "\t" << data.originalQuat.w <<"   \t" <<
    //         data.EulerAngle[0]<<"\t"<< data.EulerAngle[1]<<"\t"<<data.EulerAngle[2] << std::endl;

    //     // BYTE msg[] = { data.QuatSensor.x, data.QuatSensor.y,data.QuatSensor.z, data.QuatSensor.w};
    //    // encode_can_message(right_arm_motor_id[0], 20, motor_max_speed, &can_tx_message);
    //    // encode_com_message(&can_tx_message, &com_tx_message);
    //    //// HAL_UART_Transmit(&huart1, (, com_tx_message.Len, HAL_MAX_DELAY);
    //    // mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
    //    // Sleep(20);
    // }

}

void IMUTask()
{
    while (true)
    {
        


        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    
}

void SendData()
{
   mgr.StartSerialSend("COM5");//发送至Robot

  while (true)
  {
    if(NodeMap.size()==0)
    continue;
    if(!CanSend)
    continue;
    for (const auto& pair : NodeMap) {
        BYTE key = pair.first;
        const std::array<float, 4>& values = pair.second;

        std::cout << "Key: " << static_cast<int>(key) << " Value: ";
        for (const auto& value : values) {
             std::cout << value << " ";  // 输出值并添加空格
        }
       std::cout << "\n";  // 空一行

       encode_can_message(right_arm_motor_id[0], 20, motor_max_speed, &can_tx_message);
       encode_com_message(&can_tx_message, &com_tx_message);
       // HAL_UART_Transmit(&huart1, (, com_tx_message.Len, HAL_MAX_DELAY);
       mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
}

void Key_Watch() {
    // int count = 0;
    while (true) {
        if (_kbhit()) 
        { 
            // 检测是否有键盘输入
            char key = _getch();  // 获取按下的键
            char KeyB = std::toupper(key); 
            if (KeyB == 'K') 
            {
               CanSend=true;
            }
            if(KeyB == 'L')
            {
              CanSend=false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}


int main()
{
    Hexacercle::hcPortMgr mgr;

    // vector<char> input = { 'A','A','A','B','B','B' };
    // auto result = Solution().leastInterval(input, 0);
//     //return 0;

//     ////_CrtMemState sOld;
//     ////_CrtMemState sNew;
//     ////_CrtMemState sDiff;
//     ////_CrtMemCheckpoint(&sOld); //take a snapchot

        mgr.StartSerial("COM3");//接收动捕数据

        // mgr.StartSerialSend("COM11");//发送至Robot
        // BYTE msg[] = { 0xA0, 0xFF, 0xFF, 0x28, 0x01, 0x02, 0xff, 0x0A };
        // mgr.SerialSend(msg,10);//发送至Robot

        mgr.SetServerCallback(callback);
        mgr.InitSensor(SensorTypes::RightGlove, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::Head, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::Abdomen, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::Chest, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::RightShoulder, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::RightElbow, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::LeftShoulder, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::LeftElbow, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::LeftGlove, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
        mgr.InitSensor(SensorTypes::RightGlove, Quaternion::MakeQuat(0.717, 0, -0.717, 0));
       

        //SendData();
        std::thread thread1(SendData);//发送数据
        std::thread thread2(Key_Watch);//检测键盘事件 S发送，L停止发送
        std::thread thread3(IMUTask);
        thread1.join();
        thread2.join();
        thread3.join();

        getchar();
        mgr.StopSerial();
//     ////_CrtMemCheckpoint(&sNew); //take a snapchot 
//     ////if (_CrtMemDifference(&sDiff, &sOld, &sNew)) // if there is a difference
//     ////{
// 	   //// OutputDebugStringW(L"-----------_CrtMemDumpStatistics ---------");
// 	   //// _CrtMemDumpStatistics(&sDiff);
// 	   //// OutputDebugStringW(L"-----------_CrtMemDumpAllObjectsSince ---------");
// 	   //// _CrtMemDumpAllObjectsSince(&sOld);
// 	   //// OutputDebugStringW(L"-----------_CrtDumpMemoryLeaks ---------");
// 	   //// _CrtDumpMemoryLeaks();
//     ////}
}












