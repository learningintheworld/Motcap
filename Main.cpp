// TestCpp.cpp : This file contains the 'main' function. Program execution begins and ends there.
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
#include "fixed_point_task.h"
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
#include "data_reader.h"
#include <cstring>

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
using namespace YXWestimation;

std::map<BYTE,std::array<float, 4>> NodeMap;
// 全局变量
std::vector<double> g_row_data;
std::unique_ptr<DataReader> g_reader;
int num = 0;

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

void init_fquat()
{
    fquat0[0] = 0.9515485, fquat0[1] = 0.0381346, fquat0[2] = 0.1893079, fquat0[3] = 0.2392983;
    fquat1[0] = 0.9515485, fquat1[1] = 0.0381346, fquat1[2] = 0.1893079, fquat1[3] = 0.2392983;
    fquat2[0] = 0.9515485, fquat2[1] = 0.0381346, fquat2[2] = 0.1893079, fquat2[3] = 0.2392983;
    fquat3[0] = 0.95, fquat3[1] = 0.04, fquat3[2] = 0.19, fquat3[3] = 0.24;
    fquat4[0] = 0.92, fquat4[1] = 0.02, fquat4[2] = 0.19, fquat4[3] = 0.32;
    fquat5[0] = 0.9515485, fquat5[1] = 0.0381346, fquat5[2] = 0.1893079, fquat5[3] = 0.2392983;
    fquat6[0] = 0.9515485, fquat6[1] = 0.0381346, fquat6[2] = 0.1893079, fquat6[3] = 0.2392983;
    fquat7[0] = 0.9515485, fquat7[1] = 0.0381346, fquat7[2] = 0.1893079, fquat7[3] = 0.2392983;
    fquat8[0] = 0.9515485, fquat8[1] = 0.0381346, fquat8[2] = 0.1893079, fquat8[3] = 0.2392983;
}

void extract_IMU_raw_data(std::map<BYTE,std::array<float, 4>> *nm)
{
    for (const auto& pair : *nm) 
    {
        BYTE key = pair.first;
        const std::array<float, 4>& values = pair.second;
        std::vector<size_t> order = {3, 0, 1, 2};

        // std::cout << "Key: " << static_cast<int>(key) << " Value: ";
        // for (const auto& value : values) {
        //     std::cout << value << " ";  // 输出值并添加空格
        // }
        // std::cout << "\n";  // 空一行

        switch (static_cast<int>(key))
        {
        case 0x04:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat4[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                // std::cout << fquat4[0] << " " << fquat4[1] << " " << fquat4[2] << " " << fquat4[3] << std::endl;
                break;
            }
        case 0x05:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat3[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                break;
            }
        case 0x06:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat5[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                break;
            }
        case 0x10:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat0[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                // std::cout << fquat0[0] << " " << fquat0[1] << " " << fquat0[2] << " " << fquat0[3] << std::endl;
                break;
            }
        case 0x12:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat1[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                // std::cout << fquat1[0] << " " << fquat1[1] << " " << fquat1[2] << " " << fquat1[3] << std::endl;
                break;
            }
        case 0x13:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat2[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                // std::cout << fquat2[0] << " " << fquat2[1] << " " << fquat2[2] << " " << fquat2[3] << std::endl;
                break;
            }
        case 0x20:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat6[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                break;
            }
        case 0x22:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat7[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                break;
            }
        case 0x23:
            {
                int i = 0;
                for (const auto idx : order) {
                    if (idx < values.size()) {
                        // std::cout << "Element at index " << idx << ": " << values[idx] << std::endl;
                        // memcpy(&fquat4, &values[idx], sizeof(float));
                        fquat8[i] = values[idx];
                    } else {
                        std::cerr << "Index " << idx << " out of bounds!" << std::endl;
                    }
                    i++;
                }
                break;
            }
        
        default:
            break;
        }
    }
    
    // init_fquat();
}

void IMUTask()
{
    while (true)
    {
        
        if(NodeMap.size()==0)
        continue;
        if(!CanSend)
        continue;
        

        // for (const auto& pair : NodeMap) {
        //     BYTE key = pair.first;
        //     const std::array<float, 4>& values = pair.second;

        //     std::cout << "Key: " << static_cast<int>(key) << " Value: ";
        //     for (const auto& value : values) {
        //         std::cout << value << " ";  // 输出值并添加空格
        //     }
        //     std::cout << "\n";  // 空一行

        //     // encode_can_message(right_arm_motor_id[0], 20, motor_max_speed, &can_tx_message);
        //     // encode_com_message(&can_tx_message, &com_tx_message);
        //     // // HAL_UART_Transmit(&huart1, (, com_tx_message.Len, HAL_MAX_DELAY);
        //     // mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
        // }

		#if DEBUG
		if(right_arm_flag == 0)
		{
			for(int i = 0; i < 7; i++)
			{
				right_arm_angle[i] = right_arm_angle_00[i];
			}
		}
		else if (right_arm_flag == 1)
		{
			for(int i = 0; i < 7; i++)
			{
				right_arm_angle[i] = right_arm_angle_01[i];
			}
		}
		
		if(left_arm_flag == 0)
		{
			for(int i = 0; i < 7; i++)
			{
				left_arm_angle[i] = left_arm_angle_00[i];
			}
		}
		else if (left_arm_flag == 1)
		{
			for(int i = 0; i < 7; i++)
			{
				left_arm_angle[i] = left_arm_angle_01[i];
			}
		}
		#endif
		
		#if SYNC_IMU_ENABLE
		if(Tim3Flag)
		{
			button_ticks();
			Tim3Flag = 0;
		}
		#endif
		
		#if IMU_TASK_ENABLE
		if(IMU_Status.IsMotionTrack && (!IMU_Status.IsCalib))
		{
            extract_IMU_raw_data(&NodeMap);
			get_motor_angle();
			if(IMU_Status.IsMovmean)
			{
				movmean();
			}
			// LED0(0); 	// led0 on
		}
		else
		{
			// LED0(1); 	// led0 off
		}
		#endif

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SendData()
{
    mgr.StartSerialSend("COM24");//发送至Robot

    while (true)
    {
		#if SYNC_IMU_ENABLE
		if(Tim2Flag)
		{
//			int length01 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f, %0.2f\r\n", quat_sync_IMU23.w, quat_sync_IMU23.v[0], quat_sync_IMU23.v[1], quat_sync_IMU23.v[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length01, HAL_MAX_DELAY);
			if(IMU_Status.IsCalib && (!IMU_Status.IsMotionTrack))
			{
				if(isSyncParamEmpty())
				{
					postureIMU();
					LED1(0); 	// led1 on
				}
				else
				{
					if(OutputSyncFlag)
					{
						get_sync_param();
						int length02 = snprintf(buffer, sizeof(buffer), "%0.6f, %0.6f, %0.6f, %0.6f\r\n", quat_sync_IMU23.w, quat_sync_IMU23.v[0], quat_sync_IMU23.v[1], quat_sync_IMU23.v[2]);
						HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length02, HAL_MAX_DELAY);
						OutputSyncFlag = 0;
					}
					LED1(1); 	// led1 off
				}
			}
			Tim2Flag = 0;
		}
		#endif

        #if HAND_ENABLE
        // if(hand_task_timer < 40*2)
        // {
        //     hand_motion_status.IsSideSway = 1;
        // }
        // else
        // {
        //     hand_motion_status.IsSideSway = 0;
        // }
        // hand_motion();
        // hand_task_timer++;
		#endif

		#if IMU_TASK_ENABLE
        // millis += 25;
        if(IMU_Status.IsCalib && (!IMU_Status.IsMotionTrack))
        {
            if(isCalibParamEmpty())
            {
                extract_IMU_raw_data(&NodeMap);
                calibration();
                // LED1(0); 	// led1 on
                std::cout << "Calibing..." << std::endl;
            }
            else
            {
//					int length = snprintf(buffer, sizeof(buffer), "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\r\n", 
//															quat_calib_IMU67.w, quat_calib_IMU67.v[0], quat_calib_IMU67.v[1], quat_calib_IMU67.v[2], quat_calib_IMU78.w, quat_calib_IMU78.v[0], quat_calib_IMU78.v[1], quat_calib_IMU78.v[2]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, HAL_MAX_DELAY);
                // LED1(1); 	// led1 off
                if (!isQuaternionEmpty(&quat_calib_IMU43) || !isQuaternionEmpty(&quat_calib_IMU23) || !isQuaternionEmpty(&quat_calib_IMU12) || !isQuaternionEmpty(&quat_calib_IMU01) || !isQuaternionEmpty(&quat_calib_IMU83) || !isQuaternionEmpty(&quat_calib_IMU78) || !isQuaternionEmpty(&quat_calib_IMU67))
                {
                    std::cout << "Calib Finshed" << std::endl;
                }
            }
        }
        
        if(IMU_Status.IsMotionTrack && (!IMU_Status.IsCalib))
        {
            if(IMU_Status.IsMovmean)
            {
                #if RIGHT_ARM_ENABLE
                for (int i = 0; i < 7; i++)
                {
                    movmean_motor_angle[i] = round(movmean_motor_angle[i] * 1000) / 1000;
                }
                // std::cout << movmean_motor_angle[0] << " " << movmean_motor_angle[1] << " " << movmean_motor_angle[2] << std::endl;
                // std::cout << movmean_motor_angle[3] << " " << movmean_motor_angle[4] << " " << movmean_motor_angle[5] << " " << movmean_motor_angle[6] << std::endl;
                // int length = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f\r\n",
                //                                             movmean_motor_angle[0], movmean_motor_angle[1], movmean_motor_angle[2]);
                // mgr.SerialSend((uint8_t*)& buffer, length);
                if (IMU_Status.IsPrint == 1)
                {
                    for (int i = 0; i < 7; i++)
                    {
                        std::cout << movmean_motor_angle[i] << " ";
                    }
                    // std::cout << "" << std::endl;
                    // std::cout << "";
                    // std::cout << "" << std::endl;
                }
                
                for(int i = 0; i < 7; i++)
                {
                    encode_can_message(ARM_MODULE_ID, right_arm_motor_id[i], movmean_motor_angle[i], motor_max_speed, &can_tx_message);
                    encode_com_message(&can_tx_message, &com_tx_message);
                    mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
                }
                #endif
                #if HEAD_ENABLE
                for (int i = 0; i < 2; i++)
                {
                    movmean_motor_angle[i + 7] = round(movmean_motor_angle[i + 7] * 1000) / 1000;
                }
                // if (!(std::isnan(movmean_motor_angle[7]) && std::isnan(movmean_motor_angle[8])))
                // {
                //     std::cout << movmean_motor_angle[7] << " " << movmean_motor_angle[8] << endl;
                // }
                // std::cout << movmean_motor_angle[7] << " " << movmean_motor_angle[8] << std::endl;
                // std::cout << motor_angle[7] << " " << motor_angle[8] << std::endl;
                // std::cout << "\n" << std::endl;
                // motor_angle[7] = 40;
                // motor_angle[8] = 25;
                for(int i = 0; i < 2; i++)
                {
                    encode_can_message(ZED_MODULE_ID, head_motor_id[i], movmean_motor_angle[i + 7], 20*16, &can_tx_message);
                    encode_com_message(&can_tx_message, &com_tx_message);
                    mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
                }
                #endif
                #if WAIST_ENABLE
                int length03 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f\r\n",
                                                            movmean_motor_angle[9], movmean_motor_angle[10], movmean_motor_angle[11]);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length03, HAL_MAX_DELAY);
                #endif
                #if LEFT_ARM_ENABLE
                for (int i = 0; i < 7; i++)
                {
                    movmean_motor_angle[i + 12] = round(movmean_motor_angle[i + 12] * 1000) / 1000;
                }
				// int length01 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n",
				// 											movmean_motor_angle[12], movmean_motor_angle[13], movmean_motor_angle[14], movmean_motor_angle[15], movmean_motor_angle[16], movmean_motor_angle[17], movmean_motor_angle[18]);
				// // HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length01, HAL_MAX_DELAY);
                // mgr.SerialSend((uint8_t*)& buffer, length01);
				// int length00 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f\r\n",
				// 											movmean_motor_angle[15], movmean_motor_angle[16]);
                // mgr.SerialSend((uint8_t*)& buffer, length00);
                // std::cout << motor_angle[15] << " " << motor_angle[16] << std::endl;
                if (IMU_Status.IsPrint == 1)
                {
                    for (int i = 0; i < 7; i++)
                    {
                        std::cout << movmean_motor_angle[i + 12] << " ";
                    }
                    std::cout << "" << std::endl;
                }

                for(int i = 0; i < 7; i++)
                {
                    encode_can_message(ARM_MODULE_ID, left_arm_motor_id[i], movmean_motor_angle[i + 12], motor_max_speed, &can_tx_message);
                    encode_com_message(&can_tx_message, &com_tx_message);
                    // HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
                    mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
                }
                #endif
                #if HAND_ENABLE
                if(hand_task_timer < 40*2)
                {
                    hand_motion_status.IsSideSway = 1;
                }
                else
                {
                    hand_motion_status.IsSideSway = 0;
                }
                hand_motion();
                hand_task_timer++;
                #endif
                #if DATA_ENABLE
                if (g_isPaused) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(25));
                    continue;
                }

                // 只在不跳过检查时检查停止点
                if (!skipStopCheck) {
                    for (int i = 0; i < YXWestimation::NUM_STOP_POINTS; i++) {
                        if (g_currentRow == STOP_POINTS[i]) {
                            g_isPaused = true;
                            break;
                        }
                    }
                }
                if (!g_isPaused) {
                    try {
                        if (g_isReverse) {
                            g_row_data = g_reader->getPreviousRow();
                        } else {
                            g_row_data = g_reader->getNextRow();
                        }
                        
                        // 更新数据
                        // 复制前7个元素 (0-6)
                        std::memcpy(movmean_motor_angle, g_row_data.data() + 7, 7 * sizeof(double));
                        // 复制后7个元素 (12-18)
                        std::memcpy(movmean_motor_angle + 12, g_row_data.data(), 7 * sizeof(double));
                    }
                    catch (const std::exception& e) {
                        std::cerr << "数据读取错误: " << e.what() << std::endl;
                        g_isPaused = true;
                    }
                }
                g_currentRow = g_reader->getCurrentRow();

                if (!skipStopCheck) {
                    std::cout << g_currentRow << " ";
                    for (int i = 0; i < 7; i++)
                    {
                        movmean_motor_angle[i] = round(movmean_motor_angle[i] * 1000) / 1000;
                    }
                    if (IMU_Status.IsPrint == 1)
                    {
                        for (int i = 0; i < 7; i++)
                        {
                            std::cout << movmean_motor_angle[i] << " ";
                        }
                    }
                    for(int i = 0; i < 7; i++)
                    {
                        encode_can_message(ARM_MODULE_ID, right_arm_motor_id[i], movmean_motor_angle[i], motor_max_speed, &can_tx_message);
                        encode_com_message(&can_tx_message, &com_tx_message);
                        mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
                    }
                    for (int i = 0; i < 7; i++)
                    {
                        movmean_motor_angle[i + 12] = round(movmean_motor_angle[i + 12] * 1000) / 1000;
                    }
                    if (IMU_Status.IsPrint == 1)
                    {
                        for (int i = 0; i < 7; i++)
                        {
                            std::cout << movmean_motor_angle[i + 12] << " ";
                        }
                        std::cout << "" << std::endl;
                    }
                    for(int i = 0; i < 7; i++)
                    {
                        encode_can_message(ARM_MODULE_ID, left_arm_motor_id[i], movmean_motor_angle[i + 12], motor_max_speed, &can_tx_message);
                        encode_com_message(&can_tx_message, &com_tx_message);
                        // HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
                        mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
                    }
                }
                skipStopCheck = false;  // 重置跳过标志
                #endif
            }
            else
            {
                #if RIGHT_ARM_ENABLE
//					int length = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n",
//															motor_angle[0], motor_angle[1], motor_angle[2], motor_angle[3], motor_angle[4], motor_angle[5], motor_angle[6]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, HAL_MAX_DELAY);
                for(int i = 0; i < 7; i++)
                {
                    encode_can_message(ARM_MODULE_ID, right_arm_motor_id[i], motor_angle[i], motor_max_speed, &can_tx_message);
                    encode_com_message(&can_tx_message, &com_tx_message);
                    // HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
                }
                #endif
                #if HEAD_ENABLE
                // int length02 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f\r\n",
                //                                             motor_angle[7], motor_angle[8]);
                // HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length02, HAL_MAX_DELAY);
                #endif
                #if WAIST_ENABLE
                int length03 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f\r\n",
                                                            motor_angle[9], motor_angle[10], motor_angle[11]);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length03, HAL_MAX_DELAY);
                #endif
                #if LEFT_ARM_ENABLE
//					int length = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n",
//															motor_angle[12], motor_angle[13], motor_angle[14], motor_angle[15], motor_angle[16], motor_angle[17], motor_angle[18]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, HAL_MAX_DELAY);
                for(int i = 0; i < 7; i++)
                {
                    encode_can_message(ARM_MODULE_ID, left_arm_motor_id[i], motor_angle[i + 12], motor_max_speed, &can_tx_message);
                    encode_com_message(&can_tx_message, &com_tx_message);
                    // HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
                    mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
                }
                #endif
                #if HAND_ENABLE
                if(hand_task_timer < 40*2)
                {
                    hand_motion_status.IsSideSway = 1;
                }
                else
                {
                    hand_motion_status.IsSideSway = 0;
                }
                hand_motion();
                hand_task_timer++;
                #endif
            }
        }
		#endif

        #if FIXED_POINT_ENABLE
        if (arm_posture.IsSend == 1)
        {

        }
        
        #endif

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
            if (KeyB == 'S') 
            {
                CanSend=true;
            }
            if(KeyB == 'Q')
            {
                CanSend=false;
            }
            if(KeyB == 'C')
            {
                IMU_Status.IsCalib = !IMU_Status.IsCalib;
                if (IMU_Status.IsCalib == 1)
                {
                    CalibParamReset();
                }
            }
            if(KeyB == 'T')
            {
                IMU_Status.IsMotionTrack = !IMU_Status.IsMotionTrack;
            }
            if(KeyB == 'M')
            {
                IMU_Status.IsMovmean = !IMU_Status.IsMovmean;
            }
            if(KeyB == 'H')
            {
				hand_motion_status.IsGrasp = !hand_motion_status.IsGrasp;
				hand_motion_status.IsLoosen = !hand_motion_status.IsLoosen;
				if(hand_motion_status.IsGrasp == 1)
				{
					hand_task_timer = 0;
				}
            }
            if(KeyB == 'P')
            {
                arm_posture.IsSend = 1;
                arm_posture.PostureStatus += 1;
            }
            if(KeyB == 'O')
            {
                IMU_Status.IsPrint = !IMU_Status.IsPrint;
            }
            if(KeyB == 'N')
            {
                if (Track_Param.param_id < 17)
                {
                    Track_Param.param_id += 1;
                }
                else
                {
                    Track_Param.param_id = 0;
                }
                std::cout << Track_Param.param_id << std::endl;
            }
            if(KeyB == 'A')
            {
                if (Track_Param.param_id == 2 || Track_Param.param_id == 5 || Track_Param.param_id == 11 || Track_Param.param_id == 14)
                {
                    Track_Param.param[Track_Param.param_id] += 0.01;
                }
                else
                {
                    Track_Param.param[Track_Param.param_id] += 1;
                }

                for (int i = 0; i < 18; i++)
                {
                    std::cout << Track_Param.param[i] << " ";
                }
                std::cout << "" << std::endl;
            }
            if(KeyB == 'D')
            {
                if (Track_Param.param_id == 2 || Track_Param.param_id == 5 || Track_Param.param_id == 11 || Track_Param.param_id == 14)
                {
                    Track_Param.param[Track_Param.param_id] -= 0.01;
                }
                else
                {
                    Track_Param.param[Track_Param.param_id] -= 1;
                }

                for (int i = 0; i < 18; i++)
                {
                    std::cout << Track_Param.param[i] << " ";
                }
                std::cout << "" << std::endl;
            }
            if(KeyB == 'B')
            {
                g_isReverse = true;
                g_isPaused = false;
                skipStopCheck = true;  // 设置跳过标志
            }
            if(KeyB == 'G')
            {
                g_isReverse = false;
                g_isPaused = false;
                skipStopCheck = true;  // 设置跳过标志
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main()
{
    // 临时取消DEBUG_NEW定义
    #ifdef new
    #undef new
    #endif
    try {
        // 创建DataReader对象
        g_reader = std::make_unique<DataReader>("F:\\OneDrive\\Code\\Motcap\\data\\explosive_removal_1220_1.csv");
    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    // 重新定义DEBUG_NEW
    #ifdef _DEBUG
    #define new DEBUG_NEW
    #endif

    // std::cout << "hello world" << std::endl;
    #if IMU_TASK_ENABLE
    movmean_init();
    Hexacercle::hcPortMgr mgr;

//  vector<char> input = { 'A','A','A','B','B','B' };
//  auto result = Solution().leastInterval(input, 0);
//    //return 0;

//     ////_CrtMemState sOld;
//     ////_CrtMemState sNew;
//     ////_CrtMemState sDiff;
//     ////_CrtMemCheckpoint(&sOld); //take a snapchot

    
    mgr.StartSerial("COM5");//接收动捕数据

    // mgr.StartSerialSend("COM11");//发送至Robot
    // BYTE msg[] = { 0xA0, 0xFF, 0xFF, 0x28, 0x01, 0x02, 0xff, 0x0A };
    // mgr.SerialSend(msg,10);//发送至Robot

    mgr.SetServerCallback(callback);
    mgr.InitSensor(SensorTypes::RightGlove, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::Head, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::Abdomen, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::Chest, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::RightShoulder, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::RightElbow, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::LeftShoulder, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::LeftElbow, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::LeftGlove, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    mgr.InitSensor(SensorTypes::RightGlove, Hexacercle::Quaternion::MakeQuat(0.717, 0, -0.717, 0));
    #endif

    //SendData();
    std::thread thread1(SendData);//发送数据
    std::thread thread2(Key_Watch);//检测s键盘事件 S发送，L停止发送
    std::thread thread3(IMUTask);
    thread1.join();
    thread2.join();
    thread3.join();

    #if IMU_TASK_ENABLE
    getchar();
    mgr.StopSerial();
    #endif
    
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
    return 0;
}













