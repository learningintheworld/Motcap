#include "wit_task.h"
// #include "wit_c_sdk.h"
//#include "main.h"
// #include "usart.h"
// #include "led.h"
#include "hand_task.h"
#include "wit_sync.h"
#include <string.h>
#include <iostream>
#include <math.h>
#include <fstream>

// using YXWestimation::Quaternion;
//#define wit_data_num 44
//#define wit_buf_num (((wit_data_num >> 3) + 1) << 3)
// using namespace YXWestimation;
namespace YXWestimation {

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

#ifndef TO_RAD
		#define TO_RAD(x) (x / 180.0 * M_PI)
#endif

#ifndef TO_DEG
		#define TO_DEG(x) (x / M_PI * 180.0)
#endif


// 	IMU0~2--right arm		IMU3--benchmark		IMU4--head	IMU5--waist		IMU6~8--left arm
float fAcc0[3], fGyro0[3], fAngle0[3], fmag0[3], fquat0[4];
float fAcc1[3], fGyro1[3], fAngle1[3], fmag1[3], fquat1[4];
float fAcc2[3], fGyro2[3], fAngle2[3], fmag2[3], fquat2[4];
float fAcc3[3], fGyro3[3], fAngle3[3], fmag3[3], fquat3[4];
float fAcc4[3], fGyro4[3], fAngle4[3], fmag4[3], fquat4[4];
float fAcc5[3], fGyro5[3], fAngle5[3], fmag5[3], fquat5[4];
float fAcc6[3], fGyro6[3], fAngle6[3], fmag6[3], fquat6[4];
float fAcc7[3], fGyro7[3], fAngle7[3], fmag7[3], fquat7[4];
float fAcc8[3], fGyro8[3], fAngle8[3], fmag8[3], fquat8[4];

char buffer[300];
uint8_t wit_can_buf[8];
bool Tim2Flag = 0;
volatile int32_t seconds = 0;
volatile int32_t millis = 0-25;
int32_t i = 0-1;
// 右从手到臂：0-1-2	基座：3		头：4		腰：5	左从手到臂：6-7-8
CalibQuats quats_0 = {.count = 0};	// Be initialized before main
CalibQuats quats_1 = {.count = 0};
CalibQuats quats_2 = {.count = 0};
CalibQuats quats_3 = {.count = 0};
CalibQuats quats_4 = {.count = 0};
CalibQuats quats_5 = {.count = 0};
CalibQuats quats_6 = {.count = 0};
CalibQuats quats_7 = {.count = 0};
CalibQuats quats_8 = {.count = 0};
Quaternion quat_BSR_inv_0, quat_BSR_inv_1, quat_BSR_inv_2, quat_BSR_inv_3;
Quaternion quat_BSR_inv_4, quat_BSR_inv_5, quat_BSR_inv_6, quat_BSR_inv_7, quat_BSR_inv_8;
Quaternion quat_calib_IMU23, quat_calib_IMU12, quat_calib_IMU01;  // right arm
Quaternion quat_calib_IMU83, quat_calib_IMU78, quat_calib_IMU67;  // left arm
Quaternion quat_calib_IMU43;  // head
Quaternion quat_calib_IMU53;  // waist

//	angle0~6--right arm		angle7~8--head	angle9~11--waist	angle12~18--left arm
double joint_angle[19];
double motor_angle[19];
double movmean_motor_angle[19];
// struct Button key1;
//const int key1_id = 0;
bool Tim3Flag = 0;
//	param0~5--right arm		param6~11--left arm
/*	程坤华：.param = {0, 51, 1.58, 0, 21, 1.05, 10, 31, 1.68, 23.65, 6, 1.2}
			.param = {16, 51, 1.58, 3, -8, 1.2, 6, 51, 1.68, 26.65, 8, 1.2}	
			.param = {16-5, 51, 1.58, 3, -8+15, 1.2, 6, 51, 1.68, 26.65, 8, 1.2}	now
	张博宁：.param = {8, 19, 1.58, 0, 5, 1.05, 12, 28, 1.68, 22.65, -10, 1.05}		now
*/
/*	param0~8--right arm								param9~17--left arm
	0/1/3--肢体与IMU物体偏差						 9/10/12--肢体与IMU物体偏差
	2/5--旋转时肌肉影响补偿							 11/14--旋转时肌肉影响补偿
	4/6/7/8--机器零位偏差+机器与人之间校准时偏差	  13/15/16/17--机器零位偏差+机器与人之间校准时偏差
*/	
/**/
MotionTrackParams Track_Param = {.param = {0, 0, +1.05, 44, 6, +1.1, +0, +6, +10, 0, 0, +1.05, 50, +15, +1.15, +0-6, +5, +10}};
MotionTrackStatus IMU_Status = {0, 0, 1, 1};
CanTxMessage can_tx_message;
uint8_t head_motor_id[2] = {0x01, 0x02};
uint8_t left_arm_motor_id[7] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
uint8_t right_arm_motor_id[7] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
uint8_t waist_motor_id[3] = {23, 24, 25};
uint16_t motor_max_speed = 0;
ComTxMessage com_tx_message;
bool g_isPaused = false;
bool g_isReverse = false;
bool skipStopCheck = false; 
int g_currentRow = 0;
const int NUM_STOP_POINTS = 34;
const int STOP_POINTS[] = {1-1, 2-1, 3-1, 4-1, 5-1, 6-1, 7-1, 8-1, 9-1, 10-1, 11-1, 12-1, 13-1, 14-1, 15-1, 16-1, 17-1, 18-1, 19-1, 20-1, 21-1, 22-1, 23-1, 24-1, 25-1, 26-1, 27-1, 28-1, 29-1, 30-1, 31-1, 32-1, 33-1, 34-1};
// const int STOP_POINTS[] = {1, 123, 318, 331, 344, 467, 566, 973, 1096, 1227, 1500, 1535, 1546, 1571, 1990, 2021, 2034, 2077, 2540, 2671, 2794, 3351, 3412, 3555, 3698, 4051, 4260, 4425, 4532, 4615, 4776, 5421, 5708};
// const int STOP_POINTS[] = {1, 82, 212, 221, 230, 312, 378, 649, 731, 818, 1000, 1023, 1030, 1047, 1326, 1347, 1356, 1385, 1694, 1781, 1863, 2234, 2275, 2370, 2465, 2700, 2839, 2949, 3020, 3075, 3182, 3612, 3803};
// const int STOP_POINTS[] = {1, 62, 160, 167, 174, 236, 286, 490, 552, 618, 755, 773, 779, 792, 1002, 1018, 1025, 1047, 1279, 1345, 1407, 1686, 1717, 1789, 1861, 2038, 2143, 2226, 2280, 2322, 2403, 2726, 2870};
// const int STOP_POINTS[] = {1, 82, 212, 221, 230, 312, 378, 649, 731, 818, 1000, 1023, 1030, 1047, 1326, 1347, 1356, 1385, 1694, 1781, 1863, 2234, 2275, 2370, 2465, 2700, 2839, 2949, 3020, 3075, 3182, 3612, 3803};
//	{0, 100, 200, 300, 421}
//	{1-1, 3-1, 4-1, 6-1, 8-1, 10-1, 11-1, 12-1, 13-1, 14-1, 15-1, 16-1, 18-1}
#if DEBUG
uint8_t left_arm_flag = 0;
uint8_t right_arm_flag = 0;
double left_arm_angle[7] = {-0, -0, +0, -0, -0, -0, -0};
double right_arm_angle[7] = {+0, +0, -0, +0, +0, -0, -0};
double left_arm_angle_00[7] = {-0, -0, +0, -0, -0, -0, -0};
double right_arm_angle_00[7] = {+0, +0, -0, +0, +0, -0, -0};
double left_arm_angle_01[7] = {-40, -10, +45, -65, -90, -0, -40};
double right_arm_angle_01[7] = {+40, +10, -45, +65, +90, -0, -40};
#endif


#if WT901BC_TTL_CAN_TASK
// // 0 for success
// uint8_t proc_master_rx(uint32_t id, uint8_t *buff)
// {
// 	if (id == 0x50) // kunwei_ft frame 1   id == 0050
// 	{
// 		for(int i = 0; i < 8; i++)
// 		{
// 			WitSerialDataIn(buff[i]);
// 		}
// 		return 0;
// 	}
// 	else if (id == 0x51) // kunwei_ft frame 2
// 	{
// 		for(int i = 0; i < 8; i++)
// 		{
// 			WitSerialDataIn1(buff[i]);
// 		}
// 		return 0;
// 	}
// 	else if (id == 0x52) // pos or torque mode
// 	{
// 		for(int i = 0; i < 8; i++)
// 		{
// 			WitSerialDataIn2(buff[i]);
// 		}
// 		return 0;
// 	}
// 	else if (id == 0x53) // target angles
// 	{
// //		uart_printf("%d\r\n", 333);
// 		for(int i = 0; i < 8; i++)
// 		{
// 			WitSerialDataIn3(buff[i]);
// 		}
// 		return 0;
// 	}
// 	else if (id == 0x54) // target torques
// 	{
// //		uart_printf("%d\r\n", 888);
// 		for(int i = 0; i < 8; i++)
// 		{
// 			WitSerialDataIn8(buff[i]);
// 		}
// 		return 0;
// 	}
// 	else if (id == 0x55) // target torques
// 	{
// 		for(int i = 0; i < 8; i++)
// 		{
// 			WitSerialDataIn7(buff[i]);
// 		}
// 		return 0;
// 	}
// 	else if (id == 0x56) // target torques
// 	{
// //		uart_printf("%d\r\n", 666);
// 		for(int i = 0; i < 8; i++)
// 		{
// 			WitSerialDataIn6(buff[i]);
// 		}
// 		return 0;
// 	}
// 	else if (id == 0x57) // target torques
// 	{
// ////		uart_printf("%d\r\n", 777);
// //		for(int i = 0; i < 8; i++)
// //		{
// //			WitSerialDataIn7(buff[i]);
// //		}
// //		return 0;
// 	}
// 	else if (id == 0x58) // target torques
// 	{
// ////		uart_printf("%d\r\n", 888);
// //		for(int i = 0; i < 8; i++)
// //		{
// //			WitSerialDataIn8(buff[i]);
// //		}
// //		return 0;
// 	}
// 	else if (id == 0x59) // target torques
// 	{
// ////		uart_printf("%d\r\n", 999);
// //		for(int i = 0; i < 8; i++)
// //		{
// //			WitSerialDataIn8(buff[i]);
// //		}
// //		return 0;
// 	}
// 	return 1;
// }

// void extract_IMU_raw_data(int16_t *sReg, float *fAcc, float *fGyro, float *fAngle, float *fmag, float *fquat)
// {
// 	for(int i = 0; i < 3; i++)
// 	{
// 		fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
// 		fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
// 		fmag[i] = sReg[HX+i] / 1000.0f;
// 	}
// 	for(int i = 0; i < 4; i++)
// 	{
// 		fquat[i] = sReg[q0+i] / 32768.0f;
// 	}
// }

void CalibQuats_add(CalibQuats* input, Quaternion* q){
	if(input->count + 1 <= CALIB_NUM)
	{
		Quaternion_copy(q, &input->quats[input->count]);
		input->count++;
	}
}

void get_quat_GSR(CalibQuats* input, Quaternion* quat_GSR){
//	uart_printf("%d\r\n", input->count);
	Quaternion_average(input->quats, input->count, quat_GSR);
//	uart_printf("%0.3f\r\n", quat_GSR->w); // nan
}

void get_calib_param(Quaternion* quat_GSR, Quaternion* quat_BSR_inv){
	Quaternion quat_GBR, quat_GBR_inv, quat_BSR, result;
//	double e0[3] = {TO_RAD(180.0), 0, 0};
	double e0[3] = {0, 0, TO_RAD(180.0)};
	Quaternion_fromEulerZYX(e0, &quat_GBR);
	Quaternion_inverse(&quat_GBR, &quat_GBR_inv);
	Quaternion_multiply(&quat_GBR_inv, quat_GSR, &quat_BSR);
	Quaternion_inverse(&quat_BSR, &result);
	Quaternion_normalize(&result, quat_BSR_inv);
}

void imu_calib(Quaternion* q, Quaternion* quat_BSR_inv, Quaternion* output){
	Quaternion quat_GBR;
	Quaternion_multiply(q, quat_BSR_inv, &quat_GBR);
	Quaternion_normalize(&quat_GBR, output);
}

void calib_IMU_param(Quaternion* q, Quaternion* p, Quaternion* output)
{
	Quaternion result, q_inv;
	Quaternion_inverse(q, &q_inv);
	Quaternion_multiply(&q_inv, p, &result);
	Quaternion_normalize(&result, output);
}

void calib_correct(Quaternion* q, Quaternion* calib_param, Quaternion* output)
{
	Quaternion result;
	Quaternion_multiply(q, calib_param, &result);
	Quaternion_normalize(&result, output);
}

void body_correct(double body_eulur_param[3], Quaternion* quat_calib, Quaternion* output)
{
	Quaternion q, result;
	// std::cout << body_eulur_param[0] << " " << body_eulur_param[1] << " " << body_eulur_param[2] << std::endl;
	Quaternion_fromEulerZYX(body_eulur_param, &q);
	// std::cout << q.w << " " << q.v[0] << " " << q.v[1] << " " << q.v[2] << std::endl;
	Quaternion_multiply(&q, quat_calib, &result);
	Quaternion_normalize(&result, output);
}

void ENU2NWU(Quaternion* q, Quaternion* output){
	Quaternion quat, result1, result2;
	double e0[3] = {TO_RAD(-90.0), 0, 0};
	Quaternion_fromEulerZYX(e0, &quat);
	// std::cout << quat.w << " " << quat.v[0] << " " << quat.v[1] << " " << quat.v[2] << std::endl;
	Quaternion_multiply(&quat, q, &result1);
	Quaternion_normalize(&result1, &result2);
	Quaternion_copy(&result2, output);
}

void get_relative_quat(Quaternion* q, Quaternion* p, Quaternion* output){
	Quaternion quat_imu_qp, result, q_inv;
	Quaternion_inverse(q, &q_inv);
	Quaternion_multiply(&q_inv, p, &result);
	Quaternion_normalize(&result, &quat_imu_qp);
	Quaternion_copy(&quat_imu_qp, output);
}

void right_arm_get_joint_angle(Quaternion* quat_imu_04, Quaternion* quat_imu_45, Quaternion* quat_imu_56, double output[7]){
	double euler_04[3], euler_45[3], euler_56[3];
	Quaternion_toEulerZYX(quat_imu_04, euler_04); 	// stored in array as [x, y, z]
	Quaternion_toEulerZYX(quat_imu_45, euler_45);
	Quaternion_toEulerZYX(quat_imu_56, euler_56);
//	output[0] = euler_04[0]; 												// rad +90
//	output[1] = euler_04[2];
//	output[2] = euler_04[1];
//	output[3] = euler_45[0];
//	output[4] = euler_45[1];
//	output[5] = euler_56[0];
//	output[6] = euler_56[2];
	
	output[0] = euler_04[2]; 												// rad
	output[1] = euler_04[0];
	output[2] = euler_04[1];
	output[3] = euler_45[2];
	output[4] = euler_45[1];
	output[5] = euler_56[2];
	output[6] = euler_56[0];

	// std::cout << euler_45[0] << " " << euler_45[1] << " " << euler_45[2] << std::endl;
	
	for(int i = 0; i < 7; i++)											// deg
	{
		output[i] = TO_DEG(output[i]);
	}
}

void head_get_joint_angle(Quaternion* quat_imu_02){
	double euler_02[3], output[2];
	Quaternion_toEulerZYX(quat_imu_02, euler_02);
	output[0] = euler_02[1]; 												// rad
	output[1] = euler_02[0];
	for(int i = 0; i < 2; i++)											// deg
	{
		joint_angle[7 + i] = TO_DEG(output[i]);
	}
}

void waist_get_joint_angle(Quaternion* quat_imu_01){
	double euler_01[3], output[3];
	Quaternion_toEulerZYX(quat_imu_01, euler_01);
	output[0] = euler_01[0];												// rad
	output[1] = euler_01[1];
	output[2] = euler_01[2];
	for(int i = 0; i < 3; i++)											// deg
	{
		joint_angle[9 + i] = TO_DEG(output[i]);
	}
}

void left_arm_get_joint_angle(Quaternion* quat_imu_07, Quaternion* quat_imu_78, Quaternion* quat_imu_89, double* output){
	double euler_07[3], euler_78[3], euler_89[3];
	Quaternion_toEulerZYX(quat_imu_07, euler_07); 	// stored in array as [x, y, z]
	Quaternion_toEulerZYX(quat_imu_78, euler_78);
	Quaternion_toEulerZYX(quat_imu_89, euler_89);
	*(output + 0) = euler_07[2]; 												// rad
	*(output + 1) = euler_07[0];
	*(output + 2) = euler_07[1];
	*(output + 3) = euler_78[2];
	*(output + 4) = euler_78[1];
	*(output + 5) = euler_89[2];
	*(output + 6) = euler_89[0];

	for(int i = 0; i < 7; i++)											// deg
	{
		*(output + i) = TO_DEG(*(output + i));
	}
}

double limit_angle(double angle, double min_angle, double max_angle) {
    if (angle < min_angle) {
        return min_angle;
    } else if (angle > max_angle) {
        return max_angle;
    } else {
        return angle;
    }
}

void right_arm_joint_trans_drive(double input[7], double output[7]){
	input[5] = limit_angle(input[5], -45, 45);

	for(int i = 0; i < 4; i++)
	{
		output[i] = input[i];
	}

	output[0] = output[0] + Track_Param.param[6];
	output[1] = output[1] - Track_Param.param[7];	// Zero position deviation of robotic arm
	output[2] = output[2] + (output[2] + Track_Param.param[3]) * Track_Param.param[2];
	output[3] = output[3] + Track_Param.param[4];

	output[0] = limit_angle(output[0], -20, +90);
	if(output[0] < 30 && output[0] > -30)
	{
		// if (output[3] < 65)
		// {
		// 	output[1] = limit_angle(output[1], 0, +90);
		// }
		output[1] = limit_angle(output[1], 0, +90);
	}
	// output[1] = limit_angle(output[1], -20, +90);

	if (std::isnan(input[5]))
	{
		input[5] = 0;
	}
	// q5 to theta5
	// input[4] = limit_angle(input[4], -90, 90);
	input[4] = -input[4];
	double a = TO_DEG(acos(sin(input[4] * M_PI / 180.0)));
	double b = TO_DEG(atan(sqrt((1 - 2 * pow(sin(input[5] * M_PI / 180.0), 2))) / sin(input[5] * M_PI / 180.0)));
	if (input[4] > 90 && input[4] <= 180) {
		a = -a;
    }
	a = a - 180.0;
	if (input[5] >= -90 && input[5] < 0) {
			b = b + 180.0;
	}
	output[4] = a + b;
	output[4] = output[4] * Track_Param.param[5] + (Track_Param.param[3] - Track_Param.param[8]);
	output[4] = limit_angle(output[4], -90, 90);
	output[4] = output[4] - 90; // a difference in the zero position between IMU calibration and robotic arm calibration
	
	// q6 to theta6
	// input[5] = limit_angle(input[5], -20, 30);
	output[5] = TO_DEG(asin(sin(input[5] * M_PI / 180.0) * sqrt(2)));
	output[5] = limit_angle(output[5], -45, 45);
	
	// q7 to theta7
	// input[6] = limit_angle(input[6], -80, 70);
	output[6] = TO_DEG(input[6] + (M_PI / 4.0 * 180.0 / M_PI) - atan(1.0 / sqrt(1 - 2 * pow(sin(input[5] * M_PI / 180.0), 2))));
	output[6] = limit_angle(output[6], -80, 70);
	
	// for(int i = 0; i < 7; i++)
	// {
	// 	output[i] = input[i];
	// }
}

void left_arm_joint_trans_drive(double* input, double* output){
	input[5] = limit_angle(input[5], -45, 45);

	for(int i = 0; i < 4; i++)
	{
		output[i] = input[i];
	}

	// output[0] = output[0] - Track_Param.param[6];
	// output[1] = output[1] - Track_Param.param[7];	// Zero position deviation of robotic arm
	// output[2] = output[2] * Track_Param.param[8] + Track_Param.param[9];
	// output[3] = output[3] - Track_Param.param[10];

	output[0] = output[0] - Track_Param.param[15];
	output[1] = output[1] - Track_Param.param[16];	// Zero position deviation of robotic arm
	output[2] = output[2] + (output[2] - Track_Param.param[12]) * Track_Param.param[11];
	output[3] = output[3] - Track_Param.param[13];

	output[0] = limit_angle(output[0], -90, +20);

	if(output[0] < 30 && output[0] > -30)
	{
		// if (output[3] > -57)
		// {
		// 	output[1] = limit_angle(output[1], 0, 90);
		// }
		output[1] = limit_angle(output[1], 0, 90);
	}
	

	if (std::isnan(input[5]))
	{
		input[5] = 0;
	}
	// q5 to theta5
	// input[4] = limit_angle(input[4], -90, 90);
	double a = TO_DEG(acos(sin(input[4] * M_PI / 180.0)));
	double b = TO_DEG(atan(sqrt((1 - 2 * pow(sin(input[5] * M_PI / 180.0), 2))) / sin(input[5] * M_PI / 180.0)));
	if (input[4] > 90 && input[4] <= 180) {
		a = -a;
    }
	a = a - 180.0;
	if (input[5] >= -90 && input[5] < 0) {
			b = b + 180.0;
	}
	output[4] = a + b;
	// output[4] = output[4] * (-1);
	output[4] = output[4] * Track_Param.param[14] + (Track_Param.param[12] - Track_Param.param[17]);
	output[4] = limit_angle(output[4], -90, 90);
	output[4] = output[4] - 90; // a difference in the zero position between IMU calibration and robotic arm calibration
	
	// q6 to theta6
	output[5] = TO_DEG(asin(sin(input[5] * M_PI / 180.0) * sqrt(2)));
	output[5] = limit_angle(output[5], -45, 45);
	
	// q7 to theta7
	output[6] = TO_DEG(input[6] + (M_PI / 4.0 * 180.0 / M_PI) - atan(1.0 / sqrt(1 - 2 * pow(sin(input[5] * M_PI / 180.0), 2))));
	output[6] = limit_angle(output[6], -80, 70);
	
	
	// for(int i = 0; i < 7; i++)
	// {
	// 	output[i] = input[i];
	// }
}


void calibration(void){
	#if BENCHMARK_ENABLE
	// extract_IMU_raw_data(sReg3, fAcc3, fGyro3, fAngle3, fmag3, fquat3);
	// if(fquat3[0] < (float)QUATERNION_EPS && fquat3[1] < (float)QUATERNION_EPS && fquat3[2] < (float)QUATERNION_EPS && fquat3[3] < (float)QUATERNION_EPS)
	// {
	// 	return ;
	// }
	Quaternion quat3_ENU;
	Quaternion_set(fquat3[0], fquat3[1], fquat3[2], fquat3[3], &quat3_ENU);
	
//	Quaternion quat3_ENU_sync;
//	sync_correct(&quat3_ENU, &quat_sync_IMU23, &quat3_ENU_sync);
	
	Quaternion quat3_NWU;
	ENU2NWU(&quat3_ENU, &quat3_NWU);
//	ENU2NWU(&quat3_ENU_sync, &quat3_NWU);
	
	
	Quaternion quat3_GSR;
	if(quats_3.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat3_NWU.w) && !std::isnan(quat3_NWU.v[0]) && !std::isnan(quat3_NWU.v[1]) && !std::isnan(quat3_NWU.v[2]))
		{
			CalibQuats_add(&quats_3, &quat3_NWU);
		}
		// CalibQuats_add(&quats_3, &quat3_NWU);
	}
	else if(quats_3.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_3, &quat3_GSR);
//		get_calib_param(&quat3_GSR, &quat_BSR_inv_3);
	}
	#endif
	
	#if RIGHT_ARM_ENABLE
	// extract_IMU_raw_data(sReg, fAcc0, fGyro0, fAngle0, fmag0, fquat0);
	// extract_IMU_raw_data(sReg1, fAcc1, fGyro1, fAngle1, fmag1, fquat1);
	// extract_IMU_raw_data(sReg2, fAcc2, fGyro2, fAngle2, fmag2, fquat2);
//	uart_printf("%0.3f, %0.3f, %0.3f, %0.3f\r\n", fquat2[0], fquat2[1], fquat2[2], fquat2[3]);
//	if(fquat0[0] < (float)QUATERNION_EPS && fquat0[1] < (float)QUATERNION_EPS && fquat0[2] < (float)QUATERNION_EPS && fquat0[3] < (float)QUATERNION_EPS)
//	{
//		return ;
//	}

	Quaternion quat0_ENU, quat1_ENU, quat2_ENU;
	Quaternion_set(fquat0[0], fquat0[1], fquat0[2], fquat0[3], &quat0_ENU);
	Quaternion_set(fquat1[0], fquat1[1], fquat1[2], fquat1[3], &quat1_ENU);
	Quaternion_set(fquat2[0], fquat2[1], fquat2[2], fquat2[3], &quat2_ENU);
	
//	Quaternion quat2_ENU_sync;
//	sync_correct(&quat_sync_IMU23, &quat2_ENU, &quat2_ENU_sync);
	
	Quaternion quat0_NWU, quat1_NWU, quat2_NWU;
	ENU2NWU(&quat0_ENU, &quat0_NWU);
	ENU2NWU(&quat1_ENU, &quat1_NWU);
	ENU2NWU(&quat2_ENU, &quat2_NWU);
//	ENU2NWU(&quat2_ENU_sync, &quat2_NWU);
	
	Quaternion quat0_GSR, quat1_GSR, quat2_GSR;
	if(quats_2.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat2_NWU.w) && !std::isnan(quat2_NWU.v[0]) && !std::isnan(quat2_NWU.v[1]) && !std::isnan(quat2_NWU.v[2]))
		{
			CalibQuats_add(&quats_2, &quat2_NWU);
		}
	}
	else if(quats_2.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_2, &quat2_GSR);
		calib_IMU_param(&quat3_GSR, &quat2_GSR, &quat_calib_IMU23);
//		get_calib_param(&quat2_GSR, &quat_BSR_inv_2);
	}

	if(quats_1.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat1_NWU.w) && !std::isnan(quat1_NWU.v[0]) && !std::isnan(quat1_NWU.v[1]) && !std::isnan(quat1_NWU.v[2]))
		{
			CalibQuats_add(&quats_1, &quat1_NWU);
		}
	}
	else if(quats_1.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_1, &quat1_GSR);
		calib_IMU_param(&quat2_GSR, &quat1_GSR, &quat_calib_IMU12);
//		get_calib_param(&quat1_GSR, &quat_BSR_inv_1);
	}
	
	if(quats_0.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat0_NWU.w) && !std::isnan(quat0_NWU.v[0]) && !std::isnan(quat0_NWU.v[1]) && !std::isnan(quat0_NWU.v[2]))
		{
			CalibQuats_add(&quats_0, &quat0_NWU);
		}
	}
	else if(quats_0.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_0, &quat0_GSR);
		calib_IMU_param(&quat1_GSR, &quat0_GSR, &quat_calib_IMU01);
//		get_calib_param(&quat0_GSR, &quat_BSR_inv_0);
	}
	#endif
	
	#if HEAD_ENABLE
	// extract_IMU_raw_data(sReg4, fAcc4, fGyro4, fAngle4, fmag4, fquat4);
//	if(fquat4[0] < (float)QUATERNION_EPS && fquat4[1] < (float)QUATERNION_EPS && fquat4[2] < (float)QUATERNION_EPS && fquat4[3] < (float)QUATERNION_EPS)
//	{
//		return ;
//	}
	Quaternion quat4_ENU;
	Quaternion_set(fquat4[0], fquat4[1], fquat4[2], fquat4[3], &quat4_ENU);
	Quaternion quat4_NWU;
	ENU2NWU(&quat4_ENU, &quat4_NWU);
	Quaternion quat4_GSR;
	if(quats_4.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat4_NWU.w) && !std::isnan(quat4_NWU.v[0]) && !std::isnan(quat4_NWU.v[1]) && !std::isnan(quat4_NWU.v[2]))
		{
			CalibQuats_add(&quats_4, &quat4_NWU);
		}
	}
	else if(quats_4.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_4, &quat4_GSR);
		calib_IMU_param(&quat3_GSR, &quat4_GSR, &quat_calib_IMU43);
//		get_calib_param(&quat4_GSR, &quat_BSR_inv_4);
	}
	#endif
	
	#if WAIST_ENABLE
	extract_IMU_raw_data(sReg5, fAcc5, fGyro5, fAngle5, fmag5, fquat5);
	if(fquat5[0] < (float)QUATERNION_EPS && fquat5[1] < (float)QUATERNION_EPS && fquat5[2] < (float)QUATERNION_EPS && fquat5[3] < (float)QUATERNION_EPS)
	{
		return ;
	}
	Quaternion quat5_ENU;
	Quaternion_set(fquat5[0], fquat5[1], fquat5[2], fquat5[3], &quat5_ENU);
	Quaternion quat5_NWU;
	ENU2NWU(&quat5_ENU, &quat5_NWU);
	Quaternion quat5_GSR;
	if(quats_5.count + 1 <= CALIB_NUM)
	{
		CalibQuats_add(&quats_5, &quat5_NWU);
	}
	else if(quats_5.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_5, &quat5_GSR);
		calib_IMU_param(&quat3_GSR, &quat5_GSR, &quat_calib_IMU53);
//		get_calib_param(&quat5_GSR, &quat_BSR_inv_5);
	}
	#endif
	
	#if LEFT_ARM_ENABLE
	// extract_IMU_raw_data(sReg6, fAcc6, fGyro6, fAngle6, fmag6, fquat6);
	// extract_IMU_raw_data(sReg7, fAcc7, fGyro7, fAngle7, fmag7, fquat7);
	// extract_IMU_raw_data(sReg8, fAcc8, fGyro8, fAngle8, fmag8, fquat8);
//	if(fquat6[0] < (float)QUATERNION_EPS && fquat6[1] < (float)QUATERNION_EPS && fquat6[2] < (float)QUATERNION_EPS && fquat6[3] < (float)QUATERNION_EPS)
//	{
//		return ;
//	}
//	uart_printf("%0.3f, %0.3f, %0.3f, %0.3f\r\n", fquat7[0], fquat7[1], fquat7[2], fquat7[3]);
//	uart_printf("%0.3f, %0.3f, %0.3f\r\n", fquat6[0], fquat7[0], fquat8[0]);

	Quaternion quat6_ENU, quat7_ENU, quat8_ENU;
	Quaternion_set(fquat6[0], fquat6[1], fquat6[2], fquat6[3], &quat6_ENU);
	Quaternion_set(fquat7[0], fquat7[1], fquat7[2], fquat7[3], &quat7_ENU);
	Quaternion_set(fquat8[0], fquat8[1], fquat8[2], fquat8[3], &quat8_ENU);
	
	Quaternion quat6_NWU, quat7_NWU, quat8_NWU;
	ENU2NWU(&quat6_ENU, &quat6_NWU);
	ENU2NWU(&quat7_ENU, &quat7_NWU);
	ENU2NWU(&quat8_ENU, &quat8_NWU);
	
	Quaternion quat6_GSR, quat7_GSR, quat8_GSR;
	if(quats_8.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat8_NWU.w) && !std::isnan(quat8_NWU.v[0]) && !std::isnan(quat8_NWU.v[1]) && !std::isnan(quat8_NWU.v[2]))
		{
			CalibQuats_add(&quats_8, &quat8_NWU);
		}
	}
	else if(quats_8.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_8, &quat8_GSR);
		calib_IMU_param(&quat3_GSR, &quat8_GSR, &quat_calib_IMU83);
//		get_calib_param(&quat8_GSR, &quat_BSR_inv_8);
	}
	
	if(quats_7.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat7_NWU.w) && !std::isnan(quat7_NWU.v[0]) && !std::isnan(quat7_NWU.v[1]) && !std::isnan(quat7_NWU.v[2]))
		{
			CalibQuats_add(&quats_7, &quat7_NWU);
		}
	}
	else if(quats_7.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_7, &quat7_GSR);
		calib_IMU_param(&quat8_GSR, &quat7_GSR, &quat_calib_IMU78);
//		get_calib_param(&quat7_GSR, &quat_BSR_inv_7);
	}
	
	if(quats_6.count + 1 <= CALIB_NUM)
	{
		if (!std::isnan(quat6_NWU.w) && !std::isnan(quat6_NWU.v[0]) && !std::isnan(quat6_NWU.v[1]) && !std::isnan(quat6_NWU.v[2]))
		{
			CalibQuats_add(&quats_6, &quat6_NWU);
		}
	}
	else if(quats_6.count == CALIB_NUM)
	{
		get_quat_GSR(&quats_6, &quat6_GSR);
		calib_IMU_param(&quat7_GSR, &quat6_GSR, &quat_calib_IMU67);
//		get_calib_param(&quat6_GSR, &quat_BSR_inv_6);
	}
	#endif
}

void right_arm_robot_motor_angle_trans(void)
{
	motor_angle[0] = motor_angle[0];					// (-20,90)---(-20,90)
	motor_angle[1] = motor_angle[1] * (-1);					// (-90,25)---(-90,25)
	motor_angle[2] = motor_angle[2];
	motor_angle[3] = motor_angle[3] * (-1);
	motor_angle[4] = motor_angle[4];
	motor_angle[5] = motor_angle[5] * (-1);  
	motor_angle[6] = motor_angle[6] * (-1);
}

void head_robot_motor_angle_trans(void)
{
	motor_angle[7] = joint_angle[7] * (-1);
	motor_angle[8] = joint_angle[8];
}

void waist_robot_motor_angle_trans(void)
{
	motor_angle[9] = joint_angle[9];
	motor_angle[10] = joint_angle[10];
	motor_angle[11] = joint_angle[11];
}

void left_arm_robot_motor_angle_trans(void)
{
	motor_angle[12] = motor_angle[12];		// (-20,90)---(-90,20)
	motor_angle[13] = motor_angle[13];					// (-25,90)---(-25,90)
	motor_angle[14] = motor_angle[14];
	motor_angle[15] = motor_angle[15] * (-1);
	motor_angle[16] = motor_angle[16] * (-1);
	motor_angle[17] = motor_angle[17] * (-1);  
	motor_angle[18] = motor_angle[18] * (-1);
}

void get_motor_angle(void){
	#if BENCHMARK_ENABLE
	// extract_IMU_raw_data(sReg3, fAcc3, fGyro3, fAngle3, fmag3, fquat3);
	Quaternion quat3_ENU;
	Quaternion_set(fquat3[0], fquat3[1], fquat3[2], fquat3[3], &quat3_ENU);
	
	// std::cout << quat3_ENU.w << " " << quat3_ENU.v[0] << " " << quat3_ENU.v[1] << " " << quat3_ENU.v[2] << std::endl;

//	Quaternion quat3_ENU_sync;
//	sync_correct(&quat3_ENU, &quat_sync_IMU23, &quat3_ENU_sync);
	
	Quaternion quat3_NWU;
	ENU2NWU(&quat3_ENU, &quat3_NWU);
//	ENU2NWU(&quat3_ENU_sync, &quat3_NWU);
	
//	Quaternion quat3_GBR;
////	Quaternion_set(1, 0, 0, 0, &quat_BSR_inv_3);
////	Quaternion_set(0.620, -0.627, -0.328, 0.338, &quat_BSR_inv_3);
//	imu_calib(&quat3_NWU, &quat_BSR_inv_3, &quat3_GBR);
////	imu_calib(&quat_BSR_inv_3, &quat3_NWU, &quat3_GBR);
	#endif
	
	
	#if RIGHT_ARM_ENABLE
	// extract_IMU_raw_data(sReg, fAcc0, fGyro0, fAngle0, fmag0, fquat0);
	// extract_IMU_raw_data(sReg1, fAcc1, fGyro1, fAngle1, fmag1, fquat1);
	// extract_IMU_raw_data(sReg2, fAcc2, fGyro2, fAngle2, fmag2, fquat2);
	
	Quaternion quat0_ENU, quat1_ENU, quat2_ENU;
	Quaternion_set(fquat0[0], fquat0[1], fquat0[2], fquat0[3], &quat0_ENU);
	Quaternion_set(fquat1[0], fquat1[1], fquat1[2], fquat1[3], &quat1_ENU);
	Quaternion_set(fquat2[0], fquat2[1], fquat2[2], fquat2[3], &quat2_ENU);
	
//	Quaternion quat2_ENU_sync;
//	sync_correct(&quat_sync_IMU23, &quat2_ENU, &quat2_ENU_sync);
	
	Quaternion quat0_NWU, quat1_NWU, quat2_NWU;
	ENU2NWU(&quat0_ENU, &quat0_NWU);
	ENU2NWU(&quat1_ENU, &quat1_NWU);
	ENU2NWU(&quat2_ENU, &quat2_NWU);
//	ENU2NWU(&quat2_ENU_sync, &quat2_NWU);
	
//	Quaternion quat0_GBR, quat1_GBR, quat2_GBR;
//	imu_calib(&quat0_NWU, &quat_BSR_inv_0, &quat0_GBR);
//	imu_calib(&quat1_NWU, &quat_BSR_inv_1, &quat1_GBR);
////	Quaternion_set(0.7071068, 0, -0.7071068, 0, &quat_BSR_inv_2);
////	Quaternion_set(0.614, -0.620, -0.353, 0.338, &quat_BSR_inv_2);
//	imu_calib(&quat2_NWU, &quat_BSR_inv_2, &quat2_GBR);
////	imu_calib(&quat_BSR_inv_2, &quat2_NWU, &quat2_GBR);

	Quaternion quat3_ENU_calib_right, quat2_ENU_calib, quat1_ENU_calib;
	calib_correct(&quat3_NWU, &quat_calib_IMU23, &quat3_ENU_calib_right);
	calib_correct(&quat2_NWU, &quat_calib_IMU12, &quat2_ENU_calib);
	calib_correct(&quat1_NWU, &quat_calib_IMU01, &quat1_ENU_calib);

	Quaternion quat_imu_04, quat_imu_45, quat_imu_56;
	get_relative_quat(&quat3_ENU_calib_right, &quat2_NWU, &quat_imu_04);
	get_relative_quat(&quat2_ENU_calib, &quat1_NWU, &quat_imu_45);
	get_relative_quat(&quat1_ENU_calib, &quat0_NWU, &quat_imu_56);
	
//	Quaternion quat_imu_04, quat_imu_45, quat_imu_56;
//	get_relative_quat(&quat3_NWU, &quat2_NWU, &quat_imu_04);
//	get_relative_quat(&quat2_NWU, &quat1_NWU, &quat_imu_45);
//	get_relative_quat(&quat1_NWU, &quat0_NWU, &quat_imu_56);

	double euler_B2S2[3] = {TO_RAD(+Track_Param.param[1]), TO_RAD(-Track_Param.param[3]), TO_RAD(+Track_Param.param[0])};
	Quaternion quat_body_04;
	body_correct(euler_B2S2, &quat_imu_04, &quat_body_04);
	
	// right_arm_get_joint_angle(&quat_imu_04, &quat_imu_45, &quat_imu_56, joint_angle);
	right_arm_get_joint_angle(&quat_body_04, &quat_imu_45, &quat_imu_56, joint_angle);
	
	right_arm_joint_trans_drive(joint_angle, motor_angle);
	
	#if DEBUG
	for(int i = 0; i < 7; i++)
	{
		motor_angle[i] = right_arm_angle[i];
	}
	#endif
	
	right_arm_robot_motor_angle_trans();
	#endif
	
	#if HEAD_ENABLE
	// extract_IMU_raw_data(sReg4, fAcc4, fGyro4, fAngle4, fmag4, fquat4);
	Quaternion quat4_ENU;
	Quaternion_set(fquat4[0], fquat4[1], fquat4[2], fquat4[3], &quat4_ENU);
	Quaternion quat4_NWU;
	ENU2NWU(&quat4_ENU, &quat4_NWU);
//	Quaternion quat4_GBR;
//	imu_calib(&quat4_NWU, &quat_BSR_inv_4, &quat4_GBR);
	Quaternion quat3_ENU_calib_head;
	calib_correct(&quat3_NWU, &quat_calib_IMU43, &quat3_ENU_calib_head);
	Quaternion quat_imu_02;
	get_relative_quat(&quat3_ENU_calib_head, &quat4_NWU, &quat_imu_02);
	head_get_joint_angle(&quat_imu_02);
	head_robot_motor_angle_trans();
	#endif
	
	#if WAIST_ENABLE
	extract_IMU_raw_data(sReg5, fAcc5, fGyro5, fAngle5, fmag5, fquat5);
	Quaternion quat5_ENU;
	Quaternion_set(fquat5[0], fquat5[1], fquat5[2], fquat5[3], &quat5_ENU);
	Quaternion quat5_NWU;
	ENU2NWU(&quat5_ENU, &quat5_NWU);
//	Quaternion quat5_GBR;
//	imu_calib(&quat5_NWU, &quat_BSR_inv_5, &quat5_GBR);
	Quaternion quat3_ENU_calib_waist;
	calib_correct(&quat3_NWU, &quat_calib_IMU53, &quat3_ENU_calib_waist);
	Quaternion quat_imu_01;
	get_relative_quat(&quat3_ENU_calib_waist, &quat5_NWU, &quat_imu_01);
	waist_get_joint_angle(&quat_imu_01);
	waist_robot_motor_angle_trans();
	#endif
	
	#if LEFT_ARM_ENABLE
	// extract_IMU_raw_data(sReg6, fAcc6, fGyro6, fAngle6, fmag6, fquat6);
	// extract_IMU_raw_data(sReg7, fAcc7, fGyro7, fAngle7, fmag7, fquat7);
	// extract_IMU_raw_data(sReg8, fAcc8, fGyro8, fAngle8, fmag8, fquat8);
	//	uart_printf("%0.3f, %0.3f, %0.3f, %0.3f\r\n", fquat8[0], fquat8[1], fquat8[2], fquat8[3]);
	//	uart_printf("%0.3f, %0.3f, %0.3f\r\n", fquat6[0], fquat7[0], fquat8[0]);
	
	Quaternion quat6_ENU, quat7_ENU, quat8_ENU;
	Quaternion_set(fquat6[0], fquat6[1], fquat6[2], fquat6[3], &quat6_ENU);
	Quaternion_set(fquat7[0], fquat7[1], fquat7[2], fquat7[3], &quat7_ENU);
	Quaternion_set(fquat8[0], fquat8[1], fquat8[2], fquat8[3], &quat8_ENU);
	
	Quaternion quat6_NWU, quat7_NWU, quat8_NWU;
	ENU2NWU(&quat6_ENU, &quat6_NWU);
	ENU2NWU(&quat7_ENU, &quat7_NWU);
	ENU2NWU(&quat8_ENU, &quat8_NWU);
	
//	Quaternion quat6_GBR, quat7_GBR, quat8_GBR;
//	imu_calib(&quat6_NWU, &quat_BSR_inv_6, &quat6_GBR);
//	imu_calib(&quat7_NWU, &quat_BSR_inv_7, &quat7_GBR);
//	imu_calib(&quat8_NWU, &quat_BSR_inv_8, &quat8_GBR);

	Quaternion quat3_ENU_calib_left, quat8_ENU_calib, quat7_ENU_calib;
	calib_correct(&quat3_NWU, &quat_calib_IMU83, &quat3_ENU_calib_left);
	calib_correct(&quat8_NWU, &quat_calib_IMU78, &quat8_ENU_calib);
	calib_correct(&quat7_NWU, &quat_calib_IMU67, &quat7_ENU_calib);

	Quaternion quat_imu_07, quat_imu_78, quat_imu_89;
	get_relative_quat(&quat3_ENU_calib_left, &quat8_NWU, &quat_imu_07);
	get_relative_quat(&quat8_ENU_calib, &quat7_NWU, &quat_imu_78);
	get_relative_quat(&quat7_ENU_calib, &quat6_NWU, &quat_imu_89);

	// Quaternion quat_imu_07, quat_imu_78, quat_imu_89;
	// get_relative_quat(&quat3_NWU, &quat8_NWU, &quat_imu_07);
	// get_relative_quat(&quat8_NWU, &quat7_NWU, &quat_imu_78);
	// get_relative_quat(&quat7_NWU, &quat6_NWU, &quat_imu_89);

	double euler_B8S8[3] = {TO_RAD(+Track_Param.param[10]), TO_RAD(+Track_Param.param[12]), TO_RAD(+Track_Param.param[9])};
	Quaternion quat_body_07;
	body_correct(euler_B8S8, &quat_imu_07, &quat_body_07);
	
	// left_arm_get_joint_angle(&quat_imu_07, &quat_imu_78, &quat_imu_89, &joint_angle[12]);
	left_arm_get_joint_angle(&quat_body_07, &quat_imu_78, &quat_imu_89, &joint_angle[12]);
	
	left_arm_joint_trans_drive(&joint_angle[12], &motor_angle[12]);
	
	#if DEBUG
	for(int i = 0; i < 7; i++)
	{
		motor_angle[12 + i] = left_arm_angle[i];
	}
	#endif
	
	left_arm_robot_motor_angle_trans();
	#endif
}


int isQuaternionEmpty(Quaternion* q){
	return (q->w == 0.0 && q->v[0] == 0.0 && q->v[1] == 0.0 && q->v[2] == 0.0);
}

bool isCalibParamEmpty(void){
	bool result = 1;

	#if BENCHMARK_ENABLE && RIGHT_ARM_ENABLE
	result = result && (bool)(isQuaternionEmpty(&quat_calib_IMU23) && isQuaternionEmpty(&quat_calib_IMU12)  && isQuaternionEmpty(&quat_calib_IMU01));
	// result = result && (bool)isQuaternionEmpty(&quat_calib_IMU12) && (bool)isQuaternionEmpty(&quat_calib_IMU01);
	// result = result && (bool)isQuaternionEmpty(&quat_calib_IMU12);
	#endif
	#if BENCHMARK_ENABLE && LEFT_ARM_ENABLE
	result = result && (bool)(isQuaternionEmpty(&quat_calib_IMU83) && isQuaternionEmpty(&quat_calib_IMU78)  && isQuaternionEmpty(&quat_calib_IMU67));
	// result = result && (bool)(isQuaternionEmpty(&quat_calib_IMU83) && isQuaternionEmpty(&quat_calib_IMU78));
	// result = result && (bool)(isQuaternionEmpty(&quat_calib_IMU78));
	#endif
	#if BENCHMARK_ENABLE && HEAD_ENABLE
	result = result && (bool)(isQuaternionEmpty(&quat_calib_IMU43));
	#endif
	#if BENCHMARK_ENABLE && WAIST_ENABLE
	result = result && (bool)(isQuaternionEmpty(&quat_calib_IMU53));
	#endif
	return result;
}

void CalibParamReset(void){
	#if BENCHMARK_ENABLE && HEAD_ENABLE
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU43);
	quats_3.count = 0;
	quats_4.count = 0;
	#endif
	#if BENCHMARK_ENABLE && RIGHT_ARM_ENABLE
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU23);
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU12);
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU01);
	quats_3.count = 0;
	quats_2.count = 0;
	quats_1.count = 0;
	quats_0.count = 0;
	#endif
	#if BENCHMARK_ENABLE && LEFT_ARM_ENABLE
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU83);
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU78);
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU67);
	quats_3.count = 0;
	quats_8.count = 0;
	quats_7.count = 0;
	quats_6.count = 0;
	#endif
	#if BENCHMARK_ENABLE && WAIST_ENABLE
	Quaternion_set(0, 0, 0, 0, &quat_calib_IMU53);
	quats_3.count = 0;
	quats_5.count = 0;
	#endif
}

void movmean_init(void){
	for(int i = 0; i < 19; i++)																			// Initialize filter
	{
		initMovingAverageFilter(&filter_instance[i], WINDOW_SIZE);
	}
}

void movmean(void){
	#if RIGHT_ARM_ENABLE
	for(int i = 0; i < 7; i++)																			// Add new data points to the filter
	{
		if (!std::isnan(motor_angle[i]))
		{
			enqueue(&filter_instance[i], motor_angle[i]);
		}
	}
	for(int i = 0; i < 7; i++)																			// Obtain the mean result
	{
		movmean_motor_angle[i] = getMovingAverage(&filter_instance[i]);
	}	
	#endif
	#if HEAD_ENABLE
	for(int i = 0; i < 2; i++)																			// Add new data points to the filter
	{
		if (!(std::isnan(motor_angle[7]) || std::isnan(motor_angle[8])))
		{
			enqueue(&filter_instance[7 + i], motor_angle[7 + i]);
		}
	}
	for(int i = 0; i < 2; i++)																			// Obtain the mean result
	{
		movmean_motor_angle[7 + i] = getMovingAverage(&filter_instance[7 + i]);
	}
	// std::cout << movmean_motor_angle[7] << " " << movmean_motor_angle[8] << std::endl;
	#endif
	#if WAIST_ENABLE
	for(int i = 0; i < 3; i++)																			// Add new data points to the filter
	{
		enqueue(&filter_instance[9 + i], motor_angle[9 + i]);
	}
	for(int i = 0; i < 3; i++)																			// Obtain the mean result
	{
		movmean_motor_angle[9 + i] = getMovingAverage(&filter_instance[9 + i]);
	}
	#endif
	#if LEFT_ARM_ENABLE
	for(int i = 0; i < 7; i++)																			// Add new data points to the filter
	{
		if (!std::isnan(motor_angle[i + 12]))
		{
			enqueue(&filter_instance[12 + i], motor_angle[12 + i]);
		}
	}
	for(int i = 0; i < 7; i++)																			// Obtain the mean result
	{
		movmean_motor_angle[12 + i] = getMovingAverage(&filter_instance[12 + i]);
	}	
	#endif
}

void encode_can_message(int can_id, uint8_t motor_id, double angle, uint16_t max_speed, CanTxMessage* can_tx_message)
{
	// can_tx_message->can_id = __REV(ARM_MODULE_ID);

	can_tx_message->can_id = can_id;
	
	can_tx_message->motor_id = motor_id;
	memcpy(&can_tx_message->data[0], &can_tx_message->motor_id, sizeof(uint8_t));
	
	can_tx_message->angle = (int32_t)angle * 3600;
//	uart_printf("%d\r\n", can_tx_message->angle);
	memcpy(&can_tx_message->data[1], &can_tx_message->angle, sizeof(int32_t));
//	uart_printf("%d, %d, %d, %d\r\n", can_tx_message->data[1], can_tx_message->data[2], can_tx_message->data[3], can_tx_message->data[4]);
	
	can_tx_message->max_speed = (uint16_t)max_speed * 60;
	memcpy(&can_tx_message->data[5], &can_tx_message->max_speed, sizeof(uint16_t));
	
	memset(&can_tx_message->data[7], 0, sizeof(can_tx_message->data[7]));
}


unsigned char Sum_cheak(unsigned char* data)
{
  unsigned char cheak = 0x00;
  for(int i = 0; i < data[1] - 1; i++)
	{
		cheak += data[i];
	}
	return cheak;
}

void encode_com_message(CanTxMessage* can_tx_message, ComTxMessage* com_tx_message)
{
	com_tx_message->Head = COM_HEAD;
	com_tx_message->Len = COM_LEN;
	
	memcpy(&com_tx_message->CANData[0], &can_tx_message->can_id, sizeof(uint32_t));
	memcpy(&com_tx_message->CANData[4], &can_tx_message->data, CAN_DATA_LEN);
	
	com_tx_message->FrameID = COM_FRAME_ID;
	
	memcpy(&com_tx_message->data[0], &com_tx_message->Head, sizeof(uint8_t));
	memcpy(&com_tx_message->data[1], &com_tx_message->Len, sizeof(uint8_t));
	memcpy(&com_tx_message->data[2], &com_tx_message->CANData, (sizeof(uint32_t)+CAN_DATA_LEN));
	memcpy(&com_tx_message->data[14], &com_tx_message->FrameID, sizeof(uint8_t));
	
	uint8_t checksum = Sum_cheak(com_tx_message->data);
	com_tx_message->SumCheak = checksum;
	memcpy(&com_tx_message->data[15], &com_tx_message->SumCheak, sizeof(uint8_t));
}



#endif

}
