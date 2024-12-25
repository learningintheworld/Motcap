#ifndef __WIT_TASK_H
#define __WIT_TASK_H

//includes
// #include "stm32h7xx_hal.h"
#include "Quaternion.h"
#include "MovingAverageFilter.h"
#include <stdlib.h>
// #include "multi_button.h"

// using namespace Hexacercle;
// using YXWestimation::Quaternion;
// using namespace YXWestimation;
namespace YXWestimation {

//defines
#define WT901BC_TTL_CAN_TASK 1
#define CALIB_NUM 20
// #define ARM_MODULE_ID 0x02010112
// #define ZED_MODULE_ID 0x02010212
#define ARM_MODULE_ID 0x12010102 
#define ZED_MODULE_ID 0x12020102
#define COM_HEAD 0x55
#define COM_LEN 0x10
#define COM_FRAME_ID 0x00
#define CAN_DATA_LEN 8
#define IMU_TASK_ENABLE 1
#define RIGHT_ARM_ENABLE 0
#define LEFT_ARM_ENABLE 0
#define HEAD_ENABLE 0
#define WAIST_ENABLE 0
#define BENCHMARK_ENABLE 1
#define DATA_ENABLE 1
#define key1_id 0
#define DEBUG 0

typedef struct CalibQuats {
	YXWestimation::Quaternion quats[CALIB_NUM];
	int count;
} CalibQuats;

typedef struct MotionTrackParams {
	int param_id = 0;
	double param[18];
} MotionTrackParams;

typedef struct MotionTrackStatus {
	bool IsCalib;
	bool IsMotionTrack;
	bool IsMovmean;
	bool IsPrint;
} MotionTrackStatus;

// Define the data structure of the CAN protocol standard
typedef struct CanTxMessage {
	uint32_t can_id;
    uint8_t motor_id;      	// Unsigned int8
    int32_t angle;      		// Signed int32
    uint16_t max_speed; 		// Unsigned int16
	uint8_t data[CAN_DATA_LEN];
} CanTxMessage;

// Define the data structure of the Com protocol standard
typedef struct ComTxMessage {
	uint8_t Head;						// Fixed meter head 0x55
    uint8_t Len;      			// Com length
    uint8_t CANData[4 + CAN_DATA_LEN];    // CAN instruction data(4+8)
    uint8_t FrameID; 				// Fixed to 0
	uint8_t SumCheak;
	uint8_t data[8 + CAN_DATA_LEN];
} ComTxMessage;

//extern
extern float fAcc0[3], fGyro0[3], fAngle0[3], fmag0[3], fquat0[4];
extern float fAcc1[3], fGyro1[3], fAngle1[3], fmag1[3], fquat1[4];
extern float fAcc2[3], fGyro2[3], fAngle2[3], fmag2[3], fquat2[4];
extern float fAcc3[3], fGyro3[3], fAngle3[3], fmag3[3], fquat3[4];
extern float fAcc4[3], fGyro4[3], fAngle4[3], fmag4[3], fquat4[4];
extern float fAcc5[3], fGyro5[3], fAngle5[3], fmag5[3], fquat5[4];
extern float fAcc6[3], fGyro6[3], fAngle6[3], fmag6[3], fquat6[4];
extern float fAcc7[3], fGyro7[3], fAngle7[3], fmag7[3], fquat7[4];
extern float fAcc8[3], fGyro8[3], fAngle8[3], fmag8[3], fquat8[4];

extern char buffer[300];
extern uint8_t wit_can_buf[];
extern bool Tim2Flag;
extern volatile int32_t seconds;
extern volatile int32_t millis;
extern int32_t i;
extern CalibQuats quats_0, quats_1, quats_2, quats_3, quats_4, quats_5, quats_6, quats_7, quats_8;
extern YXWestimation::Quaternion quat_BSR_inv_0, quat_BSR_inv_1, quat_BSR_inv_2, quat_BSR_inv_3;
extern YXWestimation::Quaternion quat_BSR_inv_4, quat_BSR_inv_5, quat_BSR_inv_6, quat_BSR_inv_7, quat_BSR_inv_8;
extern YXWestimation::Quaternion quat_calib_IMU23, quat_calib_IMU12, quat_calib_IMU01;
extern YXWestimation::Quaternion quat_calib_IMU83, quat_calib_IMU78, quat_calib_IMU67;
extern YXWestimation::Quaternion quat_calib_IMU43;
extern YXWestimation::Quaternion quat_calib_IMU53;
extern double joint_angle[19];
extern double motor_angle[19];
extern double movmean_motor_angle[19];
extern struct Button key1;
//extern const int key1_id;
extern bool Tim3Flag;
extern MotionTrackParams Track_Param;
extern MotionTrackStatus IMU_Status;
extern CanTxMessage can_tx_message;
extern uint8_t head_motor_id[2];
extern uint8_t left_arm_motor_id[7];
extern uint8_t right_arm_motor_id[7];
extern uint8_t waist_motor_id[3];
extern uint16_t motor_max_speed;
extern ComTxMessage com_tx_message;
extern bool g_isPaused;
extern bool g_isReverse;
extern bool skipStopCheck; 
extern int g_currentRow;
extern const int NUM_STOP_POINTS;
extern const int STOP_POINTS[];
#if DEBUG
extern uint8_t left_arm_flag;
extern uint8_t right_arm_flag;
extern double left_arm_angle[7];
extern double right_arm_angle[7];
extern double left_arm_angle_00[7];
extern double right_arm_angle_00[7];
extern double left_arm_angle_01[7];
extern double right_arm_angle_01[7];
#endif

//functions
uint8_t proc_master_rx(uint32_t id, uint8_t *buff);
void extract_IMU_raw_data(int16_t *sReg, float *fAcc, float *fGyro, float *fAngle, float *fmag, float *fquat);

void CalibQuats_add(CalibQuats* input, YXWestimation::Quaternion* q);
void get_quat_GSR(CalibQuats* input, YXWestimation::Quaternion* quat_GSR);
void get_calib_param(YXWestimation::Quaternion* quat_GSR, YXWestimation::Quaternion* quat_BSR_inv);
void imu_calib(YXWestimation::Quaternion* q, YXWestimation::Quaternion* quat_BSR_inv, YXWestimation::Quaternion* output);
void calib_IMU_param(YXWestimation::Quaternion* q, YXWestimation::Quaternion* p, YXWestimation::Quaternion* output);
void calib_correct(YXWestimation::Quaternion* q, YXWestimation::Quaternion* calib_param, YXWestimation::Quaternion* output);
void body_correct(double body_eulur_param[3], YXWestimation::Quaternion* quat_calib, YXWestimation::Quaternion* output);

//void 

void ENU2NWU(YXWestimation::Quaternion* q, YXWestimation::Quaternion* output);
void get_relative_quat(YXWestimation::Quaternion* q, YXWestimation::Quaternion* p, YXWestimation::Quaternion* output);

void right_arm_get_joint_angle(YXWestimation::Quaternion* quat_imu_04, YXWestimation::Quaternion* quat_imu_45, YXWestimation::Quaternion* quat_imu_56, double output[7]);
void head_get_joint_angle(YXWestimation::Quaternion* quat_imu_02);
void waist_get_joint_angle(YXWestimation::Quaternion* quat_imu_01);
void left_arm_get_joint_angle(YXWestimation::Quaternion* quat_imu_07, YXWestimation::Quaternion* quat_imu_78, YXWestimation::Quaternion* quat_imu_89, double* output);

double limit_angle(double angle, double min_angle, double max_angle);
void right_arm_joint_trans_drive(double input[7], double output[7]);
void left_arm_joint_trans_drive(double* input, double* output);

void right_arm_robot_motor_angle_trans(void);
void head_robot_motor_angle_trans(void);
void waist_robot_motor_angle_trans(void);
void left_arm_robot_motor_angle_trans(void);

void calibration(void);
void get_motor_angle(void);
void CalibParamReset(void);

int isQuaternionEmpty(YXWestimation::Quaternion* q);
bool isCalibParamEmpty(void);

void movmean_init(void);
void movmean(void);

uint8_t read_key_GPIO(uint8_t button_id);
void key1_callback(void *button);
void key_hand_callback(void *button);
void button_attach_events(void);

void encode_can_message(int can_id, uint8_t motor_id, double angle, uint16_t max_speed, CanTxMessage* can_tx_message);

unsigned char Sum_cheak(unsigned char* data);
void encode_com_message(CanTxMessage* can_tx_message, ComTxMessage* com_tx_message);


}






#endif


