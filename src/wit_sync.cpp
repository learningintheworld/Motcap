#include "wit_sync.h"
// #include "usart.h"
#include "wit_task.h"
// #include "wit_c_sdk.h"

// using YXWestimation::Quaternion;
// using namespace YXWestimation;
namespace YXWestimation {

Quaternion quat_sync_IMU23 = {0.998186, -0.001267, 0.023817, -0.055276};
SyncQuats sync_quats_2 = {.count = 0};
SyncQuats sync_quats_3 = {.count = 0};
Quaternion sync_quat2_GSR;
Quaternion sync_quat3_GSR;
bool OutputSyncFlag = 1;

void SyncQuats_add(SyncQuats* input, Quaternion* q){
	if(input->count + 1 <= CALIB_NUM)
	{
		Quaternion_copy(q, &input->quats[input->count]);
		input->count++;
	}
}

void get_sync_GSR(SyncQuats* input, Quaternion* quat_GSR){
	Quaternion_average(input->quats, input->count, quat_GSR);
}

bool isSyncParamEmpty(void){
	bool result = 1;
	result = (bool)isQuaternionEmpty(&sync_quat3_GSR);
	result = result && (bool)(isQuaternionEmpty(&sync_quat2_GSR));
//	#if RIGHT_ARM_ENABLE
//	result = result && (bool)(isQuaternionEmpty(&quat_BSR_inv_0) && isQuaternionEmpty(&quat_BSR_inv_1) && isQuaternionEmpty(&quat_BSR_inv_2));
//	#endif
//	#if HEAD_ENABLE
//	result = result && (bool)(isQuaternionEmpty(&quat_BSR_inv_4));
//	#endif
//	#if WAIST_ENABLE
//	result = result && (bool)(isQuaternionEmpty(&quat_BSR_inv_5));
//	#endif
//	#if LEFT_ARM_ENABLE
//	result = result && (bool)(isQuaternionEmpty(&quat_BSR_inv_6) && isQuaternionEmpty(&quat_BSR_inv_7) && isQuaternionEmpty(&quat_BSR_inv_8));
//	#endif
	return result;
}

void postureIMU(void)
{
	// extract_IMU_raw_data(sReg3, fAcc3, fGyro3, fAngle3, fmag3, fquat3);
	if(fquat3[0] < (float)QUATERNION_EPS && fquat3[1] < (float)QUATERNION_EPS && fquat3[2] < (float)QUATERNION_EPS && fquat3[3] < (float)QUATERNION_EPS)
	{
		return ;
	}
	Quaternion quat3_ENU;
	Quaternion_set(fquat3[0], fquat3[1], fquat3[2], fquat3[3], &quat3_ENU);
//	Quaternion quat3_GSR;
	if(sync_quats_3.count + 1 <= SYNC_NUM)
	{
		SyncQuats_add(&sync_quats_3, &quat3_ENU);
	}
	else if(sync_quats_3.count == SYNC_NUM)
	{
		get_sync_GSR(&sync_quats_3, &sync_quat3_GSR);
//		get_calib_param(&quat3_GSR, &quat_BSR_inv_3);
	}
	
	// extract_IMU_raw_data(sReg2, fAcc2, fGyro2, fAngle2, fmag2, fquat2);
	Quaternion quat2_ENU;
	Quaternion_set(fquat2[0], fquat2[1], fquat2[2], fquat2[3], &quat2_ENU);
//	Quaternion quat2_GSR;
	if(sync_quats_2.count + 1 <= SYNC_NUM)
	{
		SyncQuats_add(&sync_quats_2, &quat2_ENU);
	}
	else if(sync_quats_2.count == SYNC_NUM)
	{
		get_sync_GSR(&sync_quats_2, &sync_quat2_GSR);
//		get_calib_param(&quat2_GSR, &quat_BSR_inv_2);
	}
}

void get_sync_param(void)
{
	Quaternion result, sync_quat3_GSR_inv;
	Quaternion_inverse(&sync_quat3_GSR, &sync_quat3_GSR_inv);
	Quaternion_multiply(&sync_quat3_GSR_inv, &sync_quat2_GSR, &result);
	Quaternion_normalize(&result, &quat_sync_IMU23);
}

void sync_correct(Quaternion* q, Quaternion* quat_sync, Quaternion* output)
{
	Quaternion result, quat_sync_inv;
//	Quaternion_inverse(quat_sync, &quat_sync_inv);
	Quaternion_multiply(q, quat_sync, &result);
	Quaternion_normalize(&result, output);
}

}




