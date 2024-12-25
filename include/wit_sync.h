#ifndef __WIT_SYNC_H
#define __WIT_SYNC_H

#include <stdbool.h>
#include <stdint.h>
//#include "wit_task.h"
#include "Quaternion.h"

// using YXWestimation::Quaternion;
namespace YXWestimation {

#define SYNC_IMU_ENABLE 0
#define SYNC_NUM 50

typedef struct SyncQuats {
	Quaternion quats[SYNC_NUM];
	int count;
} SyncQuats;

extern YXWestimation::Quaternion quat_sync_IMU23;
//extern Quaternion sync_quat2_GSR;
//extern Quaternion sync_quat3_GSR;
extern bool OutputSyncFlag;


void SyncQuats_add(SyncQuats* input, YXWestimation::Quaternion* q);
void get_sync_GSR(SyncQuats* input, YXWestimation::Quaternion* quat_GSR);
bool isSyncParamEmpty(void);
void postureIMU(void);
void get_sync_param(void);
void sync_correct(YXWestimation::Quaternion* q, YXWestimation::Quaternion* quat_sync, YXWestimation::Quaternion* output);

}

#endif

