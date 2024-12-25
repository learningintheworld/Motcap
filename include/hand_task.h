#ifndef __HAND_TASK_H
#define __HAND_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "wit_task.h"

namespace YXWestimation {


#define HAND_ENABLE 0
#define MOTOR_NUM 4
#define key_hand_id 1


typedef struct HandStatus {
	bool IsSideSway;
	bool IsGrasp;
	bool IsLoosen;
} HandStatus;

extern uint16_t hand_task_timer;
extern uint8_t right_hand_motor_id[4];
extern uint8_t left_hand_motor_id[4];
extern double right_hand_motor_angle[4];
extern double left_hand_motor_angle[4];
extern double grasp_angle[4];
extern double loosen_angle[4];
extern double sidesway_angle[4];
extern struct Button key_hand;
extern HandStatus hand_motion_status;
extern bool Hand_Flag;

void send_mode_motion(uint8_t* hand_motor_id, double* mode_angle);
void hand_motion(void);


}

#endif
