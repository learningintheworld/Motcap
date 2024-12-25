#include "hand_task.h"
// #include "usart.h"
#include <iostream>

#include "SerialPortPC.h"
extern Hexacercle::hcPortMgr mgr;

// using namespace YXWestimation;
namespace YXWestimation {

HandStatus hand_motion_status = {0, 0, 1};
uint16_t hand_task_timer = 0;
uint8_t right_hand_motor_id[4] = {0x13, 0x14, 0x15, 0x16};
uint8_t left_hand_motor_id[4] = {0x0F, 0x10, 0x11, 0x12};
double right_hand_motor_angle[4];
double left_hand_motor_angle[4];
double right_grasp_angle[4] = {-10, -10, 150, 25};
double right_loosen_angle[4] = {-1, -1, 1, 1};
double right_sidesway_angle[4] = {-5, -5, 5, 25};
double left_grasp_angle[4] = {-10, -10, -150, -25};
double left_loosen_angle[4] = {-1, -1, -1, -1};
double left_sidesway_angle[4] = {-5, -5, -5, -25};
bool Hand_Flag = 0;
// struct Button key_hand;

void send_mode_motion(uint8_t* hand_motor_id, double* mode_angle)
{
	for(int i = 0; i < 4; i++)
	{
		encode_can_message(ARM_MODULE_ID, hand_motor_id[i], mode_angle[i], motor_max_speed, &can_tx_message);
		encode_com_message(&can_tx_message, &com_tx_message);
		// HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
		mgr.SerialSend((uint8_t*)& com_tx_message.data, com_tx_message.Len);
	}
	std::cout << mode_angle[0] << " " << mode_angle[1] << " " << mode_angle[2] << " " << mode_angle[3] << std::endl;
}

void hand_motion(void)
{
	if(hand_motion_status.IsGrasp)
	{
		if(hand_motion_status.IsSideSway)
		{
			send_mode_motion(right_hand_motor_id, right_sidesway_angle);
			// send_mode_motion(left_hand_motor_id,  left_sidesway_angle);
		}
		else
		{
			send_mode_motion(right_hand_motor_id, right_grasp_angle);
			// send_mode_motion(left_hand_motor_id,  left_grasp_angle);
		}
	}
	
	if(hand_motion_status.IsLoosen)
	{
		send_mode_motion(right_hand_motor_id, right_loosen_angle);
		// send_mode_motion(left_hand_motor_id,  left_loosen_angle);
	}
}


}



