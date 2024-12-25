#include <iostream>
#include "testMain.h"
#include "wit_task.h"
#include "Quaternion.h"
#include <assert.h>
// #include "multi_button.h"
#include "hand_task.h"
#include "wit_sync.h"

void init_fquat()
{
    fquat0[0] = 0.0381346, fquat0[1] = 0.1893079, fquat0[2] = 0.2392983, fquat0[3] = 0.9515485;
    fquat1[0] = 0.0381346, fquat1[1] = 0.1893079, fquat1[2] = 0.2392983, fquat1[3] = 0.9515485;
    fquat2[0] = 0.0381346, fquat2[1] = 0.1893079, fquat2[2] = 0.2392983, fquat2[3] = 0.9515485;
    fquat3[0] = 0.0381346, fquat3[1] = 0.1893079, fquat3[2] = 0.2392983, fquat3[3] = 0.9515485;
    fquat4[0] = 0.0381346, fquat4[1] = 0.1893079, fquat4[2] = 0.2392983, fquat4[3] = 0.9515485;
    fquat5[0] = 0.0381346, fquat5[1] = 0.1893079, fquat5[2] = 0.2392983, fquat5[3] = 0.95154850;
    fquat6[0] = 0.0381346, fquat6[1] = 0.1893079, fquat6[2] = 0.2392983, fquat6[3] = 0.9515485;
    fquat7[0] = 0.0381346, fquat7[1] = 0.1893079, fquat7[2] = 0.2392983, fquat7[3] = 0.9515485;
    fquat8[0] = 0.0381346, fquat8[1] = 0.1893079, fquat8[2] = 0.2392983, fquat8[3] = 0.9515485;
}

int main(void)
{
    init_fquat();

    movmean_init();

    while (1)
    {
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
		
		#if !SYNC_IMU_ENABLE
		// if(Tim3Flag)
		// {
		// 	button_ticks();
		// 	Tim3Flag = 0;
		// }
		
		#if HAND_ENABLE
		if(Hand_Flag)
		{
			if(hand_task_timer < 40*2)
			{
				hand_motion_status.IsSideSway = 1;
			}
			else
			{
				hand_motion_status.IsSideSway = 0;
			}
			hand_motion();
			Hand_Flag = 0;
		}
		#endif

		if(IMU_Status.IsMotionTrack && (!IMU_Status.IsCalib))
		{
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
		
		if(Tim2Flag)
		{
			millis += 25;
			if(IMU_Status.IsCalib && (!IMU_Status.IsMotionTrack))
			{
				if(isCalibParamEmpty())
				{
					calibration();
					// LED1(0); 	// led1 on
				}
				else
				{
//					int length = snprintf(buffer, sizeof(buffer), "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\r\n", 
//															quat_calib_IMU67.w, quat_calib_IMU67.v[0], quat_calib_IMU67.v[1], quat_calib_IMU67.v[2], quat_calib_IMU78.w, quat_calib_IMU78.v[0], quat_calib_IMU78.v[1], quat_calib_IMU78.v[2]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, HAL_MAX_DELAY);
					// LED1(1); 	// led1 off
				}
			}
			
			if(IMU_Status.IsMotionTrack && (!IMU_Status.IsCalib))
			{
				if(IMU_Status.IsMovmean)
				{
					#if RIGHT_ARM_ENABLE
//					int length = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n",
//																movmean_motor_angle[0], movmean_motor_angle[1], movmean_motor_angle[2], movmean_motor_angle[3], movmean_motor_angle[4], movmean_motor_angle[5], movmean_motor_angle[6]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, HAL_MAX_DELAY);
//					int length01 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f\r\n", movmean_motor_angle[0], movmean_motor_angle[1], movmean_motor_angle[2]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length01, HAL_MAX_DELAY);
					for(int i = 0; i < 7; i++)
					{
						encode_can_message(right_arm_motor_id[i], movmean_motor_angle[i], motor_max_speed, &can_tx_message);
						encode_com_message(&can_tx_message, &com_tx_message);
						// HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
					}
					#endif
					#if HEAD_ENABLE
					int length02 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f\r\n",
																movmean_motor_angle[7], movmean_motor_angle[8]);
					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length02, HAL_MAX_DELAY);
					#endif
					#if WAIST_ENABLE
					int length03 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f\r\n",
																movmean_motor_angle[9], movmean_motor_angle[10], movmean_motor_angle[11]);
					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length03, HAL_MAX_DELAY);
					#endif
					#if LEFT_ARM_ENABLE
//					uart_printf("%d\r\n", 222);
//					int length01 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n",
//																movmean_motor_angle[12], movmean_motor_angle[13], movmean_motor_angle[14], movmean_motor_angle[15], movmean_motor_angle[16], movmean_motor_angle[17], movmean_motor_angle[18]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length01, HAL_MAX_DELAY);
//					int length01 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f, %0.2f\r\n", motor_angle[15], motor_angle[16], motor_angle[17]);
//					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length01, HAL_MAX_DELAY);
					for(int i = 0; i < 7; i++)
					{
						encode_can_message(left_arm_motor_id[i], movmean_motor_angle[i + 12], motor_max_speed, &can_tx_message);
						encode_com_message(&can_tx_message, &com_tx_message);
						// HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
					}
					#endif
					#if HAND_ENABLE
					if(Hand_Flag)
						
					{
						if(hand_task_timer < 40*5)
						{
							hand_motion_status.IsSideSway = 1;
						}
						else
						{
							hand_motion_status.IsSideSway = 0;
						}
						hand_motion();
						Hand_Flag = 0;
					}
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
						encode_can_message(right_arm_motor_id[i], motor_angle[i], motor_max_speed, &can_tx_message);
						encode_com_message(&can_tx_message, &com_tx_message);
						// HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
					}
					#endif
					#if HEAD_ENABLE
					int length02 = snprintf(buffer, sizeof(buffer), "%0.2f, %0.2f\r\n",
																motor_angle[7], motor_angle[8]);
					HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length02, HAL_MAX_DELAY);
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
						encode_can_message(left_arm_motor_id[i], motor_angle[i + 12], motor_max_speed, &can_tx_message);
						encode_com_message(&can_tx_message, &com_tx_message);
						// HAL_UART_Transmit(&huart1, (uint8_t *)&com_tx_message.data, com_tx_message.Len, HAL_MAX_DELAY);
					}
					#endif
					#if HAND_ENABLE
					if(Hand_Flag)
					{
						if(hand_task_timer < 40*2)
						{
							hand_motion_status.IsSideSway = 1;
						}
						else
						{
							hand_motion_status.IsSideSway = 0;
						}
						hand_motion();
						Hand_Flag = 0;
					}
					#endif
				}
			}

			Tim2Flag = 0;
		}
		
		#endif
    }
  /* USER CODE END 3 */
}