/**
  ******************************************************************************
  * @file     文件名.c
  * @author   作者名
  * @version  版本号
  * @date     完成时间
  * @brief    简单介绍
  ******************************************************************************
  * @attention
  * 注意事项
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "CAN_receive.h"
#include "can.h"
#include "can_odrive.h"
#include "bsp_can.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define ABS(x)        ( (x>0) ? (x) : (-x) )
/* Private  macro -------------------------------------------------------------*/
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/* Private  variables ---------------------------------------------------------*/
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
motor_measure_t motor_chassis[4];
motor_measure_t motor_plant[2];
motor_measure_t motor_shoot[2];
//extern axis_t odrive_transfer[1];
extern	axis_t odrive_friction[3];  //摩擦轮+传送 

/* Extern   variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_RX_Typedef RX;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//    CAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];

//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if(hcan -> Instance == CAN1)
		{
			CAN_RxHeaderTypeDef rx_header;
			uint8_t rx_data[8];

			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
			{
				case CAN_3508_M1_ID:
				case CAN_3508_M2_ID:
				case CAN_3508_M3_ID:
				case CAN_3508_M4_ID:
				{
					static uint8_t i = 0;
					//get motor id
					i = rx_header.StdId - CAN_3508_M1_ID;
					get_motor_measure(&motor_chassis[i], rx_data);
					get_total_angle(&motor_chassis[i]);
					break;
				}
				case CAN_3508_M5_ID:
				case CAN_3508_M6_ID:
				{
					static uint8_t i = 0;
					//get motor id
					i = rx_header.StdId - CAN_3508_M5_ID;
					get_motor_measure(&motor_plant[i], rx_data);
					get_total_angle(&motor_plant[i]);
					break;
				}
				default:
				{
					break;
				}
			}
    
		}
		else if(hcan->Instance == CAN2)
		{
			CAN_RxHeaderTypeDef rx_header;
      uint8_t rx_data[8];

			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
			{
				case CAN_2006_M1_ID:
				case CAN_2006_M2_ID:
				{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_2006_M1_ID;
				get_motor_measure(&motor_shoot[i], rx_data);
				get_total_angle(&motor_shoot[i]);
				break;
				}
				default:
				{
					int32_t ID = 0;
					ID = rx_header.StdId; 
					int32_t NODE_ID = (ID >> 5);
					int32_t CMD_ID = (ID & 0x01F);
					static uint8_t i_odr = 0;
					i_odr = NODE_ID - CAN_ODRIVE_M1_ID;		//get motor id
					odrv_get_axis_status(&odrive_friction[i_odr], CMD_ID);
//						int32_t ID = 0;
//						CAN_Get_Packet(hcan, &RX);
//						ID = RX.ID;
//						int32_t NODE_ID = (ID >> 5);
//						int32_t CMD_ID = (ID & 0x01F);

//						if (NODE_ID <= CAN_ODRIVE_M4_ID)
//						{
//							static uint8_t i = 0;
//							i = NODE_ID - CAN_ODRIVE_M1_ID;		//get motor id
//							odrv_get_axis_status(&odrive_friction[i], CMD_ID);
//						}
				break;
        
				}
		}
}
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
//	  while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) ){} //等待空邮箱
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);

}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x205) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x206) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x207) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x208) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_plant(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
		while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) ){} //等待空邮箱
	
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);

}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x205) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x206) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x207) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x208) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
		while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} //等待空邮箱
	
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_SHOOT_ALL_ID ;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);

}


/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
        */
void get_total_angle (motor_measure_t *p){
                
				int res1, res2, delta;

        if(p->ecd < p->last_ecd)
		{                        //可能的情况
                res1 = p->ecd + 8192 - p->last_ecd;        //正转，delta=+
                res2 = p->ecd - p->last_ecd;               //反转        delta=-
        }
		else
		{        //ecd > last
                res1 = p->ecd - 8192 - p->last_ecd ;//反转        delta -
                res2 = p->ecd - p->last_ecd;                                //正转        delta +
        }
        //不管正反转，肯定是转的角度小的那个是真的
        if(ABS(res1)<ABS(res2))
                delta = res1;
        else
                delta = res2;

        p->total_angle += delta;//做了角度的累加，使输入角度可以不限制于360°以内
        p->last_ecd = p->ecd;
}
                
/************************ (C) COPYRIGHT 2024 EPOCH *****END OF FILE****/
