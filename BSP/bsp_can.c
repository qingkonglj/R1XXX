/**
  ******************************************************************************
  * @file     bsp_can.c
  * @author   Junshuo
  * @version  V1.0
  * @date     Mar-28-2023
	* @brief      1. CAN��ʼ������,ʹ��CAN1��CAN2���жϣ�ʹ��CAN1��CAN2���˲���
	*             2. CAN���ͺ���
	*             3. CAN���պ���
  ******************************************************************************
  * @attention
  * ע������
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "bsp_can.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "can_receive.h"
//#include "can_odrive.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

// static uint8_t idx=0;
// static CANInstance *can_instance[DEVICE_CAN_CNT] = {NULL};
extern motor_measure_t motor_bus1_measure[8];
extern motor_measure_t motor_bus2_measure[8];
//extern DJI_Motor_Instance    DJI_motor[8];
/* Extern   variables ---------------------------------------------------------*/

extern CAN_HandleTypeDef  hcan1;
extern CAN_HandleTypeDef  hcan2;

extern CAN_RX_Typedef RX;

//extern axis_t odrive_wheel[4];
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/


/**
  * @brief          CAN��ʼ������main�����е���
  */
void CAN_Filter_Init(void)
{

	CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	  can_filter_st.FilterBank = 14;
	 

//		HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);  
//	  HAL_CAN_Start(&hcan2);
//	  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  
}


/**
  * @brief          [����ODriveͨ��]����CAN���ݰ�
  * @param[in]      hcan:CAN���
  * @param[in]     	tx:��������
	* @note					  ODriveͨ�ſ����ʺɽ����
  */
void CAN_Send_Packet(CAN_HandleTypeDef *hcan, CAN_TX_Typedef *tx)
{
	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(hcan)) ){} //�ȴ�������

	uint32_t send_mail_box;
	static CAN_TxHeaderTypeDef  chassis_tx_message;

	chassis_tx_message.StdId = tx->ID;
	chassis_tx_message.IDE = tx->id_type;
	chassis_tx_message.RTR = tx->frame_type;
	chassis_tx_message.DLC = tx->data_length;

	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, tx->data, &send_mail_box);

}

/**
  * @brief          [����ODriveͨ��]��ȡCAN���ݰ�����can�ж��е���
  * @param[in]      hcan:CAN���
  * @param[out]     rx:�������ݰ�
	* @note					  ODriveͨ�ſ����ʺɽ����
  */
void CAN_Get_Packet(CAN_HandleTypeDef *hcan, CAN_RX_Typedef *rx)
{

	int frame_type = 0;
	int id_type = 0;


	id_type =  (hcan->Instance->sFIFOMailBox[0].RIR & CAN_RI0R_IDE_Msk) >> CAN_RI0R_IDE_Pos ;
	frame_type = (hcan->Instance -> sFIFOMailBox[0].RIR & CAN_RI0R_RTR_Msk) >> CAN_RI0R_RTR_Pos ;

	if(id_type)
	{
		//Extended ID
		rx->id_type = CAN_ID_EXT;
		rx->ID = (hcan -> Instance -> sFIFOMailBox[0].RIR & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos;
	}
	else
	{
		//Standard ID
		rx->id_type = CAN_ID_STD;
		rx->ID = (hcan -> Instance -> sFIFOMailBox[0].RIR & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos;
	}

	if(frame_type)
	{
		//RTR Frame
		rx->frame_type = CAN_RTR_REMOTE;
		rx->data_length = (hcan -> Instance -> sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
		hcan -> Instance -> RF0R |= CAN_RF0R_RFOM0;
		while((hcan -> Instance -> RF0R & CAN_RF0R_RFOM0)){}
	}
	else
	{
		//Data Frame
		rx->frame_type = CAN_RTR_DATA;
		rx->data_length = (hcan -> Instance -> sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
		for(int i = 0; i < rx->data_length; i++)
		{
			if(i < 4)
			{
				rx->data[i] =  (hcan -> Instance -> sFIFOMailBox[0].RDLR & ( 0xFF << (8*i))) >> (8*i);
			}
			else
			{
				rx->data[i] =  (hcan -> Instance -> sFIFOMailBox[0].RDHR & ( 0xFF << (8*(i-4)))) >> (8*(i-4));
			}
		}

		hcan -> Instance -> RF0R |= CAN_RF0R_RFOM0;	}

}



/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/

