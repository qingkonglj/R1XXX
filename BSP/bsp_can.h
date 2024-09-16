/**
  ******************************************************************************
  * @file     bsp_can.h
  * @author   Junshuo
  * @version  V1.0
  * @date     Mar-28-2023
  * @brief    This file contains the headers of bsp_can.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "can.h"
//#include "CAN_defines.h"
/* Exported types ------------------------------------------------------------*/
typedef struct CAN_RX_Typedef
{
	int message_timestamp;
	int data_length;
	int data[8];
	int filter_index;
	int frame_type;
	int id_type;
	int32_t ID;
}CAN_RX_Typedef;

typedef struct CAN_TX_Typedef
{
	int id_type;
	int frame_type;
	int send_timestamp;
	int32_t ID;
	int data_length;
	uint8_t data[8];
}CAN_TX_Typedef;
typedef union
{
	
	uint8_t data8[8];	
	int16_t data16[4];	
	int data32[2];
	float dataf[2];
	
}UnionDataType;



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

void CAN_Filter_Init(void);
void CAN_Send_Packet(CAN_HandleTypeDef *hcan, CAN_TX_Typedef *tx);
void CAN_Get_Packet(CAN_HandleTypeDef *hcan, CAN_RX_Typedef *rx);

#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

