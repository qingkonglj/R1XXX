/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __c620_h
#define __c620_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_3508_M5_ID = 0x205,
    CAN_3508_M6_ID = 0x206,
    CAN_GIMBAL_ALL_ID = 0x1FF,
	
		CAN_SHOOT_ALL_ID = 0x200,
		CAN_2006_M1_ID = 0x201,
		CAN_2006_M2_ID = 0x202,

} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	//int16_t total_ecd;
    float total_angle;
} motor_measure_t;


//typedef struct
//{  
//  // fp32 accel;
//  // fp32 speed;
//  fp32 angle_set; //设置旋转角度；转子角度8192*减速比36*转轴旋转圈数 x/2pi
//  int16_t give_current;
//} dji_motor_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_plant(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void get_total_angle (motor_measure_t *p);

#endif

/****************** (C) COPYRIGHT 2024 EPOCH *****END OF FILE*************/

