//#include "can_odrive.h"
//#include "shoot_task.h"
//#include "pid.h"
//#include "cmsis_os.h"
//	axis_t odrive_friction[3];  // 发射摩擦轮
//	shoot_t shoot;


///**
//  * @brief  5065摩擦轮速度设置
//  */
////float vel = 20;
//void shoot_vel(float vel,float vel2)
//{
//	odrv_Set_Input_Vel(odrive_friction[0],-vel, 0);
//	odrv_Set_Input_Vel(odrive_friction[1],-vel, 0);
//	odrv_Set_Input_Vel(odrive_friction[2],vel2, 0);
//}


// void shoot_init(shoot_t *shoot_move_init)
//{
//	shoot_move_init->shoot_RC = get_remote_point();
//	
//	// 初始化ODrive电机 摩擦轮
//	odrive_friction[0].AXIS_ID = 0x011;
//	odrive_friction[1].AXIS_ID = 0x012;
//	odrive_friction[2].AXIS_ID = 0x013;

//	
//	pid_init(&shoot.transfer[0].clamp_motor_pos);
//	shoot.transfer[0].clamp_motor_pos.f_param_init(&shoot.transfer[0].clamp_motor_pos, PID_Position, 
//													   10000.0f,0,5.0f,0,0,0,0.3f,0.0f,0.15f,PID_IMPROVE_NONE);
//	pid_init(&shoot.transfer[0].clamp_motor_vel);
//	shoot.transfer[0].clamp_motor_pos.f_param_init(&shoot.transfer[0].clamp_motor_vel, PID_Speed, 
//													   10000.0f,0,5.0f,0,0,0,2.0f,0.03f,0.0f,PID_IMPROVE_NONE);
//	
//	pid_init(&shoot.transfer[1].clamp_motor_pos);
//	shoot.transfer[1].clamp_motor_pos.f_param_init(&shoot.transfer[1].clamp_motor_pos, PID_Position, 
//													   10000.0f,0,5.0f,0,0,0,0.3f,0.0f,0.15f,PID_IMPROVE_NONE);
//	pid_init(&shoot.transfer[1].clamp_motor_vel);
//	shoot.transfer[1].clamp_motor_pos.f_param_init(&shoot.transfer[1].clamp_motor_vel, PID_Speed, 
//													   10000.0f,0,5.0f,0,0,0,2.0f,0.03f,0.0f,PID_IMPROVE_NONE);

//	shoot_vel(20,10);
//	vTaskDelay(2000);
//	shoot_vel(20,10);
//	
//  // 初始化达妙电机
//	vTaskDelay(3000);
//	ctrl_motor_in(0x01);
//	ctrl_motor_zero(0x01);

//}


