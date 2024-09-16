#include "pid.h"
#include "chassis.h"
#include "CAN_receive.h"


_reseive_TypeDef rx_motor;
	PID_TypeDef rm3508_speed_pid[4];
	PID_TypeDef rm3508_position_pid[4];
extern motor_measure_t motor_chassis[7];



float a=1.4142135;
float b=1.4142135;
float l=400;
float cal_target[4];




 void rx_motor_init(void)
{
		rx_motor.V_x=600;
		rx_motor.V_y=0;
		rx_motor.W=0;
}

float V_calculate (int i)
{
	float V;
	
	if(i == 0)
	{
		V=-rx_motor.V_x*a+rx_motor.V_y*b+l*rx_motor.W;
	}
	else if(i == 1)
	{
		V=-rx_motor.V_x*a-rx_motor.V_y*b+l*rx_motor.W;
	}
	else if(i == 2)
	{
		V=rx_motor.V_x*a-rx_motor.V_y*b+l*rx_motor.W;
	}
	else if(i ==3)
	{
		 V=rx_motor.V_x*a+rx_motor.V_y*b+l*rx_motor.W;
	}
		
		return V;
}

void chassis_init(void)
{
	  for(int i = 0 ;i<4;i++ )
  	{
	rx_motor_init();
	pid_init(&rm3508_speed_pid[i]);
	rm3508_speed_pid[i].f_param_init(&rm3508_speed_pid[i], PID_Speed, 
								  16000,5000,0,0,800,V_calculate(i),1.48,0.039,0.1,PID_Integral_Limit  | PID_OutputFilter | PID_Derivative_On_Measurement | PID_DerivativeFilter);
	}
}


void chassis_run(void)
{

	  for(int i = 0; i<4 ; i ++)
	  {
		  rx_motor_init();
		 
		  rm3508_speed_pid[i].f_cal_pid(&rm3508_speed_pid[i], motor_chassis[i].speed_rpm);
  
	  }

		  
	CAN_cmd_chassis(rm3508_speed_pid[0].output,rm3508_speed_pid[1].output,rm3508_speed_pid[2].output,rm3508_speed_pid[3].output);

}


