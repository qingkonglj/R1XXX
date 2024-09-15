#ifndef __CHASSIS_H__
#define __CHASSIS_H__

typedef struct _reseive_TypeDef
{
float V_x;
float V_y;
float W;

}_reseive_TypeDef;

 void rx_motor_init(void);
float V_calculate (int i);
void chassis_run(void);
void chassis_init(void);



#endif
