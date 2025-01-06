#ifndef __CONTROL_H
#define __CONTROL_H


void gohome(void);
//void gohome1(void);
void go_line(uint16_t speed1,uint16_t speed2);
void spin_turn(uint8_t tangle);//输入 0,1,2  0原地180度转向  1左90度 2右90度  
extern uint8_t spinback_flag;//背身转向标志位,置1表示开始掉头
extern uint8_t spinturn_flag;//转向标志位,置1表示开始转向
extern uint8_t preturn_flag;

extern uint8_t Do_count;


//常用控制函数
void twomotor_ctl(float aar1,float aar2);
void set_twomotor_target(float L_speed,float R_speed);
void twomotor_out(void);//调用该函数可直接得到pid输出值
void Motor_Enable(void);
void Motor_Disable(void);
void motor_R_turn(void);
void motor_L_turn(void);
void Task_action(void);
void motor_R_Return(void);//轮子初始极性
void motor_L_Return(void);//轮子初始极性

#endif
