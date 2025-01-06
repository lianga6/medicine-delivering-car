#ifndef __MYUSARY_H
#define __MYUSARY_H
#include "main.h"

extern int output;//openmv巡线偏差
extern int OUTPUT;
extern uint8_t uart2_rxbuff;//串口接收缓冲区
extern uint8_t uart1_rxbuff;
extern uint8_t uart3_rxbuff;

extern uint8_t output1,output2,output3,output4;

extern uint8_t road;
extern uint8_t receive_flag1;//接收中断，只有该位置0才会进行相应数据的接收
extern uint8_t receive_flag2;//接收中断，只有该位置0才会进行相应数据的接收

extern uint8_t preLOR;
//volatile extern uint8_t preLOR;//LOR 置0直线 1左转 2右转  找到数字或者回家时使用
volatile extern int Task_flag;//0是任务一 1是任务二
extern uint8_t Num;
extern uint8_t Stop_flag;//停止位置0前进 置1停止一段时间进行识别
void SendData_toOpenmv(void);



#endif
