#ifndef __MYUSARY_H
#define __MYUSARY_H
#include "main.h"

extern int output;//openmvѲ��ƫ��
extern int OUTPUT;
extern uint8_t uart2_rxbuff;//���ڽ��ջ�����
extern uint8_t uart1_rxbuff;
extern uint8_t uart3_rxbuff;

extern uint8_t output1,output2,output3,output4;

extern uint8_t road;
extern uint8_t receive_flag1;//�����жϣ�ֻ�и�λ��0�Ż������Ӧ���ݵĽ���
extern uint8_t receive_flag2;//�����жϣ�ֻ�и�λ��0�Ż������Ӧ���ݵĽ���

extern uint8_t preLOR;
//volatile extern uint8_t preLOR;//LOR ��0ֱ�� 1��ת 2��ת  �ҵ����ֻ��߻ؼ�ʱʹ��
volatile extern int Task_flag;//0������һ 1�������
extern uint8_t Num;
extern uint8_t Stop_flag;//ֹͣλ��0ǰ�� ��1ֹͣһ��ʱ�����ʶ��
void SendData_toOpenmv(void);



#endif
