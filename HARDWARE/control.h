#ifndef __CONTROL_H
#define __CONTROL_H


void gohome(void);
//void gohome1(void);
void go_line(uint16_t speed1,uint16_t speed2);
void spin_turn(uint8_t tangle);//���� 0,1,2  0ԭ��180��ת��  1��90�� 2��90��  
extern uint8_t spinback_flag;//����ת���־λ,��1��ʾ��ʼ��ͷ
extern uint8_t spinturn_flag;//ת���־λ,��1��ʾ��ʼת��
extern uint8_t preturn_flag;

extern uint8_t Do_count;


//���ÿ��ƺ���
void twomotor_ctl(float aar1,float aar2);
void set_twomotor_target(float L_speed,float R_speed);
void twomotor_out(void);//���øú�����ֱ�ӵõ�pid���ֵ
void Motor_Enable(void);
void Motor_Disable(void);
void motor_R_turn(void);
void motor_L_turn(void);
void Task_action(void);
void motor_R_Return(void);//���ӳ�ʼ����
void motor_L_Return(void);//���ӳ�ʼ����

#endif
