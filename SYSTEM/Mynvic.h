#ifndef __MYNVIC_H
#define __MYNVIC_H
#include "main.h"


extern uint8_t lor;//�ؼ�ת��ʹ�ã���ΪLORʹ��ʱ��ı�CNT_L  R��ֵ�������޷���¼����  Ҳ������һ�����鱣���·���˴�����
extern volatile uint8_t cnt_L,cnt_R,cnt_LOR;//cnt_L��cnt_R ������ͳ������ת�Ĵ����ģ���cnt_LOR�����ֱ�5 �� 7�ŷ��� ��2���ǲ������������� 0��5 1��7
extern uint16_t prespin_time;//������·����Ҫת��ʱ���õ�λ++ һֱ����������ʱ��
extern uint16_t homespin_time;//���ؼҵ���·����Ҫת��ʱ���õ�λ++ һֱ����������ʱ��
extern uint16_t SendtoOV_time;
extern uint8_t LOR;

#endif





