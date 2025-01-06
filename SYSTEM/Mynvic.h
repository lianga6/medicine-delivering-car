#ifndef __MYNVIC_H
#define __MYNVIC_H
#include "main.h"


extern uint8_t lor;//回家转向使用，因为LOR使用时会改变CNT_L  R的值，导致无法记录房间  也可以用一个数组保存下房间此处不用
extern volatile uint8_t cnt_L,cnt_R,cnt_LOR;//cnt_L和cnt_R 是用来统计左右转的次数的，而cnt_LOR用来分辨5 和 7号房间 置2就是不是这两个房间 0是5 1是7
extern uint16_t prespin_time;//当到达路口需要转向时，该单位++ 一直调整到合适时间
extern uint16_t homespin_time;//当回家到达路口需要转向时，该单位++ 一直调整到合适时间
extern uint16_t SendtoOV_time;
extern uint8_t LOR;

#endif





