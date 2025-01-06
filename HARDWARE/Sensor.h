#ifndef __SENSOR_H
#define __SENSOR_H

extern uint8_t Load_flag;////带上药物置0  取下药物置1
extern uint8_t Stop_flag;//停止位置0前进 置1停止一段时间进行识别
extern uint8_t Stopcar_flag;//判断是否到达停车点 比如药房  或者回家时使用  到达置1

#endif
