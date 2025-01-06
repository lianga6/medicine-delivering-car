#include "stm32f1xx_hal.h"
#include "Sensor.h"
uint8_t Load_flag=0;//带上药物置1  未装药物置0
//uint8_t Stop_flag=0;//停止位置0前进 置1停止一段时间进行识别  到达分岔路口置1
uint8_t Stopcar_flag=0;//判断是否到达停车点 比如药房  或者回家时使用  到达置1
