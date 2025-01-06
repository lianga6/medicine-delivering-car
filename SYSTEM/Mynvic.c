#include "stm32f1xx_hal.h"
#include "tim.h"
#include "main.h"
#include "Myusart.h"
#include "PID.h"
#include "control.h"
#include "Sensor.h"
#include "PID_location.h"
#include "Mynvic.h"
#include "stdio.h"
//uint16_t spinturn_time=0;//左右转计时若用位置环转向不需要  此处先注释掉
//uint16_t spinback_time=0;//转身计时 若用位置环转向不需要
//uint16_t stop_time=0;//分叉路口识别到时  暂停，不一定需要  此处先注释掉

uint8_t lor=0;//回家转向使用，因为LOR使用时会改变CNT_L  R的值，导致无法记录房间  也可以用一个数组保存下房间此处不用
volatile uint8_t cnt_L=0,cnt_R=0,cnt_LOR=2;//cnt_L和cnt_R 是用来统计左右转的次数的，而cnt_LOR用来分辨5 和 7号房间 置2就是不是这两个房间 0是5 1是7
uint8_t LOR=0;



#define zhuanxiang 800//位置环左右转的距离
#define yuzhuanxiang 1800//预转向时间  原本是740
#define zhuanshen 1700//位置环转身的距离

uint16_t SendtoOV_time=0;//用于给openmv发送计时，防止发送时间太快
uint16_t prespin_time=0;//当到达路口需要转向时，该单位++ 一直调整到合适时间
uint16_t homespin_time=0;//当回家到达路口需要转向时，该单位++ 一直调整到合适时间
uint16_t preback_time=0;//转身延时标志位
uint16_t stopreset_time=0;//若误接收到达路口指令则一秒后复位

//uint8_t LOR=0;//LOR 置0直线 1左转 2右转
//uint8_t Task_flag=0;//0是任务一 1是任务二
//uint8_t Num=0;//找到的数字
//uint8_t Gohome_flag=0;//带上药物置0  取下药物置1


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//0.5ms进入一次定时器7  0.2ms进入一次定时器6
{
	if(htim == (&htim7))
	{
        
         if(spinturn_flag==0 && spinback_flag==0 && preturn_flag==0)
        {
            output=OUTPUT;
            if(output>=100)output=-(output-100);
            delay_time2++;
            if(receive_flag1==0 && delay_time2>=2000)
            {
                if(Stop_flag==0&&road==1)
                    Stop_flag=1;
                else if(Stop_flag==0&&road==2)
                    Stop_flag=2;
    //                Stop_flag=road;
                receive_flag1=1;
            }
        }
        if(receive_flag2==0&&preLOR!=0)
        {
            LOR=preLOR;
            receive_flag2=1;
        }
//		if(Stop_flag==0)Stop_flag=Stop_flag_temp;
//		else if(Stop_flag)//若到达分岔路  就开始计时
//		{
//			stop_time++;
//			if(stop_time>=15)
//			{
//				Stop_flag=0;
//				stop_time=0;
//			}
//		}
        if(SendtoOV_time>=100)
        {
            SendData_toOpenmv();
            SendtoOV_time=0;
        }
    }
    if(htim == (&htim6))
    {
        GET_ENCODER_NUM();


        if(Task_flag==2)
        {
            if((Stop_flag==1) && (LOR!=0) )//识别到分岔口 且有转向标志位
            {
                if(LOR!=0 && prespin_time<yuzhuanxiang)//lor非零  1左转 2右转
                {
                    preturn_flag=1;//达到路口 进行转向标志位
                    prespin_time++;//识别到转向后延时一会 等待达到路口后进行转向  
                    set_pid_target(&pid_location1,zhuanxiang);
                    set_pid_target(&pid_location2,zhuanxiang);

                }
                else if(prespin_time>=yuzhuanxiang)//预转向时间  若达到就开始转向 清零在转向完成后 pid_location中
                {
                    
                    spinturn_flag=1;//达到路口 进行转向标志位
                    if(LOR==1&&preturn_flag==1)//向左转  极性转换
                    {
                        motor_L_turn();
                        if(cnt_R==1) cnt_LOR=1;//是7号房间
                        cnt_L++;
                    }
                    else if(LOR==2&&preturn_flag==1)
                    {
                        motor_R_turn();
                        if(cnt_L==1) cnt_LOR=0;//是5号房间
                        cnt_R++;
                    }
					preturn_flag=0;
                }
            }
        }
        else if(Task_flag==3)
        {
            if((Stop_flag==1) && (lor!=0) )//识别到分岔口 且有转向标志位
            {
                if(lor!=0 && homespin_time<yuzhuanxiang)
                {
                    homespin_time++;//识别到转向后延时一会 等待达到路口后进行转向  
                    set_pid_target(&pid_location1,zhuanxiang);
                    set_pid_target(&pid_location2,zhuanxiang);
                }
                else if(homespin_time>=yuzhuanxiang)
                {
                    spinturn_flag=1;//达到路口 进行转向标志位
                    if(lor==1)//向左转  极性转换
                    {
                        motor_L_turn();
                    }
                    else if(lor==2)
                    {
                        motor_R_turn();
                    }
                }
            }
            else if(spinback_flag==1)//转身标志位置1
            {
                preback_time++;
                if(preback_time==3)
                {
                    set_pid_target(&pid_location1,zhuanshen);
                    set_pid_target(&pid_location2,zhuanshen);
                    motor_L_turn();//轮子是任意的  不一定非得左轮
                    preback_time=0;
                }
            }
        }
    }
}


