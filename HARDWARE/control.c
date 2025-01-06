#include "stm32f1xx.h"
#include "tim.h"
#include "Myusart.h"
#include "PID.h"
#include "control.h"
#include "Sensor.h"
#include "Mynvic.h"
#include "PID_location.h"


uint8_t Line_flag=1;//1为直线状态，0为转向  
uint8_t Motor_en=0;//电机使能标志位 置1使能 0失能
uint8_t en_switch=0;//电机使能开关

uint16_t output_sum;//用于存储openmv和编码器的矫正输出


volatile int iStopCount;
uint8_t Do_count=0;//
uint8_t preturn_flag=0;//当找到岔口时，该位置表示进入预转向模式，该模式下只用电机控制，而不受openmv巡线影响
uint8_t spinback_flag=0;//背身转向标志位,置1表示开始掉头
uint8_t spinturn_flag=0;//转向标志位,置1表示开始转向

//uint8_t Stop_flag=0;//停止位置0前进 置1停止一段时间进行识别 即到达分岔口
//uint8_t LOR=0;//LOR 置0直线 1左转 2右转


#define OUTPUT_OPENMV 50//openmv巡线PID结果倍放

/* 使能输出 */
#define MOTOR_L_ENABLE()      HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
#define MOTOR_R_ENABLE()      HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

/* 禁用输出 */
#define MOTOR_L_DISABLE()     HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
#define MOTOR_R_DISABLE()     HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);

#define hou 0
#define zuo 1
#define you 2
void Task_action(void)
{
		if(Task_flag==1&&Load_flag==1)//执行任务一 load_flag置1表示装上药物
		{
			Motor_Disable();
			if(Num!=0) 
            {
                Task_flag=2;//识别数字后执行任务二
            }
		}
      
		else if(Task_flag==2)//执行任务二  货物已经装载
		{
            if(Stop_flag!=2)//未到达停车点
            {
                if (en_switch==0)
                {
                    en_switch=1;
                    Motor_Enable();
                }
                car_spinctl();//小车进行位置环和速度环的选择   及pwm的输出
            }
			else if (Stop_flag==2)//到达停车点
			{
              
                if(en_switch==1)
                {
                    Motor_Disable();
                    en_switch=0;
					HAL_Delay(1000);
                }
				if(Load_flag==0)//卸下药物
				{
                   Task_flag=3;
				}
				Task_flag=3;
			}
		}
        else if(Task_flag==3)
        {
            if (en_switch==0)
            {
                en_switch=1;
                Motor_Enable();
            }
			cnt_LOR=0;
			cnt_L=1;
			cnt_R=1;
			Task_flag=3;
            gohome();
			
        }
}

void gohome(void)//lor1是左转 2是右转
{
    
    
	if((cnt_L==1&&cnt_R==0&&cnt_LOR==2))//3号房间  和1号
	{
		switch(Do_count)
		{
			case 0://先掉头
				spinback_flag=1;
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//转身后直行一直到达分岔口
				{		
					lor=2;
                    Do_count++;
				}
				break;
			case 3:
				if(Stop_flag==2)//遇到停车点
				{
					Motor_Disable();
                    Do_count++;
				}
                break;
		}
	}
	else if(cnt_L==0&&cnt_R==1&&cnt_LOR==2)//4号房间  和2号
	{
		switch(Do_count)
		{
			case 0://先掉头
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//转身后直行一直到达分岔口
				{		
					lor=1;
                    Do_count++;
				}
				break;
			case 3:
				if(Stop_flag==2)//到达停车点
				{
					Motor_Disable();
					Do_count++;
				}
                break;
		}				
	}
	else if(cnt_L==1&&cnt_R==1&&cnt_LOR==0)//5号房间
	{
		switch(Do_count)
		{
			case 0://先掉头
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(road==1)//到达岔口
				{		
					lor=1;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0 &&lor==0)//转向完成后
				{
					Do_count++;
				}
                break;
			case 4:
				if(road)//转向完成后又遇到路口
				{
					lor=2;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 && lor==0)//到达第二分岔口
					Do_count++;
                break;
//			case 6:
//				if(Stop_flag==1)//到达停车点
//				{
//					iStopCount++;
//					if(iStopCount>=20000)
//					{
//						Stop_flag=0;
//						Do_count++;
//						iStopCount=0;
//					}
//				}
//                break;
			case 6:
				if(road==2)//到达停车点
				{
					Motor_Disable();
					Do_count++;
				}
                break;
				
		}	
	}
	else if(cnt_L==2&&cnt_R==0&&cnt_LOR==2)//6号房间
	{
		switch(Do_count)
		{
			case 0://先掉头
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
            break;
			case 2:
				if(Stop_flag==1)//到达岔口
				{		
					lor=2;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0 &&lor==0)//转向完成后
				{
					Do_count++;
				}
                break;
			case 4:
				if(Stop_flag)//转向完成后又遇到路口
				{
					lor=2;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 &&lor==0)//到达第二分岔口
					Do_count++;
                break;
			case 6:
				if(Stop_flag==2)//到达停车点
				{
					Motor_Disable();
					Do_count++;
				}
                break;
		}
	}
	else if(cnt_L==1&&cnt_R==1&&cnt_LOR==1)//7号房间
	{
		switch(Do_count)
		{
			case 0://先掉头
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//到达岔口
				{		
					lor=2;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0&&lor==0)//转向完成后
				{
					Do_count++;
				}
                break;
			case 4:
				if(Stop_flag)//转向完成后又遇到路口
				{
					lor=1;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 &&lor==0)//到达第二分岔口
					Do_count++;
                break;
			case 6:
				if(Stop_flag==2)//到达停车点
				{
					Motor_Disable();
					Do_count++;
				}
                break;
		}
	}
	else if(cnt_L==0&&cnt_R==2&&cnt_LOR==2)//8号房间
	{
		switch(Do_count)
		{
			case 0://先掉头
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//到达岔口
				{		
					lor=1;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0 &&lor==0)//转向完成后
				{
					Do_count++;
				}
                break;
			case 4:
				if(Stop_flag)//转向完成后又遇到路口
				{
					lor=1;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 && lor==0)//到达第二分岔口
					Do_count++;
                break;
			case 6:
				if(Stop_flag==2)//到达停车点
				{
					Motor_Disable();
					Do_count++;
				}
                break;
		}
	}
//    else
//    {
//        Motor_Disable();
//    }
    car_spinctl();
}

void twomotor_ctl(float aar1,float aar2)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,aar1);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,aar2);
}
void set_twomotor_target(float L_speed,float R_speed)//设置两个轮子的速度环目标值
{
	set_pid_target(&pid_speed1,L_speed);
	set_pid_target(&pid_speed2,R_speed);
}

void twomotor_out(void)//调用该函数可直接得到pid输出值
{
	speed1_Outval=speed1_pid_control();
	speed2_Outval=speed2_pid_control();
}

void Motor_Enable(void)
{
	Motor_en=1;
	MOTOR_L_ENABLE()
	MOTOR_R_ENABLE()
}

void Motor_Disable(void)
{

	Motor_en=0;
	MOTOR_L_DISABLE()
	MOTOR_R_DISABLE()
}

void motor_L_Return(void)//轮子初始极性
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
}
void motor_R_Return(void)//轮子初始极性
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
}
void motor_L_turn(void)//左轮轮子极性反转
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
}
void motor_R_turn(void)//右轮轮子极性反转
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
}

