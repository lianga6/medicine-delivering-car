#include "main.h"
#include "stm32f1xx.h"
#include "PID_location.h"
#include "PID.h"
#include "control.h"
#include "Myusart.h"
#include "tim.h"
#include "Mynvic.h"
#include "stdio.h"

#define zuozhuan 200
#define youzhuan 200
#define zhuanshen 400

uint16_t delay_time=0;
uint16_t delay_time2=0;//转向切换到直线时需要加延时  此位在每次转完后清0
int a=0,b=0;

unsigned char location_control_count = 0;  //执行频率不需要那么高的用这个事件计数，用在中断中

void car_location(int32_t location_cm)//小车位置环，传入行走的距离
{
	Stop_flag = 0;//设置距离后置0不停止 继续运行  然后到达位置后置1
	spinturn_flag=1;//开始转向  转完置0
	
	
	
	set_pid_target(&pid_location1, location_cm);//将要跑的位置传入给位置环pid结构
	set_pid_target(&pid_location2, location_cm);
	
	Motor_Enable();//使能电机
	
	g_motor1_sum=0;//将当前位置记录清0
	g_motor2_sum=0;
}

void car_spinctl(void)
{
	if(Task_flag==2)
	{
    if(spinturn_flag)//若当前为转向
    {
        if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
        {
            if(lor==1 || LOR==1) motor_L_turn();
            if(lor==2 || LOR==2) motor_R_turn();
            printf("%ld",g_lMotor1PulseSigma);
            twomotor_ctl(400,400);
        }
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//转向完成
        {
            delay_time2++;
            motor_L_Return();
            motor_R_Return();
            LOR=0;//转向标志置为直行
            lor=0;
            Stop_flag=0;
            spinturn_flag=0;
  
            prespin_time=0;
            homespin_time=0;
            g_lMotor1PulseSigma=0;
            g_lMotor2PulseSigma=0;
        }
        
    }
    else if(spinturn_flag==0 && spinback_flag==0 && preturn_flag==0)//还未开始转向只进行速度环和openmv巡线补偿  且未预转向
    {
//        output=OUTPUT;
//        if(output>=100)output=-(output-100);
////        delay_time2++;
//		if(receive_flag1==0)
//		{
//			if(Stop_flag==0&&road==1)
//				Stop_flag=1;
//			else if(Stop_flag==0&&road==2)
//				Stop_flag=2;
////                Stop_flag=road;
//			receive_flag1=1;
//		}
        
//        printf("%ld",g_lMotor1PulseSigma);//在这里面成功打印出来都是0
        g_lMotor1PulseSigma=0;//在直行时，位置环脉冲累加和一直为0，待到转向时再开始计数
        g_lMotor2PulseSigma=0;
        delay_time++;
        if(delay_time>=1000)
        {
            delay_time=0;
            receive_flag1=0;
            receive_flag2=0;
        }
        set_pid_target(&pid_speed1,1200);
        set_pid_target(&pid_speed2,1200);
        speed1_Outval=speed_pid_realize(&pid_speed1,cnt_temp1);
        speed2_Outval=speed_pid_realize(&pid_speed2,cnt_temp2);
        a=speed1_Outval;
        b=speed2_Outval;
        if(a>800 || b>800 ){
            a=800;
            b=800;
        }
//        a=__HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_2);
        twomotor_ctl(a+output*15,b-output*15);
    }
    else if(spinback_flag==1)//背身转向标志位
    {
        if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
        {
            twomotor_ctl(500,500);
        }
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//转向完成
        {
            motor_L_Return();
            motor_R_Return();
            g_lMotor1PulseSigma=0;
            g_lMotor2PulseSigma=0;
            spinback_flag=0;
//			Stop_flag=0;
        }
    }
    else if(prespin_time)
    {
        delay_time2=0;
        motor_L_Return();
        motor_R_Return();
        g_lMotor1PulseSigma=0;
        g_lMotor2PulseSigma=0;
        set_pid_target(&pid_speed1,1200);
        set_pid_target(&pid_speed2,1200);
        speed1_Outval=speed_pid_realize(&pid_speed1,cnt_temp1);
        speed2_Outval=speed_pid_realize(&pid_speed2,cnt_temp2);
        a=speed1_Outval;
        b=speed2_Outval;
//        a=__HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_2);
        twomotor_ctl(a,b);
		}
	}
	else if (Task_flag==3)
	{
		if(spinturn_flag)//若当前为转向
		{
			if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
			{
				if(lor==1 || LOR==1) motor_L_turn();
				if(lor==2 || LOR==2) motor_R_turn();
				printf("%ld",g_lMotor1PulseSigma);
				twomotor_ctl(400,400);
			}
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//转向完成
        {
            delay_time2++;
            motor_L_Return();
            motor_R_Return();
            LOR=0;//转向标志置为直行
            lor=0;
            Stop_flag=0;
            spinturn_flag=0;
  
            prespin_time=0;
            homespin_time=0;
            g_lMotor1PulseSigma=0;
            g_lMotor2PulseSigma=0;
        }
        
    }
    else if(spinturn_flag==0 && spinback_flag==0 && preturn_flag==0)//还未开始转向只进行速度环和openmv巡线补偿  且未预转向
    {
//        output=OUTPUT;
//        if(output>=100)output=-(output-100);
////        delay_time2++;
//		if(receive_flag1==0)
//		{
//			if(Stop_flag==0&&road==1)
//				Stop_flag=1;
//			else if(Stop_flag==0&&road==2)
//				Stop_flag=2;
////                Stop_flag=road;
//			receive_flag1=1;
//		}
        
//        printf("%ld",g_lMotor1PulseSigma);//在这里面成功打印出来都是0
        g_lMotor1PulseSigma=0;//在直行时，位置环脉冲累加和一直为0，待到转向时再开始计数
        g_lMotor2PulseSigma=0;
        delay_time++;
        if(delay_time>=1000)
        {
            delay_time=0;
            receive_flag1=0;
            receive_flag2=0;
        }
        set_pid_target(&pid_speed1,1200);
        set_pid_target(&pid_speed2,1200);
        speed1_Outval=speed_pid_realize(&pid_speed1,cnt_temp1);
        speed2_Outval=speed_pid_realize(&pid_speed2,cnt_temp2);
        a=speed1_Outval;
        b=speed2_Outval;
        if(a>800 || b>800 ){
            a=800;
            b=800;
        }
//        a=__HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_2);
        twomotor_ctl(a+output*15,b-output*15);
    }
    else if(spinback_flag==1)//背身转向标志位
    {
        if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
        {
            twomotor_ctl(500,500);
        }
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//转向完成
        {
            motor_L_Return();
            motor_R_Return();
            g_lMotor1PulseSigma=0;
            g_lMotor2PulseSigma=0;
            spinback_flag=0;
//			Stop_flag=0;
        }
    }
    else if(prespin_time)
    {
        delay_time2=0;
        motor_L_Return();
        motor_R_Return();
        g_lMotor1PulseSigma=0;
        g_lMotor2PulseSigma=0;
        set_pid_target(&pid_speed1,1200);
        set_pid_target(&pid_speed2,1200);
        speed1_Outval=speed_pid_realize(&pid_speed1,cnt_temp1);
        speed2_Outval=speed_pid_realize(&pid_speed2,cnt_temp2);
        a=speed1_Outval;
        b=speed2_Outval;
//        a=__HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_2);
        twomotor_ctl(a,b);
		}
	}
}

/**
  * @brief  位置PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float location_pid_realize(_pid *pid, float actual_val)  //位置环光个Kp好像也可以
{
		/*计算目标值与实际值的误差*/
    pid->err=pid->target_val-actual_val;
  
//    /* 设定闭环死区 */   //外环死区可以不要 
//    if((pid->err >= -0.1) && (pid->err <= 0.1)) 
//    {
//      pid->err = 0;
//      pid->integral = 0;
//    }
    
    pid->integral += pid->err;    // 误差累积
    
    
    if((pid->err<5 ) && (pid->err>-5))   //假如以最大允许速度偏差运行1分钟，输出轴最大偏差为半圈
    {
        pid->err = 0.0;
    }
	if (pid->integral >= 50) {pid->integral =100;}//积分限幅
      else if (pid->integral < -50)  {pid->integral = -100;}
		/*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->actual_val;
}


float location1_pid_control(void)//位置环
{
	float cont_val = 0.0; //位置环输出
	int32_t actual_location;
	
	
	actual_location =  g_lMotor1PulseSigma;   //1圈 = 2464个脉冲 = 56*11*4  //这里位置用圈数代替。

    cont_val = location_pid_realize(&pid_location1, actual_location);//pid的实现函数
	  
	  //还没加串级pID之前，位置环的cont_val对应PWM。 改成串级PID后，位置换的cont_val对应目标速度
	    
     	/* 目标速度上限处理 */
      if (cont_val > TARGET_SPEED_MAX)
      {
        cont_val = TARGET_SPEED_MAX;
      }
      else if (cont_val < -TARGET_SPEED_MAX)
      {
        cont_val = -TARGET_SPEED_MAX;
      }
	
//	 #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_location, 1);                // 给通道 1 发送实际值

//  #endif
	
	return cont_val;
}

float location2_pid_control(void)  
{
	float cont_val = 0.0; 
	int32_t actual_location;
	
	  actual_location =  g_lMotor2PulseSigma;   //1圈 = 2464个脉冲 = 56*11*4  //这里位置用圈数代替。

    cont_val = location_pid_realize(&pid_location2, actual_location);   
	  
	  // 改成串级PID后，位置换的cont_val对应目标速度
	
	//目标速度限幅
	    	/* 目标速度上限处理 */
      if (cont_val > TARGET_SPEED_MAX)
      {
        cont_val = TARGET_SPEED_MAX;
      }
      else if (cont_val < -TARGET_SPEED_MAX)
      {
        cont_val = -TARGET_SPEED_MAX;
      }
	
	
//	  #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_location2, 1);                // 给通道 1 发送实际值
//		//set_computer_value(SEND_TARGET_CMD, CURVES_CH1,&TargetSpeed, 1);                // 给通道 1 发送目标值？这个是目标控制值，整定参数时不用发送，否则无法在上位机上设置
//  #else
//    printf("实际值：%d. 目标值：%.0f\n", actual_speed, get_pid_target());      // 打印实际值和目标值
//  #endif
	
	return cont_val;
}
