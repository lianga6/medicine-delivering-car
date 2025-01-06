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
uint16_t delay_time2=0;//ת���л���ֱ��ʱ��Ҫ����ʱ  ��λ��ÿ��ת�����0
int a=0,b=0;

unsigned char location_control_count = 0;  //ִ��Ƶ�ʲ���Ҫ��ô�ߵ�������¼������������ж���

void car_location(int32_t location_cm)//С��λ�û����������ߵľ���
{
	Stop_flag = 0;//���þ������0��ֹͣ ��������  Ȼ�󵽴�λ�ú���1
	spinturn_flag=1;//��ʼת��  ת����0
	
	
	
	set_pid_target(&pid_location1, location_cm);//��Ҫ�ܵ�λ�ô����λ�û�pid�ṹ
	set_pid_target(&pid_location2, location_cm);
	
	Motor_Enable();//ʹ�ܵ��
	
	g_motor1_sum=0;//����ǰλ�ü�¼��0
	g_motor2_sum=0;
}

void car_spinctl(void)
{
	if(Task_flag==2)
	{
    if(spinturn_flag)//����ǰΪת��
    {
        if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
        {
            if(lor==1 || LOR==1) motor_L_turn();
            if(lor==2 || LOR==2) motor_R_turn();
            printf("%ld",g_lMotor1PulseSigma);
            twomotor_ctl(400,400);
        }
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//ת�����
        {
            delay_time2++;
            motor_L_Return();
            motor_R_Return();
            LOR=0;//ת���־��Ϊֱ��
            lor=0;
            Stop_flag=0;
            spinturn_flag=0;
  
            prespin_time=0;
            homespin_time=0;
            g_lMotor1PulseSigma=0;
            g_lMotor2PulseSigma=0;
        }
        
    }
    else if(spinturn_flag==0 && spinback_flag==0 && preturn_flag==0)//��δ��ʼת��ֻ�����ٶȻ���openmvѲ�߲���  ��δԤת��
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
        
//        printf("%ld",g_lMotor1PulseSigma);//��������ɹ���ӡ��������0
        g_lMotor1PulseSigma=0;//��ֱ��ʱ��λ�û������ۼӺ�һֱΪ0������ת��ʱ�ٿ�ʼ����
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
    else if(spinback_flag==1)//����ת���־λ
    {
        if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
        {
            twomotor_ctl(500,500);
        }
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//ת�����
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
		if(spinturn_flag)//����ǰΪת��
		{
			if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
			{
				if(lor==1 || LOR==1) motor_L_turn();
				if(lor==2 || LOR==2) motor_R_turn();
				printf("%ld",g_lMotor1PulseSigma);
				twomotor_ctl(400,400);
			}
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//ת�����
        {
            delay_time2++;
            motor_L_Return();
            motor_R_Return();
            LOR=0;//ת���־��Ϊֱ��
            lor=0;
            Stop_flag=0;
            spinturn_flag=0;
  
            prespin_time=0;
            homespin_time=0;
            g_lMotor1PulseSigma=0;
            g_lMotor2PulseSigma=0;
        }
        
    }
    else if(spinturn_flag==0 && spinback_flag==0 && preturn_flag==0)//��δ��ʼת��ֻ�����ٶȻ���openmvѲ�߲���  ��δԤת��
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
        
//        printf("%ld",g_lMotor1PulseSigma);//��������ɹ���ӡ��������0
        g_lMotor1PulseSigma=0;//��ֱ��ʱ��λ�û������ۼӺ�һֱΪ0������ת��ʱ�ٿ�ʼ����
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
    else if(spinback_flag==1)//����ת���־λ
    {
        if(g_lMotor1PulseSigma<get_pid_target(&pid_location1) || g_lMotor2PulseSigma<get_pid_target(&pid_location2))
        {
            twomotor_ctl(500,500);
        }
        else if (g_lMotor1PulseSigma>=get_pid_target(&pid_location1) ||g_lMotor2PulseSigma>=get_pid_target(&pid_location2) )//ת�����
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
  * @brief  λ��PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float location_pid_realize(_pid *pid, float actual_val)  //λ�û����Kp����Ҳ����
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;
  
//    /* �趨�ջ����� */   //�⻷�������Բ�Ҫ 
//    if((pid->err >= -0.1) && (pid->err <= 0.1)) 
//    {
//      pid->err = 0;
//      pid->integral = 0;
//    }
    
    pid->integral += pid->err;    // ����ۻ�
    
    
    if((pid->err<5 ) && (pid->err>-5))   //��������������ٶ�ƫ������1���ӣ���������ƫ��Ϊ��Ȧ
    {
        pid->err = 0.0;
    }
	if (pid->integral >= 50) {pid->integral =100;}//�����޷�
      else if (pid->integral < -50)  {pid->integral = -100;}
		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}


float location1_pid_control(void)//λ�û�
{
	float cont_val = 0.0; //λ�û����
	int32_t actual_location;
	
	
	actual_location =  g_lMotor1PulseSigma;   //1Ȧ = 2464������ = 56*11*4  //����λ����Ȧ�����档

    cont_val = location_pid_realize(&pid_location1, actual_location);//pid��ʵ�ֺ���
	  
	  //��û�Ӵ���pID֮ǰ��λ�û���cont_val��ӦPWM�� �ĳɴ���PID��λ�û���cont_val��ӦĿ���ٶ�
	    
     	/* Ŀ���ٶ����޴��� */
      if (cont_val > TARGET_SPEED_MAX)
      {
        cont_val = TARGET_SPEED_MAX;
      }
      else if (cont_val < -TARGET_SPEED_MAX)
      {
        cont_val = -TARGET_SPEED_MAX;
      }
	
//	 #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_location, 1);                // ��ͨ�� 1 ����ʵ��ֵ

//  #endif
	
	return cont_val;
}

float location2_pid_control(void)  
{
	float cont_val = 0.0; 
	int32_t actual_location;
	
	  actual_location =  g_lMotor2PulseSigma;   //1Ȧ = 2464������ = 56*11*4  //����λ����Ȧ�����档

    cont_val = location_pid_realize(&pid_location2, actual_location);   
	  
	  // �ĳɴ���PID��λ�û���cont_val��ӦĿ���ٶ�
	
	//Ŀ���ٶ��޷�
	    	/* Ŀ���ٶ����޴��� */
      if (cont_val > TARGET_SPEED_MAX)
      {
        cont_val = TARGET_SPEED_MAX;
      }
      else if (cont_val < -TARGET_SPEED_MAX)
      {
        cont_val = -TARGET_SPEED_MAX;
      }
	
	
//	  #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_location2, 1);                // ��ͨ�� 1 ����ʵ��ֵ
//		//set_computer_value(SEND_TARGET_CMD, CURVES_CH1,&TargetSpeed, 1);                // ��ͨ�� 1 ����Ŀ��ֵ�������Ŀ�����ֵ����������ʱ���÷��ͣ������޷�����λ��������
//  #else
//    printf("ʵ��ֵ��%d. Ŀ��ֵ��%.0f\n", actual_speed, get_pid_target());      // ��ӡʵ��ֵ��Ŀ��ֵ
//  #endif
	
	return cont_val;
}
