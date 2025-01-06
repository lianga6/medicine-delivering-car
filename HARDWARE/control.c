#include "stm32f1xx.h"
#include "tim.h"
#include "Myusart.h"
#include "PID.h"
#include "control.h"
#include "Sensor.h"
#include "Mynvic.h"
#include "PID_location.h"


uint8_t Line_flag=1;//1Ϊֱ��״̬��0Ϊת��  
uint8_t Motor_en=0;//���ʹ�ܱ�־λ ��1ʹ�� 0ʧ��
uint8_t en_switch=0;//���ʹ�ܿ���

uint16_t output_sum;//���ڴ洢openmv�ͱ������Ľ������


volatile int iStopCount;
uint8_t Do_count=0;//
uint8_t preturn_flag=0;//���ҵ����ʱ����λ�ñ�ʾ����Ԥת��ģʽ����ģʽ��ֻ�õ�����ƣ�������openmvѲ��Ӱ��
uint8_t spinback_flag=0;//����ת���־λ,��1��ʾ��ʼ��ͷ
uint8_t spinturn_flag=0;//ת���־λ,��1��ʾ��ʼת��

//uint8_t Stop_flag=0;//ֹͣλ��0ǰ�� ��1ֹͣһ��ʱ�����ʶ�� ������ֲ��
//uint8_t LOR=0;//LOR ��0ֱ�� 1��ת 2��ת


#define OUTPUT_OPENMV 50//openmvѲ��PID�������

/* ʹ����� */
#define MOTOR_L_ENABLE()      HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
#define MOTOR_R_ENABLE()      HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

/* ������� */
#define MOTOR_L_DISABLE()     HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
#define MOTOR_R_DISABLE()     HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);

#define hou 0
#define zuo 1
#define you 2
void Task_action(void)
{
		if(Task_flag==1&&Load_flag==1)//ִ������һ load_flag��1��ʾװ��ҩ��
		{
			Motor_Disable();
			if(Num!=0) 
            {
                Task_flag=2;//ʶ�����ֺ�ִ�������
            }
		}
      
		else if(Task_flag==2)//ִ�������  �����Ѿ�װ��
		{
            if(Stop_flag!=2)//δ����ͣ����
            {
                if (en_switch==0)
                {
                    en_switch=1;
                    Motor_Enable();
                }
                car_spinctl();//С������λ�û����ٶȻ���ѡ��   ��pwm�����
            }
			else if (Stop_flag==2)//����ͣ����
			{
              
                if(en_switch==1)
                {
                    Motor_Disable();
                    en_switch=0;
					HAL_Delay(1000);
                }
				if(Load_flag==0)//ж��ҩ��
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

void gohome(void)//lor1����ת 2����ת
{
    
    
	if((cnt_L==1&&cnt_R==0&&cnt_LOR==2))//3�ŷ���  ��1��
	{
		switch(Do_count)
		{
			case 0://�ȵ�ͷ
				spinback_flag=1;
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//ת���ֱ��һֱ����ֲ��
				{		
					lor=2;
                    Do_count++;
				}
				break;
			case 3:
				if(Stop_flag==2)//����ͣ����
				{
					Motor_Disable();
                    Do_count++;
				}
                break;
		}
	}
	else if(cnt_L==0&&cnt_R==1&&cnt_LOR==2)//4�ŷ���  ��2��
	{
		switch(Do_count)
		{
			case 0://�ȵ�ͷ
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//ת���ֱ��һֱ����ֲ��
				{		
					lor=1;
                    Do_count++;
				}
				break;
			case 3:
				if(Stop_flag==2)//����ͣ����
				{
					Motor_Disable();
					Do_count++;
				}
                break;
		}				
	}
	else if(cnt_L==1&&cnt_R==1&&cnt_LOR==0)//5�ŷ���
	{
		switch(Do_count)
		{
			case 0://�ȵ�ͷ
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(road==1)//������
				{		
					lor=1;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0 &&lor==0)//ת����ɺ�
				{
					Do_count++;
				}
                break;
			case 4:
				if(road)//ת����ɺ�������·��
				{
					lor=2;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 && lor==0)//����ڶ��ֲ��
					Do_count++;
                break;
//			case 6:
//				if(Stop_flag==1)//����ͣ����
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
				if(road==2)//����ͣ����
				{
					Motor_Disable();
					Do_count++;
				}
                break;
				
		}	
	}
	else if(cnt_L==2&&cnt_R==0&&cnt_LOR==2)//6�ŷ���
	{
		switch(Do_count)
		{
			case 0://�ȵ�ͷ
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
            break;
			case 2:
				if(Stop_flag==1)//������
				{		
					lor=2;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0 &&lor==0)//ת����ɺ�
				{
					Do_count++;
				}
                break;
			case 4:
				if(Stop_flag)//ת����ɺ�������·��
				{
					lor=2;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 &&lor==0)//����ڶ��ֲ��
					Do_count++;
                break;
			case 6:
				if(Stop_flag==2)//����ͣ����
				{
					Motor_Disable();
					Do_count++;
				}
                break;
		}
	}
	else if(cnt_L==1&&cnt_R==1&&cnt_LOR==1)//7�ŷ���
	{
		switch(Do_count)
		{
			case 0://�ȵ�ͷ
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//������
				{		
					lor=2;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0&&lor==0)//ת����ɺ�
				{
					Do_count++;
				}
                break;
			case 4:
				if(Stop_flag)//ת����ɺ�������·��
				{
					lor=1;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 &&lor==0)//����ڶ��ֲ��
					Do_count++;
                break;
			case 6:
				if(Stop_flag==2)//����ͣ����
				{
					Motor_Disable();
					Do_count++;
				}
                break;
		}
	}
	else if(cnt_L==0&&cnt_R==2&&cnt_LOR==2)//8�ŷ���
	{
		switch(Do_count)
		{
			case 0://�ȵ�ͷ
				spinback_flag=1;
                HAL_Delay(20);
				Do_count++;
				break;
            case 1:
                if(spinback_flag==0)Do_count++;
                break;
			case 2:
				if(Stop_flag==1)//������
				{		
					lor=1;
					Do_count++;
				}
				break;
			case 3:
				if(spinturn_flag==0 &&lor==0)//ת����ɺ�
				{
					Do_count++;
				}
                break;
			case 4:
				if(Stop_flag)//ת����ɺ�������·��
				{
					lor=1;
					Do_count++;
				}
                break;
			case 5:
				if(spinturn_flag==0 && lor==0)//����ڶ��ֲ��
					Do_count++;
                break;
			case 6:
				if(Stop_flag==2)//����ͣ����
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
void set_twomotor_target(float L_speed,float R_speed)//�����������ӵ��ٶȻ�Ŀ��ֵ
{
	set_pid_target(&pid_speed1,L_speed);
	set_pid_target(&pid_speed2,R_speed);
}

void twomotor_out(void)//���øú�����ֱ�ӵõ�pid���ֵ
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

void motor_L_Return(void)//���ӳ�ʼ����
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
}
void motor_R_Return(void)//���ӳ�ʼ����
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
}
void motor_L_turn(void)//�������Ӽ��Է�ת
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
}
void motor_R_turn(void)//�������Ӽ��Է�ת
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
}

