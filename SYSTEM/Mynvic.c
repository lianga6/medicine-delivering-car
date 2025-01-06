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
//uint16_t spinturn_time=0;//����ת��ʱ����λ�û�ת����Ҫ  �˴���ע�͵�
//uint16_t spinback_time=0;//ת���ʱ ����λ�û�ת����Ҫ
//uint16_t stop_time=0;//�ֲ�·��ʶ��ʱ  ��ͣ����һ����Ҫ  �˴���ע�͵�

uint8_t lor=0;//�ؼ�ת��ʹ�ã���ΪLORʹ��ʱ��ı�CNT_L  R��ֵ�������޷���¼����  Ҳ������һ�����鱣���·���˴�����
volatile uint8_t cnt_L=0,cnt_R=0,cnt_LOR=2;//cnt_L��cnt_R ������ͳ������ת�Ĵ����ģ���cnt_LOR�����ֱ�5 �� 7�ŷ��� ��2���ǲ������������� 0��5 1��7
uint8_t LOR=0;



#define zhuanxiang 800//λ�û�����ת�ľ���
#define yuzhuanxiang 1800//Ԥת��ʱ��  ԭ����740
#define zhuanshen 1700//λ�û�ת��ľ���

uint16_t SendtoOV_time=0;//���ڸ�openmv���ͼ�ʱ����ֹ����ʱ��̫��
uint16_t prespin_time=0;//������·����Ҫת��ʱ���õ�λ++ һֱ����������ʱ��
uint16_t homespin_time=0;//���ؼҵ���·����Ҫת��ʱ���õ�λ++ һֱ����������ʱ��
uint16_t preback_time=0;//ת����ʱ��־λ
uint16_t stopreset_time=0;//������յ���·��ָ����һ���λ

//uint8_t LOR=0;//LOR ��0ֱ�� 1��ת 2��ת
//uint8_t Task_flag=0;//0������һ 1�������
//uint8_t Num=0;//�ҵ�������
//uint8_t Gohome_flag=0;//����ҩ����0  ȡ��ҩ����1


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//0.5ms����һ�ζ�ʱ��7  0.2ms����һ�ζ�ʱ��6
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
//		else if(Stop_flag)//������ֲ�·  �Ϳ�ʼ��ʱ
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
            if((Stop_flag==1) && (LOR!=0) )//ʶ�𵽷ֲ�� ����ת���־λ
            {
                if(LOR!=0 && prespin_time<yuzhuanxiang)//lor����  1��ת 2��ת
                {
                    preturn_flag=1;//�ﵽ·�� ����ת���־λ
                    prespin_time++;//ʶ��ת�����ʱһ�� �ȴ��ﵽ·�ں����ת��  
                    set_pid_target(&pid_location1,zhuanxiang);
                    set_pid_target(&pid_location2,zhuanxiang);

                }
                else if(prespin_time>=yuzhuanxiang)//Ԥת��ʱ��  ���ﵽ�Ϳ�ʼת�� ������ת����ɺ� pid_location��
                {
                    
                    spinturn_flag=1;//�ﵽ·�� ����ת���־λ
                    if(LOR==1&&preturn_flag==1)//����ת  ����ת��
                    {
                        motor_L_turn();
                        if(cnt_R==1) cnt_LOR=1;//��7�ŷ���
                        cnt_L++;
                    }
                    else if(LOR==2&&preturn_flag==1)
                    {
                        motor_R_turn();
                        if(cnt_L==1) cnt_LOR=0;//��5�ŷ���
                        cnt_R++;
                    }
					preturn_flag=0;
                }
            }
        }
        else if(Task_flag==3)
        {
            if((Stop_flag==1) && (lor!=0) )//ʶ�𵽷ֲ�� ����ת���־λ
            {
                if(lor!=0 && homespin_time<yuzhuanxiang)
                {
                    homespin_time++;//ʶ��ת�����ʱһ�� �ȴ��ﵽ·�ں����ת��  
                    set_pid_target(&pid_location1,zhuanxiang);
                    set_pid_target(&pid_location2,zhuanxiang);
                }
                else if(homespin_time>=yuzhuanxiang)
                {
                    spinturn_flag=1;//�ﵽ·�� ����ת���־λ
                    if(lor==1)//����ת  ����ת��
                    {
                        motor_L_turn();
                    }
                    else if(lor==2)
                    {
                        motor_R_turn();
                    }
                }
            }
            else if(spinback_flag==1)//ת���־λ��1
            {
                preback_time++;
                if(preback_time==3)
                {
                    set_pid_target(&pid_location1,zhuanshen);
                    set_pid_target(&pid_location2,zhuanshen);
                    motor_L_turn();//�����������  ��һ���ǵ�����
                    preback_time=0;
                }
            }
        }
    }
}


