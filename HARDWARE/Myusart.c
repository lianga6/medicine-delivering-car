#include "stm32f1xx_hal.h"
#include "usart.h"
#include "stdio.h"
#include "Myusart.h"

/*
����1�ƻ���������ͨ��
����2��Ҫ����openmv1֮��ͨ��  openmv1����Ѳ�ߺ���·�� 
����3����openmv2֮��ͨ��  openmv2��������ʶ��
*/

uint8_t uart1_rxbuff;
uint8_t uart2_rxbuff;
uint8_t uart3_rxbuff;

uint8_t receive_flag1=0;//�����жϣ�ֻ�и�λ��0�Ż������Ӧ���ݵĽ���
uint8_t receive_flag2=0;//�����жϣ�ֻ�и�λ��0�Ż������Ӧ���ݵĽ���

uint8_t road=0;
uint8_t preLOR=0;
int output;
int OUTPUT=0;
uint8_t Stop_flag=0;//·��ʶ��λ�� 0ǰ��    ��1Ԥ��ת��

//volatile uint8_t LOR=0;//LOR ��0ֱ�� 1��ת 2��ת
volatile int Task_flag=1;//1������һ 2�������  ������ͷֻ��Ѳ�ߺ�ת��ģʽ  0Ѳ�� 1ת��
uint8_t Num=0;//�ҵ�������

uint8_t Sendbuff[8];//���ͻ�������



int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

void SendData_toOpenmv(void)
{
    uint8_t i;
    //���Ϸ��͸�openmv �����ݵĴ��� (֡ͷ�� ģ��ƥ��ģʽѡ���־λ��ģʽ2����Ҫƥ������֣�֡β)   //����Ҫ�ܸߵķ���Ƶ��
    for(i = 0; i <= 4; i++)   //��TASK��TargetNum���һ���Է��͸�openmv
    {
    sprintf((char *)Sendbuff, "*%d%d&", Task_flag,Num);//sprintf����Ҫ�����ض���
    HAL_UART_Transmit(&huart3, Sendbuff, sizeof(Sendbuff), 1000);
    }
    
}

void Openmv1_Receive_Data(uint8_t com_data)//openmv1����Ѳ�ߺ���·�� 
{
		static uint8_t RxCounter1=0;//����
		static uint16_t RxBuffer1[50]={0};//��������
		static uint8_t RxState = 0;	//״̬��
		if(RxState==0&&com_data==0xfe)
		{
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data;
		}
		else if(RxState==1&&com_data==0x2C)
		{
			RxState=2;
			RxBuffer1[RxCounter1++]=com_data;
		}
		else if (RxState==2&&com_data==0x12)
		{
			RxState=3;
			RxBuffer1[RxCounter1++]=com_data;
		}
		else if (RxState==3)//��ʼ��������
		{
			RxBuffer1[RxCounter1++]=com_data;
			if(RxCounter1>=10||com_data==0x5B)//���յ�֡β �������
			{
				RxState=4;
				OUTPUT=RxBuffer1[3];
                road=RxBuffer1[4];//ʶ��·�ڱ�־  ���ʶ��Ϊ1ʱ�Ž��и�ֵ����ֹת��ʱ���޸�Ϊ0����ת����ǰ���                

			}
		}
		else if (RxState==4)//��ȷ����	
		{
			if(RxBuffer1[RxCounter1-1]==0x5B)
			{
				RxState=0;
				RxCounter1=0;
			}
		}
		else //���մ���
		{
				RxState = 0;
				RxCounter1=0;
				for(int i=0;i<10;i++)
				{
						RxBuffer1[i]=0x00;      //�����������������
				}
		}
}

void Openmv2_Receive_Data(uint8_t com_data)//openmv2��������ʶ��
{
		static uint8_t RxCounter2=0;//����
		static uint16_t RxBuffer2[6]={0};//��������
		static uint8_t RxState2 = 0;	//״̬��
		if(RxState2==0&&com_data==0x2e)
		{
			RxState2=1;
			RxBuffer2[RxCounter2++]=com_data;
		}
		else if (RxState2==1&&com_data==0x99)
		{
			RxState2=2;
			RxBuffer2[RxCounter2++]=com_data;
		}
		else if (RxState2==2)//��ʼ��������
		{
			RxBuffer2[RxCounter2++]=com_data;
			if(RxCounter2>7||com_data==0xee)//���յ�֡β �������
			{
				RxState2=3;
				Num=RxBuffer2[2];
                preLOR=RxBuffer2[3];//ֻ����openmv������1  
//                if(RxBuffer2[3]!=0)
//                {
//                    if(receive_flag2==0)
//                    {
//                    LOR=RxBuffer2[3];//ʶ��·�ڱ�־  ���ʶ��Ϊ1ʱ�Ž��и�ֵ����ֹת��ʱ���޸�Ϊ0����ת����ǰ���
//                    receive_flag2=1;                         
//                    }
//                }
			}
		}
		else if (RxState2==3)//��ȷ����	
		{
			if(RxBuffer2[RxCounter2-1]==0xee)
			{
				RxState2=0;
				RxCounter2=0;
			}
			
		}
		else //���մ���
		{
				RxState2 = 0;
				RxCounter2=0;
				for(int i=0;i<8;i++)
				{
						RxBuffer2[i]=0x00;      //�����������������
				}
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//�����жϻص�����
{
    uint8_t tem1;// ��������޷��ŵ�
    uint8_t tem2;
//   if(huart->Instance== USART1)//������ͨ�Ŵ���
//  {   
//    tem=uart2_rxbuff;
//    Openmv1_Receive_Data(tem);
//    HAL_UART_Receive_IT(&huart2,&uart2_rxbuff,1);//�жϽ���һ���ַ�����ŵ�uart2_rxbuff 
//  }	
    
  if(huart->Instance== USART2)//����ֻ��������дUSART2  openmv1��ͨ�Ŵ���
  {   
    tem1=uart2_rxbuff;
    Openmv1_Receive_Data(tem1);
    HAL_UART_Receive_IT(&huart2,&uart2_rxbuff,1);//�жϽ���һ���ַ�����ŵ�uart2_rxbuff 
  }	
  if(huart->Instance==USART3)//openmv2��ͨ�Ŵ���
  {
      tem2=uart3_rxbuff;
      Openmv2_Receive_Data(tem2);
      HAL_UART_Receive_IT(&huart3,&uart3_rxbuff,1);
  }

}


