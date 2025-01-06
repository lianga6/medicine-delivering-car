#include "stm32f1xx_hal.h"
#include "usart.h"
#include "stdio.h"
#include "Myusart.h"

/*
串口1计划用于蓝牙通信
串口2主要用于openmv1之间通信  openmv1用于巡线和找路口 
串口3用于openmv2之间通信  openmv2用于数字识别
*/

uint8_t uart1_rxbuff;
uint8_t uart2_rxbuff;
uint8_t uart3_rxbuff;

uint8_t receive_flag1=0;//接收中断，只有该位置0才会进行相应数据的接收
uint8_t receive_flag2=0;//接收中断，只有该位置0才会进行相应数据的接收

uint8_t road=0;
uint8_t preLOR=0;
int output;
int OUTPUT=0;
uint8_t Stop_flag=0;//路口识别位置 0前进    置1预备转向

//volatile uint8_t LOR=0;//LOR 置0直线 1左转 2右转
volatile int Task_flag=1;//1是任务一 2是任务二  该摄像头只有巡线和转向模式  0巡线 1转向
uint8_t Num=0;//找到的数字

uint8_t Sendbuff[8];//发送缓冲数组



int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

void SendData_toOpenmv(void)
{
    uint8_t i;
    //加上发送给openmv 的数据的代码 (帧头， 模板匹配模式选择标志位，模式2所需要匹配的数字，帧尾)   //不需要很高的发送频率
    for(i = 0; i <= 4; i++)   //将TASK和TargetNum打包一次性发送给openmv
    {
    sprintf((char *)Sendbuff, "*%d%d&", Task_flag,Num);//sprintf不需要串口重定向
    HAL_UART_Transmit(&huart3, Sendbuff, sizeof(Sendbuff), 1000);
    }
    
}

void Openmv1_Receive_Data(uint8_t com_data)//openmv1用于巡线和找路口 
{
		static uint8_t RxCounter1=0;//计数
		static uint16_t RxBuffer1[50]={0};//接收数组
		static uint8_t RxState = 0;	//状态机
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
		else if (RxState==3)//开始接收数据
		{
			RxBuffer1[RxCounter1++]=com_data;
			if(RxCounter1>=10||com_data==0x5B)//接收到帧尾 或接收满
			{
				RxState=4;
				OUTPUT=RxBuffer1[3];
                road=RxBuffer1[4];//识别路口标志  如果识别到为1时才进行赋值，防止转向时被修改为0导致转向提前完成                

			}
		}
		else if (RxState==4)//正确接收	
		{
			if(RxBuffer1[RxCounter1-1]==0x5B)
			{
				RxState=0;
				RxCounter1=0;
			}
		}
		else //接收错误
		{
				RxState = 0;
				RxCounter1=0;
				for(int i=0;i<10;i++)
				{
						RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}

void Openmv2_Receive_Data(uint8_t com_data)//openmv2用于数字识别
{
		static uint8_t RxCounter2=0;//计数
		static uint16_t RxBuffer2[6]={0};//接收数组
		static uint8_t RxState2 = 0;	//状态机
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
		else if (RxState2==2)//开始接收数据
		{
			RxBuffer2[RxCounter2++]=com_data;
			if(RxCounter2>7||com_data==0xee)//接收到帧尾 或接收满
			{
				RxState2=3;
				Num=RxBuffer2[2];
                preLOR=RxBuffer2[3];//只接收openmv发来的1  
//                if(RxBuffer2[3]!=0)
//                {
//                    if(receive_flag2==0)
//                    {
//                    LOR=RxBuffer2[3];//识别路口标志  如果识别到为1时才进行赋值，防止转向时被修改为0导致转向提前完成
//                    receive_flag2=1;                         
//                    }
//                }
			}
		}
		else if (RxState2==3)//正确接收	
		{
			if(RxBuffer2[RxCounter2-1]==0xee)
			{
				RxState2=0;
				RxCounter2=0;
			}
			
		}
		else //接收错误
		{
				RxState2 = 0;
				RxCounter2=0;
				for(int i=0;i<8;i++)
				{
						RxBuffer2[i]=0x00;      //将存放数据数组清零
				}
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口中断回调函数
{
    uint8_t tem1;// 这里的是无符号的
    uint8_t tem2;
//   if(huart->Instance== USART1)//蓝牙的通信串口
//  {   
//    tem=uart2_rxbuff;
//    Openmv1_Receive_Data(tem);
//    HAL_UART_Receive_IT(&huart2,&uart2_rxbuff,1);//中断接收一个字符，存放到uart2_rxbuff 
//  }	
    
  if(huart->Instance== USART2)//这里只能这样大写USART2  openmv1的通信串口
  {   
    tem1=uart2_rxbuff;
    Openmv1_Receive_Data(tem1);
    HAL_UART_Receive_IT(&huart2,&uart2_rxbuff,1);//中断接收一个字符，存放到uart2_rxbuff 
  }	
  if(huart->Instance==USART3)//openmv2的通信串口
  {
      tem2=uart3_rxbuff;
      Openmv2_Receive_Data(tem2);
      HAL_UART_Receive_IT(&huart3,&uart3_rxbuff,1);
  }

}


