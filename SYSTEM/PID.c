#include "main.h"
#include "tim.h"
#include "stm32f1xx.h"
#include "PID.h"

#include <math.h>

/*
pid的调节，写在了go_line和spin_turn里面，其他情况不会调用，进行pid控制
*/

float speed1_Outval,location1_Outval;
float speed2_Outval,location2_Outval;

long g_motor1_sum=0,g_motor2_sum=0;
short cnt_temp1=0,cnt_temp2=0;//存储的是编码器里的脉冲数，用于速度环

volatile long g_lMotor1PulseSigma=0;//电机25ms内累计脉冲总和
volatile long g_lMotor2PulseSigma=0;
short g_nMotor1Pulse=0,g_nMotor2Pulse=0;//全局变量， 保存电机脉冲数值



_pid pid_speed1,pid_speed2;//这是一个结构体变量，里边的数据结构是pid控制需要的数据  speed是速度环专用
_pid pid_location1,pid_location2;//这是一个结构体变量，里边的数据结构是pid控制需要的数据  location是速度环专用


/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init()
{
  
  	/* 速度相关初始化参数 */
    pid_speed1.target_val=0.0;				
    pid_speed1.actual_val=0.0;
    pid_speed1.err=0.0;
    pid_speed1.err_last=0.0;
    pid_speed1.integral=0.0;
  
    pid_speed1.Kp = 0.25;
    pid_speed1.Ki = 0.002;
    pid_speed1.Kd = 0.15;
		
  
  	/* 速度相关初始化参数 */
    pid_speed2.target_val=0.0;				
    pid_speed2.actual_val=0.0;
    pid_speed2.err=0.0;
    pid_speed2.err_last=0.0;
    pid_speed2.integral=0.0;
  
    pid_speed2.Kp = 0.25;
    pid_speed2.Ki = 0.002;
    pid_speed2.Kd = 0.15;
    
    
  	/* 位置相关初始化参数 */
    pid_location1.target_val=0.0;				
    pid_location1.actual_val=0.0;
    pid_location1.err=0.0;
    pid_location1.err_last=0.0;
    pid_location1.integral=0.0;
                
    pid_location1.Kp = 0.24;
    pid_location1.Ki = 0.0;
    pid_location1.Kd = 0.0;
  
  	/* 位置相关初始化参数 */
    pid_location2.target_val=0.0;				
    pid_location2.actual_val=0.0;
    pid_location2.err=0.0;
    pid_location2.err_last=0.0;
    pid_location2.integral=0.0;
  
    pid_location2.Kp = 0.24;
    pid_location2.Ki = 0.0;
    pid_location2.Kd = 0.0;
		
		
#if defined(PID_ASSISTANT_EN)
    float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值
#endif
}

/**设置目标值
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // 设置当前的目标值
}

/**获取目标值
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // 设置当前的目标值
}

float speed1_pid_control(void)//速度环控制  
{
   
    float cont_val = 0.0;                       // 当前控制值
    float actual_speed;
	  actual_speed = ((float)cnt_temp1*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);//宏定义的乘积算出的是你捕获的圈数  *1000变为秒 *60变为分钟，则单位是rmp 每分钟多少转
    cont_val = speed_pid_realize(&pid_speed1, actual_speed);    // 进行 PID 计算，actual_speed实际值
		return cont_val; 
}

float speed2_pid_control(void)  
{
   
    float cont_val = 0.0;                       // 当前控制值
    float actual_speed;
	  actual_speed = ((float)cnt_temp2*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);
    cont_val = speed_pid_realize(&pid_speed2, actual_speed);    // 进行 PID 计算
		return cont_val;
}

//PID速度环和位置环串级调速
void Speed_control(void) //这个控制是以转速为单位的
{
    speed1_Outval = speed1_pid_control();    //要是电机转向不符合预期，就在这两句里取反数值
    speed2_Outval = speed2_pid_control();  
}


void GET_ENCODER_NUM(void)	
{
	cnt_temp1 = (short)__HAL_TIM_GetCounter(&htim2);//获取编码器计数器的值
    cnt_temp2 = (short)__HAL_TIM_GetCounter(&htim3);
    if (cnt_temp1<=0) cnt_temp1=-cnt_temp1;
    if (cnt_temp2<=0) cnt_temp2=-cnt_temp2;
    g_nMotor2Pulse=cnt_temp2;
    g_nMotor1Pulse=cnt_temp1;
    
	__HAL_TIM_SET_COUNTER(&htim2,0);//读取之后清零，以后会放在定时器中断里面用
//	cnt_temp1=-cnt_temp1;//不一定用到，若极性选择错了调用，另一个亦然
	__HAL_TIM_SET_COUNTER(&htim3,0);
    
	g_lMotor1PulseSigma+=g_nMotor1Pulse;//累计转向环的值
	g_lMotor2PulseSigma+=g_nMotor2Pulse;//累计转向环的值
	
}



/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float speed_pid_realize(_pid *pid, float actual_val)//第一个参数是PID的参数和数据，第二个是实际值此处实际值的单位是rmp 转/分钟
{
	
		/*计算目标值与实际值的误差*/
    pid->err=pid->target_val-actual_val;

    if((pid->err<0.5f ) && (pid->err>-0.5f))  //差1这么多可以吗？运行1分钟，位置差为1个轮子的周长 
		{
      pid->err = 0.0f;
		}
        
    pid->integral += pid->err;    // 误差累积
	
	  /*积分限幅*/
	   	 if (pid->integral >= 1000) {pid->integral =1000;}
      else if (pid->integral < -1000)  {pid->integral = -1000;}

		/*PID算法实现*///此处的actual_val指的是输出值
    pid->actual_val = pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->actual_val;
}




