#include "main.h"
#include "tim.h"
#include "stm32f1xx.h"
#include "PID.h"

#include <math.h>

/*
pid�ĵ��ڣ�д����go_line��spin_turn���棬�������������ã�����pid����
*/

float speed1_Outval,location1_Outval;
float speed2_Outval,location2_Outval;

long g_motor1_sum=0,g_motor2_sum=0;
short cnt_temp1=0,cnt_temp2=0;//�洢���Ǳ���������������������ٶȻ�

volatile long g_lMotor1PulseSigma=0;//���25ms���ۼ������ܺ�
volatile long g_lMotor2PulseSigma=0;
short g_nMotor1Pulse=0,g_nMotor2Pulse=0;//ȫ�ֱ����� ������������ֵ



_pid pid_speed1,pid_speed2;//����һ���ṹ���������ߵ����ݽṹ��pid������Ҫ������  speed���ٶȻ�ר��
_pid pid_location1,pid_location2;//����һ���ṹ���������ߵ����ݽṹ��pid������Ҫ������  location���ٶȻ�ר��


/**
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */
void PID_param_init()
{
  
  	/* �ٶ���س�ʼ������ */
    pid_speed1.target_val=0.0;				
    pid_speed1.actual_val=0.0;
    pid_speed1.err=0.0;
    pid_speed1.err_last=0.0;
    pid_speed1.integral=0.0;
  
    pid_speed1.Kp = 0.25;
    pid_speed1.Ki = 0.002;
    pid_speed1.Kd = 0.15;
		
  
  	/* �ٶ���س�ʼ������ */
    pid_speed2.target_val=0.0;				
    pid_speed2.actual_val=0.0;
    pid_speed2.err=0.0;
    pid_speed2.err_last=0.0;
    pid_speed2.integral=0.0;
  
    pid_speed2.Kp = 0.25;
    pid_speed2.Ki = 0.002;
    pid_speed2.Kd = 0.15;
    
    
  	/* λ����س�ʼ������ */
    pid_location1.target_val=0.0;				
    pid_location1.actual_val=0.0;
    pid_location1.err=0.0;
    pid_location1.err_last=0.0;
    pid_location1.integral=0.0;
                
    pid_location1.Kp = 0.24;
    pid_location1.Ki = 0.0;
    pid_location1.Kd = 0.0;
  
  	/* λ����س�ʼ������ */
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
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // ��ͨ�� 1 ���� P I D ֵ
#endif
}

/**����Ŀ��ֵ
  * @brief  ����Ŀ��ֵ
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ��
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**��ȡĿ��ֵ
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // ���õ�ǰ��Ŀ��ֵ
}

float speed1_pid_control(void)//�ٶȻ�����  
{
   
    float cont_val = 0.0;                       // ��ǰ����ֵ
    float actual_speed;
	  actual_speed = ((float)cnt_temp1*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);//�궨��ĳ˻���������㲶���Ȧ��  *1000��Ϊ�� *60��Ϊ���ӣ���λ��rmp ÿ���Ӷ���ת
    cont_val = speed_pid_realize(&pid_speed1, actual_speed);    // ���� PID ���㣬actual_speedʵ��ֵ
		return cont_val; 
}

float speed2_pid_control(void)  
{
   
    float cont_val = 0.0;                       // ��ǰ����ֵ
    float actual_speed;
	  actual_speed = ((float)cnt_temp2*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);
    cont_val = speed_pid_realize(&pid_speed2, actual_speed);    // ���� PID ����
		return cont_val;
}

//PID�ٶȻ���λ�û���������
void Speed_control(void) //�����������ת��Ϊ��λ��
{
    speed1_Outval = speed1_pid_control();    //Ҫ�ǵ��ת�򲻷���Ԥ�ڣ�������������ȡ����ֵ
    speed2_Outval = speed2_pid_control();  
}


void GET_ENCODER_NUM(void)	
{
	cnt_temp1 = (short)__HAL_TIM_GetCounter(&htim2);//��ȡ��������������ֵ
    cnt_temp2 = (short)__HAL_TIM_GetCounter(&htim3);
    if (cnt_temp1<=0) cnt_temp1=-cnt_temp1;
    if (cnt_temp2<=0) cnt_temp2=-cnt_temp2;
    g_nMotor2Pulse=cnt_temp2;
    g_nMotor1Pulse=cnt_temp1;
    
	__HAL_TIM_SET_COUNTER(&htim2,0);//��ȡ֮�����㣬�Ժ����ڶ�ʱ���ж�������
//	cnt_temp1=-cnt_temp1;//��һ���õ���������ѡ����˵��ã���һ����Ȼ
	__HAL_TIM_SET_COUNTER(&htim3,0);
    
	g_lMotor1PulseSigma+=g_nMotor1Pulse;//�ۼ�ת�򻷵�ֵ
	g_lMotor2PulseSigma+=g_nMotor2Pulse;//�ۼ�ת�򻷵�ֵ
	
}



/**
  * @brief  �ٶ�PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float speed_pid_realize(_pid *pid, float actual_val)//��һ��������PID�Ĳ��������ݣ��ڶ�����ʵ��ֵ�˴�ʵ��ֵ�ĵ�λ��rmp ת/����
{
	
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;

    if((pid->err<0.5f ) && (pid->err>-0.5f))  //��1��ô�����������1���ӣ�λ�ò�Ϊ1�����ӵ��ܳ� 
		{
      pid->err = 0.0f;
		}
        
    pid->integral += pid->err;    // ����ۻ�
	
	  /*�����޷�*/
	   	 if (pid->integral >= 1000) {pid->integral =1000;}
      else if (pid->integral < -1000)  {pid->integral = -1000;}

		/*PID�㷨ʵ��*///�˴���actual_valָ�������ֵ
    pid->actual_val = pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}




