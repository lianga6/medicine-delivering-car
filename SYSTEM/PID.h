#ifndef __PID_H_
#define __PID_H_	

typedef struct
{
    float target_val;           //目标值
    float actual_val;        		//实际值
    float err;             			//定义偏差值
    float err_last;          		//定义上一个偏差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
}_pid;

extern float speed1_Outval,location1_Outval;
extern float speed2_Outval,location2_Outval;


extern long g_motor1_sum,g_motor2_sum;//编码器累计值
extern short cnt_temp1;//编码器读到的值
extern short cnt_temp2;


extern _pid pid_speed1,pid_speed2;//速度环pid结构体
extern _pid pid_location1,pid_location2;//位置环pid结构体


//电机减速比
#define REDUCTION_RATIO 21.6

//编码器物理分辨率
#define ENCODER_RESOLUTION 11

//经过4倍频之后的分辨率
#define ENCODER_TOTAL_RESOLUTION 	(ENCODER_RESOLUTION*4)

//速度环控制周期 看定时器的中断频率  此处为定时器7  单位毫秒
#define SPEED_PID_PERIOD 20

#define TARGET_SPEED_MAX  600  //// 60rpm可以3s走完60cm

extern volatile long g_lMotor1PulseSigma;//电机25ms内累计脉冲总和
extern volatile long g_lMotor2PulseSigma;
extern short g_nMotor1Pulse,g_nMotor2Pulse;//全局变量， 保存电机脉冲数值



void PID_param_init(void);//PID参数初始化
float speed_pid_realize(_pid *pid, float actual_val);//速度环的算法实现

void Speed_control(void);//对双轮进行PID实际控制

float speed1_pid_control(void);//将实际值赋给全局变量speed1_Outval  speed2_Outval
float speed2_pid_control(void) ; 

void GET_ENCODER_NUM(void);//得到编码器的值
void set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);

#endif

