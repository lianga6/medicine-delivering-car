#ifndef __PID_H_
#define __PID_H_	

typedef struct
{
    float target_val;           //Ŀ��ֵ
    float actual_val;        		//ʵ��ֵ
    float err;             			//����ƫ��ֵ
    float err_last;          		//������һ��ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
}_pid;

extern float speed1_Outval,location1_Outval;
extern float speed2_Outval,location2_Outval;


extern long g_motor1_sum,g_motor2_sum;//�������ۼ�ֵ
extern short cnt_temp1;//������������ֵ
extern short cnt_temp2;


extern _pid pid_speed1,pid_speed2;//�ٶȻ�pid�ṹ��
extern _pid pid_location1,pid_location2;//λ�û�pid�ṹ��


//������ٱ�
#define REDUCTION_RATIO 21.6

//����������ֱ���
#define ENCODER_RESOLUTION 11

//����4��Ƶ֮��ķֱ���
#define ENCODER_TOTAL_RESOLUTION 	(ENCODER_RESOLUTION*4)

//�ٶȻ��������� ����ʱ�����ж�Ƶ��  �˴�Ϊ��ʱ��7  ��λ����
#define SPEED_PID_PERIOD 20

#define TARGET_SPEED_MAX  600  //// 60rpm����3s����60cm

extern volatile long g_lMotor1PulseSigma;//���25ms���ۼ������ܺ�
extern volatile long g_lMotor2PulseSigma;
extern short g_nMotor1Pulse,g_nMotor2Pulse;//ȫ�ֱ����� ������������ֵ



void PID_param_init(void);//PID������ʼ��
float speed_pid_realize(_pid *pid, float actual_val);//�ٶȻ����㷨ʵ��

void Speed_control(void);//��˫�ֽ���PIDʵ�ʿ���

float speed1_pid_control(void);//��ʵ��ֵ����ȫ�ֱ���speed1_Outval  speed2_Outval
float speed2_pid_control(void) ; 

void GET_ENCODER_NUM(void);//�õ���������ֵ
void set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);

#endif

