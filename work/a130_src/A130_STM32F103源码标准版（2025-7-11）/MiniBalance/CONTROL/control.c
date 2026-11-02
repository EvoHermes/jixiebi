/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
版本V1.0
修改时间：2021-11-04
All rights reserved
***********************************************/
#include "control.h"	
#include <math.h>
#include "stdlib.h"

u8 mode=1;
u8 ik_flag=1;
int Servo1=0,Servo2=0,Servo3=0,Servo4=0,Servo5=0,Servo6=0; //舵机PWM变量
float Velocity1=0,Velocity2=0,Velocity3=0,Velocity4=0,Velocity5=0,Velocity6=0;     //电机PWM变量
float Position1=1500,Position2=1500,Position3=1500,Position4=1500,Position5=1500,Position6=1500;   
float move_angle[7]={0,0,0,0,0,0};
short Angle1_init=0,Angle2_init=0,Angle3_init=0,Angle4_init=0,Angle5_init=0,Angle6_init=0; //舵机初始位置值
float px=0.23,py=0,pz=0.3,rx=0,ry=0;;

float angle_range[5][2]={{-1.57,1.57},//{min,max} 方向请看DH坐标示意图
												 {-1.57,1.57},
												 {-2.3,1.57},
												 {-1.57,1},
												 {-1.57,1.57}};

#define pi 3.14159
#define t1 theta_i[0]
#define t2 theta_i[1]
#define t3 theta_i[2]
#define t4 theta_i[3]
#define t5 theta_i[4]

#define a2 0.105f
#define a3 0.0975f
#define a4 0.02822f
#define d1 0.1085f
#define d5 0.17283f

float T0t[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 1}};
float T05[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 1}};
u8 i = 0; // 用以循环的变量
u8 j = 0; // 用以循环的变量
u8 k = 0; // 用以循环的变量
	
static u8 angle_range_check(float j1, float j2, float j3, float j4, float j5)
{
    float angles[] = {j1, j2, j3, j4, j5};
    for (int i = 0; i < 5; ++i) {
        if (angles[i] < angle_range[i][0] || angles[i] > angle_range[i][1]) {
            return 1;
        }
    }
    return 0;
}
	
void fk_kinematics(float angle1,float angle2,float angle3,float angle4,float angle5)
{
		if (angle_range_check(angle1, angle2, angle3, angle4, angle5)) {
				return;
		}	  
		T05[0][0] = sin(angle1)*sin(angle5) - cos(angle2 + angle3 + angle4)*cos(angle1)*cos(angle5);
    T05[0][1] = cos(angle5)*sin(angle1) + cos(angle2 + angle3 + angle4)*cos(angle1)*sin(angle5);
    T05[0][2] = -sin(angle2 + angle3 + angle4)*cos(angle1);
    T05[0][3] = -cos(angle1)*(a3*sin(angle2 + angle3) + a2*sin(angle2) + a4*cos(angle2 + angle3 + angle4) + d5*sin(angle2 + angle3 + angle4));
    T05[1][0] = -cos(angle1)*sin(angle5) - cos(angle2 + angle3 + angle4)*cos(angle5)*sin(angle1);
    T05[1][1] = cos(angle2 + angle3 + angle4)*sin(angle1)*sin(angle5) - cos(angle1)*cos(angle5);
    T05[1][2] = -sin(angle2 + angle3 + angle4)*sin(angle1);
    T05[1][3] = -sin(angle1)*(a3*sin(angle2 + angle3) + a2*sin(angle2) + a4*cos(angle2 + angle3 + angle4) + d5*sin(angle2 + angle3 + angle4));
    T05[2][0] = -sin(angle2 + angle3 + angle4)*cos(angle5);
    T05[2][1] = sin(angle2 + angle3 + angle4)*sin(angle5);
    T05[2][2] = cos(angle2 + angle3 + angle4);
    T05[2][3] = d1 + a3*cos(angle2 + angle3) + a2*cos(angle2) + d5*cos(angle2 + angle3 + angle4) - a4*sin(angle2 + angle3 + angle4);
		px=T05[0][3];
		py=T05[1][3];
    pz=T05[2][3];
		rx=atan2(T05[2][1],T05[1][1])+3.14;
		ry=atan2(T05[0][0],T05[0][2]);
}

void ik_kinematics(float x,float y,float z,float rx,float ry)
{
    // 初始化
//	  float T5t_inv[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -l}, {0, 0, 0, 1}};
		
    // 初始化
    static float theta[8][5] = {0};
    float temp[8][5] = {0};
    float t234[8] = {0};
    float M[8] = {0};
    float N[8] = {0};
    float K[4] = {0};
    float P = 0;
    float Q = 0;
    float s2 = 0;
    float c2 = 0;
    float distance = 0;
    float distance_min = 1000;
    int label = 0;
    int num = 0;
    int count[8] = {0};

    T05[0][0]=sin(ry);
		T05[0][1]=0;
		T05[0][2]=cos(ry);
		T05[0][3]=x;
		
    T05[1][0]=-cos(ry)*sin(rx);
    T05[1][1]=-cos(rx);
    T05[1][2]=sin(rx)*sin(ry);
    T05[1][3]=y;

    T05[2][0]=cos(rx)*cos(ry);
    T05[2][1]=-sin(rx);
    T05[2][2]=-cos(rx)*sin(ry);
    T05[2][3]=z;

    // 求解关节角1
    temp[0][0] = atan2(T05[1][3],T05[0][3]);
    temp[1][0] = temp[0][0];
    temp[2][0] = atan2(-T05[1][3],-T05[0][3]);
    temp[3][0] = temp[2][0];

    // 求解关节角5
    if (fabs(T05[2][2]-1) < 1e-3)
    {
        for (i=0; i<4; i++)
        {
            temp[i][4] = atan2(-T05[1][0],-T05[0][0]) - temp[i][0];
        }
    }
    else if (fabs(T05[2][2]+1) < 1e-3)
    {
        for (i=0; i<4; i++)
        {
            temp[i][4] = temp[i][0] - atan2(T05[1][0],T05[0][0]);
        }
    }
    else
    {
        temp[0][4] = atan2(T05[2][1],-T05[2][0]);
        temp[2][4] = temp[0][4];
        temp[1][4] = atan2(-T05[2][1],T05[2][0]);
        temp[3][4] = temp[1][4];
    }

    // 求解关节角2+关节角3+关节角4
    for (i=0; i<4; i++)
    {
        if (fabs(sin(temp[i][4])) < 1e-3)
        {
            t234[i] = atan2(-T05[2][0]/cos(temp[i][4]),T05[2][2]);
        }
        else
        {
            t234[i] = atan2(T05[2][1]/sin(temp[i][4]),T05[2][2]);
        }
    }

    // 求解关节角3
    for (i=0; i<4; i++)
    {
        if (fabs(sin(temp[i][0])) < 1e-3)
        {
            M[i] = (-T05[0][3]/cos(temp[i][0])) - a4*cos(t234[i]) - d5*sin(t234[i]);
        }
        else
        {
            M[i] = (-T05[1][3]/sin(temp[i][0])) - a4*cos(t234[i]) - d5*sin(t234[i]);
        }
        N[i] = T05[2][3] - d1 + a4*sin(t234[i]) - d5*cos(t234[i]);
        K[i] = (M[i]*M[i] + N[i]*N[i] - a2*a2 - a3*a3) / (2*a2*a3);
        if (fabs(K[i]-1) < 1e-3) K[i] = 1;
        else if (fabs(K[i]+1) < 1e-3) K[i] = -1;
        if (fabs(K[i]) > 1) temp[i][2] = 100, num++;
        else temp[i][2] = acos(K[i]);
    }
    for (i=4; i<8; i++)
    {
        M[i] = M[i-4];
        N[i] = N[i-4];
        t234[i] = t234[i-4];
        for (j=0; j<5; j++)
        {
            if (j == 2) temp[i][j] = -temp[i-4][j];
            else temp[i][j] = temp[i-4][j];
        }
    }

    // 求解关节角2
    for (i=0; i<8; i++)
    {
        P = a2 + a3*cos(temp[i][2]);
        Q = a3*sin(temp[i][2]);
        s2 = (P*M[i] - Q*N[i])/(P*P + Q*Q);
        c2 = (Q*M[i] + P*N[i])/(P*P + Q*Q);
        temp[i][1] = atan2(s2,c2);
    }

    // 求解关节角4
    for (i=0; i<8; i++) temp[i][3] = t234[i] - temp[i][1] - temp[i][2];

    // 筛选合适的解，换算至合理的区间范围以及选取最优解.
    j = 0;
    for (i=0; i<8; i++)
    {
        if (fabs(temp[i][2]) != 100)
        {
            count[j] = i;
            j += 1;
        }
				
    }
		ik_flag=0;
    for (i=0; i<(8-2*num); i++)
    {
				ik_flag=1;
        distance = 0;
        for (j=0; j<5; j++)
        {
            k = count[i];
            theta[i][j] = temp[k][j];
            if (theta[i][j] > pi) theta[i][j] = theta[i][j] - 2*pi;
            else if (theta[i][j] < -pi) theta[i][j] = theta[i][j] + 2*pi;
            theta[i][j] = (int)(theta[i][j]*10000.0f)/10000.0f; // 结果保留四位小数
            distance += theta[i][j]*theta[i][j]; // 计算点到原点的距离（把一组解看成五维空间中的一个点，其坐标为五个关节角的值）
        }
        if (distance < distance_min) distance_min = distance, label = i; // 选取最优解
    }
//调试信息，默认不打印
// 打印所有可能的逆解
//    printf("\n");
//    printf("可能的关节角设置为:\t");
//    printf("\n");
//    for (i=0; i<(8-2*num); i++)
//    {
//        for (j=0; j<5; j++)
//        {
//            if (theta[i][j]>=0) printf(" %.4f\t\t",fabs(theta[i][j]));
//            else printf("-%.4f\t\t",fabs(theta[i][j]));
//        }
//        printf("\n");
//    }

//    // 打印最优的逆解
//    printf("\n");
//    printf("最优的关节角设置为:\t");
//    printf("\n");
//    for (j=0; j<5; j++)
//    {
//        if (theta[label][j]>=0) printf(" %.4f\t\t",fabs(theta[label][j]));
//        else printf("-%.4f\t\t",fabs(theta[label][j]));
//    }
//    printf("\n");
//调试信息，默认不打印
			for(u8 i=0;i<5;i++) 
		{
			if(theta[label][i]<angle_range[i][0] || theta[label][i]>angle_range[i][1])
			{
						ik_flag=0;
					  return;
			}
		}
		if(ik_flag)
		{
				for(i=0;i<5;i++)
			{
						move_angle[i+1]=theta[label][i];
			}
		}
		
}
	

/**************************************************************************
函数功能：定时中断函数 所有的控制代码都在这里面		 
**************************************************************************/

int TIM2_IRQHandler(void)
{ 
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{   
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源	
			Key();        
			//扫描按键变化	
			if(PS2_ON_Flag) PS2_Control(Speed/1000); //ps2手柄控制
			
			if(mode) ik_kinematics(px,py,pz,rx,ry);
			else     fk_kinematics(move_angle[1],move_angle[2],move_angle[3],move_angle[4],move_angle[5]);

			Servo_angle_check();     //舵机旋转角度范围限制，单位弧度
			
			//通过目标角度求解舵机对应的PWM值
			Servo1=SERVO_PWM_VALUE_180(-move_angle[1])+Angle1_init;
			Servo2=SERVO_PWM_VALUE_180(move_angle[2])+Angle2_init;
			
			Servo3=SERVO_PWM_VALUE_270(-move_angle[3])+Angle3_init;
			
			Servo4=SERVO_PWM_VALUE_180(-move_angle[4])+Angle4_init;
			Servo5=SERVO_PWM_VALUE_180(-move_angle[5])+Angle5_init;
			Servo6=SERVO_PWM_VALUE_180(move_angle[6])+Angle6_init;
			Servo_Pwm_check();   //舵机旋转PWM范围限制
			//通过目标PWM值做位置PID控制
			Velocity1=Position_PID1(Position1,Servo1);//舵机PID控制
			Velocity2=Position_PID2(Position2,Servo2);//舵机PID控制
			Velocity3=Position_PID3(Position3,Servo3);//舵机PID控制
			Velocity4=Position_PID4(Position4,Servo4);//舵机PID控制
			Velocity5=Position_PID5(Position5,Servo5);//舵机PID控制
			Velocity6=Position_PID6(Position6,Servo6);//舵机PID控制
	    Servo_Speed_Check();
			Set_Pwm(Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6);    //赋值给PWM寄存器
     		
   	}       	
	 return 0;	  
} 
/**************************************************************************
函数功能：将PWM值输出给舵机，实现舵机的控制	 
**************************************************************************/
void Set_Pwm(float velocity1,float velocity2,float velocity3,float velocity4,float velocity5,float velocity6)
{	
		Position1+=velocity1;		   //速度的积分，得到舵机的位置
		Position2+=velocity2;
		Position3+=velocity3;
		Position4+=velocity4;
		Position5+=velocity5;	
		Position6+=velocity6;		
	  //赋值给STM32的寄存器
	  TIM4->CCR4=(int)Position1;   //云台
	  TIM4->CCR3=(int)Position3;   //云台上的第二个关节
	  TIM4->CCR2=(int)Position4;   //云台上的第三个关节
	  TIM4->CCR1=(int)Position5;   //控制夹爪旋转关节
	  TIM3->CCR1=(int)Position2;   //云台上的第一个关节
	  TIM3->CCR2=(int)Position6;   //夹爪
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1150)//电池电压过低  舵机将不能控制
			{	                                                
      temp=1;      
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
//舵机目标弧度极限值
void Servo_angle_check(void)
{	
	  float Amplitude_H=1.57, Amplitude_L=-1.57;  
    if(move_angle[1]<Amplitude_L)  move_angle[1]=Amplitude_L;	
		if(move_angle[1]>Amplitude_H)  move_angle[1]=Amplitude_H;	
	
	  if(move_angle[2]<Amplitude_L)  move_angle[2]=Amplitude_L;	
		if(move_angle[2]>Amplitude_H)  move_angle[2]=Amplitude_H;	
	
		if(move_angle[3]<-2.3)         move_angle[3]=-2.3;	
		if(move_angle[3]>Amplitude_H)  move_angle[3]=Amplitude_H;	
	
		if(move_angle[4]<Amplitude_L)  move_angle[4]=Amplitude_L;	
		if(move_angle[4]>1)  					 move_angle[4]=1;	//安装了摄像头的关节
	
	  if(move_angle[5]<Amplitude_L)  move_angle[5]=Amplitude_L;	
		if(move_angle[5]>Amplitude_H)  move_angle[5]=Amplitude_H;	
	
		if(move_angle[6]<-0.9)         move_angle[6]=-0.9;	//夹爪
		if(move_angle[6]>0.9)          move_angle[6]=0.9;	  
}

//舵机舵机PWM极限值
void Servo_Pwm_check(void)
{	
	  int Amplitude_H=2500, Amplitude_L=500;       
    if(Servo1<Amplitude_L)  Servo1=Amplitude_L;	
		if(Servo1>Amplitude_H)  Servo1=Amplitude_H;	
	  if(Servo2<Amplitude_L)  Servo2=Amplitude_L;	
		if(Servo2>Amplitude_H)  Servo2=Amplitude_H;	
		if(Servo3<Amplitude_L)  Servo3=Amplitude_L;	
		if(Servo3>Amplitude_H)  Servo3=Amplitude_H;	
		if(Servo4<Amplitude_L)  Servo4=Amplitude_L;	
		if(Servo4>Amplitude_H)  Servo4=Amplitude_H;	
	  if(Servo5<Amplitude_L)  Servo5=Amplitude_L;	
		if(Servo5>Amplitude_H)  Servo5=Amplitude_H;	
		if(Servo6<Amplitude_L)  Servo6=Amplitude_L;	
		if(Servo6>Amplitude_H)  Servo6=Amplitude_H;		
}
//舵机运动速度控制值
void Servo_Speed_Check(void)
{	
	  int Amplitude_H=Speed, Amplitude_L=-Speed;   
    if(Velocity1<Amplitude_L)  Velocity1=Amplitude_L;	
		if(Velocity1>Amplitude_H)  Velocity1=Amplitude_H;	
	  if(Velocity2<Amplitude_L)  Velocity2=Amplitude_L;	
		if(Velocity2>Amplitude_H)  Velocity2=Amplitude_H;	
		if(Velocity3<Amplitude_L)  Velocity3=Amplitude_L;	
		if(Velocity3>Amplitude_H)  Velocity3=Amplitude_H;	
		if(Velocity4<Amplitude_L)  Velocity4=Amplitude_L;	
		if(Velocity4>Amplitude_H)  Velocity4=Amplitude_H;	
	  if(Velocity5<Amplitude_L)  Velocity5=Amplitude_L;	
		if(Velocity5>Amplitude_H)  Velocity5=Amplitude_H;	
		if(Velocity6<Amplitude_L)  Velocity6=Amplitude_L;	
		if(Velocity6>Amplitude_H)  Velocity6=Amplitude_H;	
}

/**************************************************************************
函数功能：按键修改机械臂使能状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 temp1;
	temp1=click();
	//Clear_Oled_Flag=Long_Press();
  if(temp1==1)	mode=!mode; //单击切换使能标位置状态
}

/**************************************************************************
函数功能：手柄控制机械臂
入口参数：控制速度
返回  值：无
**************************************************************************/
void PS2_Control(float Step)
{ 	 
	 float positon_step=0.001;//单位m
	 float rotation_step=0.01;//单位rad
				if(PS2_KEY==6)    if(mode)py-=positon_step;else move_angle[3]-=Step;    //云台
	 else if(PS2_KEY==8)		if(mode)py+=positon_step;else move_angle[3]+=Step;
	
	 else if(PS2_KEY==11)		if(mode)pz+=positon_step;else move_angle[1]-=Step;    //机械手的张合
	 else if(PS2_KEY==9)		if(mode)pz-=positon_step;else move_angle[1]+=Step;
	
	 else if(PS2_KEY==5)		if(mode)px+=positon_step;else move_angle[2]+=Step;
	 else if(PS2_KEY==7)		if(mode)px-=positon_step;else move_angle[2]-=Step;
	 
	 else if(PS2_KEY==13)		 if(mode)ry+=rotation_step;else move_angle[5]+=Step;   //舵机3
	 else if(PS2_KEY==15)		 if(mode)ry-=rotation_step;else move_angle[5]-=Step;   
		
	 else if(PS2_KEY==14)		if(mode)rx+=rotation_step;else move_angle[4]+=Step;  //舵机4
	 else if(PS2_KEY==16)		if(mode)rx-=rotation_step;else move_angle[4]-=Step;
	
	 else if(PS2_KEY==12)		 move_angle[6]+=Step;
	 else if(PS2_KEY==10)		move_angle[6]-=Step;
}


/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：输入舵机的目标角度输出对应的pwm值
入口参数：舵机转向角度（单位：弧度）
返回  值：对应输出的pwm值（范围：500——2500）
**************************************************************************/
int SERVO_PWM_VALUE_180(float angle)
{
			int K=1000;
			float Ratio=0.6369; //180°舵机
	    //float Ratio=0.4246; //270°舵机
			int pwm_value;
			
			pwm_value=(SERVO_INIT-angle*K*Ratio); //舵机目标PWM数值
	    return pwm_value;
}

int SERVO_PWM_VALUE_270(float angle)
{
			int K=1000;
//			float Ratio=0.6369; //180°舵机
	    float Ratio=0.4246; //270°舵机
			int pwm_value;
			
			pwm_value=(SERVO_INIT-angle*K*Ratio); //舵机目标PWM数值
	    return pwm_value;
}

/**************************************************************************
函数功能：输入舵机的pwm值输出对应的关节角度
入口参数：舵机的pwm值（范围：500——2500）
返回  值：输出对应的关节角度（范围：-1.57 —— 1.57）
**************************************************************************/
float PWM_Angle_VALUE(int pwm)
{
			int K=1000;
			float Ratio=0.6369; //180°舵机
	    float angle=0;
	      //float Ratio=0.4246; //270°舵机
			
			angle=(SERVO_INIT-pwm)/(K*Ratio); //舵机目标PWM数值
	    return angle;
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
float Position_PID1 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID2 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID3 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID4 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID5 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
float Position_PID6 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：舵机初始位置调整函数
入口参数：无
返回  值：无
**************************************************************************/
void Servo_init_angle_adjust_function(void)
{
	int step=1; 
				if(PS2_KEY==9)     Angle1_init+=step;     //云台
	 else if(PS2_KEY==11)		 Angle1_init-=step;
	
	 else if(PS2_KEY==12)		 Angle6_init-=step;    //机械手的张合
	 else if(PS2_KEY==10)		 Angle6_init+=step;
	
	 else if(PS2_KEY==5)		Angle2_init-=step;     //云台上的第1个自由度
	 else if(PS2_KEY==7)		Angle2_init+=step;   
		
	 else if(PS2_KEY==6)		Angle3_init-=step;   //云台上的第2个自由度
	 else if(PS2_KEY==8)		Angle3_init+=step;   
		
	 else if(PS2_KEY==13)		Angle4_init-=step;   //云台上的第3个自由度
	 else if(PS2_KEY==15)		Angle4_init+=step;
	
	 else if(PS2_KEY==14)		Angle5_init+=step;   //控制机械手旋转的舵机
	 else if(PS2_KEY==16)		Angle5_init-=step;	
}
/**************************************************************************
函数功能：舵机初始位置调整模式检测函数
入口参数：无
返回  值：无
**************************************************************************/
void Servo_init_angle_adjust_mode_check(void)
{
	static int check_count1=0,check_count2=0;
	
		if(PS2_LX<100 && PS2_RX>200)   check_count1++;  //手柄左右两边摇杆往左右两边推
	  else check_count1=0;
	
		if(PS2_LX>200 && PS2_RX<100)   check_count2++;  //手柄左右两边摇杆往中间推
	  else check_count2=0;
	//进入模式检测
	if(check_count1>500)//连续推住5秒
	{
		Beep_Control_Flag=1;  //蜂鸣器标志位置1，蜂鸣器响1秒
		Servo_init_angle_adjust=1; //进入舵机初始位置调整模式
		OLED_Refresh_Gram(); //刷新屏幕
    OLED_Clear();  //清除屏幕
		OLED_Refresh_Gram();
		check_count1=0;
		
	}
		//退出模式检测
	if(check_count2>500 && Servo_init_angle_adjust==1)//连续推住5秒
	{
	  Flash_Write();     //将调整好的值写入flash
		OLED_Clear();  //清除屏幕
		OLED_Refresh_Gram();//刷新屏幕
		Servo_init_angle_adjust=0;
		check_count2=0;
	}

}


void Servo_init_pose(u8 j1,u8 j2,u8 j3,u8 j4,u8 j5,u8 j6)
{
	int step=5;
	if(j1==1)   		Angle1_init+=step;
	else if(j1==2)  Angle1_init-=step;
	
	if(j2==1)   		Angle2_init+=step;
	else if(j2==2)  Angle2_init-=step;
	
	if(j3==1)   		Angle3_init+=step;
	else if(j3==2)  Angle3_init-=step;
	
	if(j4==1)   		Angle4_init+=step;
	else if(j4==2)  Angle4_init-=step;
	
	if(j5==1)   		Angle5_init+=step;
	else if(j5==2)  Angle5_init-=step;
	
	if(j6==1)   		Angle6_init+=step;
	else if(j6==2)  Angle6_init-=step;
	
}



