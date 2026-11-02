#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "vision_config.h"

int EXTI15_10_IRQHandler(void);
void Key(void);
void Servo_angle_check(void);
void Servo_Pwm_check(void);
void Servo_Speed_Check(void);
u8 Turn_Off( int voltage);
void Get_Angle(u8 way);
int myabs(int a);
void PS2_Control(float Step);
int SERVO_PWM_VALUE_180(float angle);
int SERVO_PWM_VALUE_270(float angle);

	float PWM_Angle_VALUE(int pwm);
float Position_PID1 (float Encoder,float Target);
float Position_PID2 (float Encoder,float Target);
float Position_PID3 (float Encoder,float Target);
float Position_PID4 (float Encoder,float Target);
float Position_PID5 (float Encoder,float Target);
float Position_PID6 (float Encoder,float Target);
void Set_Pwm(float velocity1,float velocity2,float velocity3,float velocity4,float velocity5,float velocity6);
void Servo_init_angle_adjust_function(void);
void Servo_init_angle_adjust_mode_check(void);
void Servo_init_pose(u8 j1,u8 j2,u8 j3,u8 j4,u8 j5,u8 j6);
void CAN_Control(void);
void save_init_pose(void);
void ik_kinematics(float x,float y,float z,float rx,float ry);
void Vision_SetTarget(int target_x, int target_y);
void Vision_SetNoTarget(void);
void fk_kinematics(float angle1,float angle2,float angle3,float angle4,float angle5);
u8 Vision_TakeDoneFlag(void);

extern float px,py,pz,rx,ry;
extern float T05[4][4];

extern u8 mode;
extern u8 ik_flag;//����־λ���н�Ϊ1���޽�Ϊ0

extern int Servo1,Servo2,Servo3,Servo4,Servo5,Servo6;
extern float Position1,Position2,Position3,Position4,Position5,Position6; //PID������ر���    
extern float Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6;     //���PWM����
extern float move_angle[7];
extern short Angle1_init,Angle2_init,Angle3_init,Angle4_init,Angle5_init,Angle6_init;
#endif
