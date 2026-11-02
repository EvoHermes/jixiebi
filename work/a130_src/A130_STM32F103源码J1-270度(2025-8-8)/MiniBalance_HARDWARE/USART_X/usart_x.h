#ifndef __USRATX_H
#define __USRATX_H 
#include "sys.h"	  	
extern u8 Usart3_Receive;
extern u8 pose_arm;
extern u8 Flag_Left,Flag_Right,Flag_Direction,Turn_Flag;//蓝牙遥控相关的标志位
extern u8 uasrt_sent[16];
void data_transition(void);
void uart3_init(u32 bound);
int  USART3_IRQHandler(void);
int USART1_IRQHandler(void);
void uart1_init(u32 bound);
void usart1_send(u8 data);
void usart3_send(u8 data);
void Vision_SendDone(void);
void usart1_send_task(void);
u8 Check_Sum(u8 *array);
#define FRAME_HEADER_MOVEIT 0XAA //控制机械臂运动的数据帧头
#define FRAME_TAIL_MOVEIT 0XBB //控制机械臂运动的数据帧头
#define follower 2
#define default_mode 1
#endif

