/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
版本V1.0
修改时间：2021-11-04
All rights reserved
***********************************************/
#include "sys.h"
float Speed=VISION_TRACK_SPEED;                   //舵机默认运动上限速度
float	Position_KP=10,Position_KI=0,Position_KD=18;  //位置控制PID参数 
u8 delay_50,delay_flag,PID_Send,Flash_Send; //延时和调参等变量
u8 PS2_ON_Flag=0, APP_ON_Flag=0, CAN_ON_Flag=0;  //控制模式标志位
int Servo_init_angle_adjust=0,start_count=0; //舵机初始位置校正相关变量
u8 one_hundred_ms_flag=0,Clear_Oled_Flag=0;
int main(void)
{ 
		JTAG_Set(JTAG_SWD_DISABLE);
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
		KEY_Init();                     //=====按键初始化
 		MY_NVIC_PriorityGroupConfig(2); //=====中断分组
		delay_init();	    	            //=====延时函数初始化	
		BEEP_Init();                    //=====蜂鸣器初始化
		LED_Init();                     //=====初始化LED
		uart1_init(115200);             //=====串口1初始化
		uart3_init(115200);             //=====串口3初始化
		Adc_Init();                     //=====adc初始化
    OLED_Init();                    //=====OLED初始化	
		TIM3_PWM_Init(9999,71);         //=====PWM初始化
		TIM4_PWM_Init(9999,71);         //=====PWM初始化
		PS2_Init();                     //=====PS2手柄初始化
		PS2_SetInit();		 							//=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
		TIM2_Int_Init(99,7199);         //=====定时器10ms中断
    while(1)
	  {
      read_ps2_data();  //===获取手柄的控制信号
			oled_show();      //===显示屏内容显示
			if (Vision_TakeDoneFlag()) Vision_SendDone();
			
			
		} 
}

