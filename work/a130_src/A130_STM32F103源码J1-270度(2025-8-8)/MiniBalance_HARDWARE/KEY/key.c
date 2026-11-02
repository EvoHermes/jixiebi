#include "key.h"
u8 Beep_Control_Flag=0;
/**************************************************************************
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;	            //端口配置
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
//  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
//	
//	PWR_BackupAccessCmd(ENABLE);//允许修改RTC 和后备寄存器
//	RCC_LSEConfig(RCC_LSE_OFF);//关闭外部低速外部时钟信号功能 后，PC13 PC14 PC15 才可以当普通IO用。
//	BKP_TamperPinCmd(DISABLE);//关闭入侵检测功能，也就是 PC13，也可以当普通IO 使用
//	PWR_BackupAccessCmd(DISABLE);//禁止修改后备寄存器
} 

void BEEP_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3;	           //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);					     //根据设定参数初始化GPIO
	BEEP=0;
} 
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击 
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	  if(KEY==0)  Forever_count++;   //长按标志位未置1
     else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2;//双击执行的指令
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1;//单击执行的指令
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作  
**************************************************************************/
u8 click(void)
{     
	    int  temp;
			static u8 flag_key=1;//按键按松开标志
			if(flag_key&&KEY==0)
			{
			flag_key=0;
			if(KEY==0)  temp=1;
	
			return temp;	  // 按键按下
			}
			else if(1==KEY)			flag_key=1;
			return 0;//无按键按下
}
/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
			static u16 Long_Press_count=0,Long_Press=0;
	    if(Long_Press==0&&KEY==0)  Long_Press_count++;   //长按标志位未置1
      else                       Long_Press_count=0; 
		  if(Long_Press_count>200)		
			{
				Long_Press=1;	
				Long_Press_count=0;
				return 1;
			}				
			 if(Long_Press==1)     //长按标志位置1
			{
				  Long_Press=0;
			}
			return 0;
}

//void beep_ring(int time)
//{
//	static int count=0;
//	static int target_time=100;
//	if(count == 0) target_time=100,count=1;
//	if(target_time>0) BEEP=1,target_time--;
//	else BEEP=0;
//}

void beep_ring(void)
{  
	static int count=50; //开机响0.5s
  if(Beep_Control_Flag==1)  count=100,Beep_Control_Flag=0; //每次标志位Beep_Control_Flag被置1的时候，蜂鸣器响1秒
	if(count>0) count--,BEEP=1;
	else BEEP=0;
}

void beep_flash(int count)
{
	  static int time=0;
	  if(0==count) BEEP=0;
	  else if(++time==count)	BEEP=~BEEP,time=0;
	
}
