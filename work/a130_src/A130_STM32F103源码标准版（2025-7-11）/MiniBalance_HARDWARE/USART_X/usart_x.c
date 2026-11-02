#include "usart_x.h"
u8 Usart3_Receive;
u8 Flag_Left,Flag_Right,Flag_Direction=0,Turn_Flag;//蓝牙遥控相关的标志位
u8 uasrt_sent[16];
u8 pose_arm;
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

	//USART_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	 //Reuse push-pull output //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
  //USART_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //使能串口1
}
/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
函数功能：串口1接收中断
入口参数：无
返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{	
  static u8 Count_moveit;
	static u8 Count_init;
	static char rxbuf_moveit[16];
	u8 Usart_Receive;
	int check=0,error=1,i;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //判断是否接收到数据
	{
		
		Usart_Receive = USART_ReceiveData(USART1);//读取数据
		
		if(Usart_Receive==0xA1 || Count_init>0)
		{
			rxbuf_moveit[Count_init]=Usart_Receive;
			Count_init++;
		}
		else if(Usart_Receive == 0xAA||Count_moveit>0) 
		{ 
			rxbuf_moveit[Count_moveit]=Usart_Receive;
 			Count_moveit++;
		}
		else
		{
			 Count_moveit=0; //接收控制机械臂的数据帧
		   Count_init=0;
		}
	 if(Count_init ==10)
	 {
	   
		 Count_init=0;
		 if(rxbuf_moveit[9]==0xF1)
		 {
			 for(i=0;i<8;i++)
			 {
				 check=rxbuf_moveit[i]^check;
			 }
			 if(check==rxbuf_moveit[8])
			 {
				 APP_ON_Flag=0,PS2_ON_Flag=0, CAN_ON_Flag=0;
				 if(rxbuf_moveit[7])
				 {
					 pose_arm=1;
				 }
				 else
					{
						Servo_init_pose(rxbuf_moveit[1],rxbuf_moveit[2],rxbuf_moveit[3],
														rxbuf_moveit[4],rxbuf_moveit[5],rxbuf_moveit[6]);						
					}
			 }
		 }
	 }
	 if (Count_moveit == 16)	//验证数据包的长度
		{   
				Count_moveit=0;//重新开始接收
				if(rxbuf_moveit[15] == FRAME_TAIL_MOVEIT) //验证数据包的尾部校验信息
				{ 
					for(i=0; i<14; i++)
					{
						check=rxbuf_moveit[i]^check; //异或，用于检测数据是否出错
					}
					if(check==rxbuf_moveit[14]) error=0; //检验成功
					
					if(error==0)	 //数据校验位计算
				  {	
						APP_ON_Flag=0,PS2_ON_Flag=0, CAN_ON_Flag=0;
						
						move_angle[1]=(short)((rxbuf_moveit[1]<<8)+(rxbuf_moveit[2]));   //取第1个舵机的目标角度 分高8位和低8位 单位(弧度)
						move_angle[2]=(short)((rxbuf_moveit[3]<<8)+(rxbuf_moveit[4]));   //取第2个舵机的目标角度 分高8位和低8位 单位(弧度)
						move_angle[3]=(short)((rxbuf_moveit[5]<<8)+(rxbuf_moveit[6]));   //取第3个舵机的目标角度 分高8位和低8位 单位(弧度)
						move_angle[4]=(short)((rxbuf_moveit[7]<<8)+(rxbuf_moveit[8]));   //取第4个舵机的目标角度 分高8位和低8位 单位(弧度)
						move_angle[5]=(short)((rxbuf_moveit[9]<<8)+(rxbuf_moveit[10]));  //取第5个舵机的目标角度 分高8位和低8位 单位(弧度)
						move_angle[6]=(short)((rxbuf_moveit[11]<<8)+(rxbuf_moveit[12])); //取第6个舵机的目标角度 分高8位和低8位 单位(弧度)
						
						move_angle[0]= move_angle[1]*0.001; //单位(弧度)，数据在发送前*1000，获取数据后需要做数据还原
						move_angle[2]= move_angle[2]*0.001; 
						move_angle[3]=-move_angle[3]*0.001;  //（rviz模型中运动方向相反，所以这里做了方向纠正将运动方向取反）
						move_angle[4]=-move_angle[4]*0.001; 
						move_angle[5]= move_angle[5]*0.001; 
						move_angle[6]= move_angle[6]*0.001;
						
						if(rxbuf_moveit[13]==follower)    Position_KP=0.6,Position_KI=0,Position_KD=6,Speed=6;  //跟随功能时的PID参数（舵机响应较慢），为了消除抖动
						else                              Position_KP=10,Position_KI=0,Position_KD=18,Speed=12;  //默认使用的PID参数（舵机效应较快）
				  }
			  }
		 }
		
	} 
return 0;											 
} 

/**************************************************************************
函数功能：串口3初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能UGPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3时钟
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{	

	int Usart_Receive;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{	      
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
		Usart_Receive=USART3->DR;
		
    if(APP_ON_Flag==0) PS2_ON_Flag=0, CAN_ON_Flag=0,APP_ON_Flag=1;
		
		if(Usart_Receive==0x58)  Speed=Speed+1; //加速按键
		if(Usart_Receive==0x59)  Speed=Speed-1; //减速按键
	 if(Usart_Receive==0x61)    //机械臂位置复位
	 {
		 move_angle[1]=0,move_angle[2]=0,move_angle[3]=0,move_angle[4]=0,move_angle[5]=0,move_angle[6]=0;
	 }
		
	 //以下是与APP调试界面通讯
	 if(Usart_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
	 if(Usart_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位

	 if(Flag_PID==1)  //采集数据
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2)  //分析数据
	 {
					if(Receive[3]==0x50) 	 PID_Send=1;
				 else  if(Receive[1]!=0x23) 
				 {								
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
					//使用调参界面拖动滑轨来控制机械臂运动
					switch(Receive[1])
					 {
						 case 0x30:  move_angle[1]=Data*0.01-1.57;break; //*0.01是因为参数在发送前是放大了一百倍的，这里需要做还原
						 case 0x31:  move_angle[2]=Data*0.01-1.57;break; //-1.57是因为参数在发送前加了1.57，这里需要做还原
						 case 0x32:  move_angle[3]=Data*0.01-1.57;break;
						 case 0x33:  move_angle[4]=Data*0.01-1.57;break;
						 case 0x34:  move_angle[5]=Data*0.01-1.57;break;
						 case 0x35:  move_angle[6]=Data*0.01-1.57;break;
						 case 0x36:  Position_KP=Data;break;
						 case 0x37:  Position_KD=Data;break;
						 case 0x38:  Speed=Data;break; 		//预留
					 }
				 }				 
				 Flag_PID=0;//相关标志位清零
				 i=0;
				 j=0;
				 Data=0;
				 memset(Receive, 0, sizeof(u8)*50);//数组清零
	 }
	 
   }
return 0;										 
} 

void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
函数功能：串口将欧数据发送前，将要发送的数据拆分放进数组里面
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
	 uasrt_sent[0]=0x7b;  //帧头
	//6个关节的当前位置（单位：弧度），为了避免小数，放大一百倍发送数据
	 uasrt_sent[1]=(short)(PWM_Angle_VALUE(Position1)*100)>>8;  //高八位数据
	 uasrt_sent[2]=(short)(PWM_Angle_VALUE(Position1)*100);     //低八位数据
	
	 uasrt_sent[3]=(short)(PWM_Angle_VALUE(Position2)*100)>>8;
	 uasrt_sent[4]=(short)(PWM_Angle_VALUE(Position2)*100);
	
	 uasrt_sent[5]=(short)(PWM_Angle_VALUE(Position3)*100)>>8;
	 uasrt_sent[6]=(short)(PWM_Angle_VALUE(Position3)*100);
	
	 uasrt_sent[7]=(short)(PWM_Angle_VALUE(Position4)*100)>>8;
	 uasrt_sent[8]=(short)(PWM_Angle_VALUE(Position4)*100);
	
	 uasrt_sent[9]=(short)(PWM_Angle_VALUE(Position5)*100)>>8;
	 uasrt_sent[10]=(short)(PWM_Angle_VALUE(Position5)*100);
	
	 uasrt_sent[11]=(short)(PWM_Angle_VALUE(Position6)*100)>>8;
	 uasrt_sent[12]=(short)(PWM_Angle_VALUE(Position6)*100);
	 uasrt_sent[13]=Speed;             //舵机运行速度
	 uasrt_sent[14]=Check_Sum(uasrt_sent); //检验位
	 uasrt_sent[15]=0x7d; //帧尾

}
/**************************************************************************
函数功能：校验位计算
入口参数：需要计算校验位的数组
返回  值：无
**************************************************************************/
u8 Check_Sum(u8 *array)
{
	u8 result=0;
  int k=0;
	for(k=0;k<14;k++)
	{
	result=result^array[k];
	}
	return result;	
}
/**************************************************************************
函数功能：串口1的发送函数
入口参数：无
返回  值：无
**************************************************************************/
void usart1_send_task(void)
{
	int num=16;
	int i;
	for(i=0; i<num; i++)
	{
		usart1_send(uasrt_sent[i]);
	}	 
}
