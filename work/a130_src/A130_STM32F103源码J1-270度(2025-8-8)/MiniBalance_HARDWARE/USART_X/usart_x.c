#include "usart_x.h"
#include "control.h"
u8 Usart3_Receive;
u8 Flag_Left,Flag_Right,Flag_Direction=0,Turn_Flag;//����ң����صı�־λ
u8 uasrt_sent[16];
u8 pose_arm;
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
�������ܣ�����1��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //Enable the gpio clock //ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	//USART_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	 //Reuse push-pull output //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
  //USART_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //��ʼ������1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //ʹ�ܴ���1
}
/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
�������ܣ�����1�����ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/
int USART1_IRQHandler(void)
{	
  static u8 Count_moveit;
	static u8 Count_init;
	static char rxbuf_moveit[16];
	u8 Usart_Receive;
	int check=0,error=1,i;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�ж��Ƿ���յ�����
	{
		
		Usart_Receive = USART_ReceiveData(USART1);//��ȡ����
		
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
			 Count_moveit=0; //���տ��ƻ�е�۵�����֡
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
	 if (Count_moveit == 16)	//��֤���ݰ��ĳ���
		{   
				Count_moveit=0;//���¿�ʼ����
				if(rxbuf_moveit[15] == FRAME_TAIL_MOVEIT) //��֤���ݰ���β��У����Ϣ
				{ 
					for(i=0; i<14; i++)
					{
						check=rxbuf_moveit[i]^check; //������ڼ�������Ƿ����
					}
					if(check==rxbuf_moveit[14]) error=0; //����ɹ�
					
					if(error==0)	 //����У��λ����
				  {	
						APP_ON_Flag=0,PS2_ON_Flag=0, CAN_ON_Flag=0;
						
						move_angle[1]=(short)((rxbuf_moveit[1]<<8)+(rxbuf_moveit[2]));   //ȡ��1�������Ŀ��Ƕ� �ָ�8λ�͵�8λ ��λ(����)
						move_angle[2]=(short)((rxbuf_moveit[3]<<8)+(rxbuf_moveit[4]));   //ȡ��2�������Ŀ��Ƕ� �ָ�8λ�͵�8λ ��λ(����)
						move_angle[3]=(short)((rxbuf_moveit[5]<<8)+(rxbuf_moveit[6]));   //ȡ��3�������Ŀ��Ƕ� �ָ�8λ�͵�8λ ��λ(����)
						move_angle[4]=(short)((rxbuf_moveit[7]<<8)+(rxbuf_moveit[8]));   //ȡ��4�������Ŀ��Ƕ� �ָ�8λ�͵�8λ ��λ(����)
						move_angle[5]=(short)((rxbuf_moveit[9]<<8)+(rxbuf_moveit[10]));  //ȡ��5�������Ŀ��Ƕ� �ָ�8λ�͵�8λ ��λ(����)
						move_angle[6]=(short)((rxbuf_moveit[11]<<8)+(rxbuf_moveit[12])); //ȡ��6�������Ŀ��Ƕ� �ָ�8λ�͵�8λ ��λ(����)
						
						move_angle[1]= move_angle[1]*0.001f; //��λ(����)�������ڷ���ǰ*1000����ȡ���ݺ���Ҫ�����ݻ�ԭ
						move_angle[2]= move_angle[2]*0.001; 
						move_angle[3]=-move_angle[3]*0.001;  //��rvizģ�����˶������෴�������������˷���������˶�����ȡ����
						move_angle[4]=-move_angle[4]*0.001; 
						move_angle[5]= move_angle[5]*0.001; 
						move_angle[6]= move_angle[6]*0.001;
						
						if(rxbuf_moveit[13]==follower)    Position_KP=0.6,Position_KI=0,Position_KD=6,Speed=6;  //���湦��ʱ��PID�����������Ӧ��������Ϊ����������
						else                              Position_KP=10,Position_KI=0,Position_KD=18,Speed=12;  //Ĭ��ʹ�õ�PID���������ЧӦ�Ͽ죩
				  }
			  }
		 }
		
	} 
return 0;											 
} 

/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��UGPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3ʱ��
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//USART3 priority higher than TIM2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
static u8 vision_frame_index = 0;
static u8 vision_frame_buf[7];

static void Vision_ProcessFrame(const u8 *f)
{
    u8 sum = (u8)((0xEF + 0xFE + f[2] + f[3] + f[4] + f[5]) & 0xFF);
    int x, y;

    if (f[6] != sum) return;
    x = (int)f[2] | ((int)f[3] << 8);
    y = (int)f[4] | ((int)f[5] << 8);
    Vision_SetTarget(x, y);
}

int USART3_IRQHandler(void)
{
    u8 b;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == RESET) return 0;
    b = (u8)USART3->DR;
    if (vision_frame_index == 0 && b != 0xEF) return 0;
    if (vision_frame_index == 1 && b != 0xFE) {
        vision_frame_index = (b == 0xEF) ? 1 : 0;
        return 0;
    }
    vision_frame_buf[vision_frame_index++] = b;
    if (vision_frame_index >= 7) {
        vision_frame_index = 0;
        Vision_ProcessFrame(vision_frame_buf);
    }
    return 0;
}

void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);
}
void Vision_SendDone(void)
{
	usart3_send(0xEF);
	usart3_send(0xFE);
	usart3_send(0x7E);
	usart3_send(0x6B);
}
/**************************************************************************
�������ܣ����ڽ�ŷ���ݷ���ǰ����Ҫ���͵����ݲ�ַŽ���������
��ڲ�������
����  ֵ����
**************************************************************************/
void data_transition(void)
{
	 uasrt_sent[0]=0x7b;  //֡ͷ
	//6���ؽڵĵ�ǰλ�ã���λ�����ȣ���Ϊ�˱���С�����Ŵ�һ�ٱ���������
	 uasrt_sent[1]=(short)(PWM_Angle_VALUE(Position1)*100)>>8;  //�߰�λ����
	 uasrt_sent[2]=(short)(PWM_Angle_VALUE(Position1)*100);     //�Ͱ�λ����
	
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
	 uasrt_sent[13]=Speed;             //��������ٶ�
	 uasrt_sent[14]=Check_Sum(uasrt_sent); //����λ
	 uasrt_sent[15]=0x7d; //֡β

}
/**************************************************************************
�������ܣ�У��λ����
��ڲ�������Ҫ����У��λ������
����  ֵ����
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
�������ܣ�����1�ķ��ͺ���
��ڲ�������
����  ֵ����
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
