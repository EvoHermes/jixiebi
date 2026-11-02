#include "show.h"
unsigned char m;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{   
	if(Clear_Oled_Flag==1) //双击用户按键清除屏幕
	{
    OLED_Clear();  //清除屏幕
		//delay_ms(500);
		OLED_Clear();  //清除屏幕
		OLED_Refresh_Gram(); //刷新屏幕
	  Clear_Oled_Flag=0;
	}

	if(mode)
	{
		if(!ik_flag)
		{
					OLED_ShowString(103,48,"!");
		}
		else
		{
			OLED_Clear_rect(103,48,108,60);
		}
		
		OLED_ShowString(0,0,"x:");
		OLED_ShowString(0,20,"y:");
		OLED_ShowString(0,40,"z:");
		OLED_ShowString(55,0,"rx:");
		OLED_ShowString(55,20,"ry:");
		
		OLED_ShowString(110,48,"ik"),
		OLED_Showdecimal(20,0,  px,1,2,12,1); 
		OLED_Showdecimal(20,20, py,1,2,12,1);          	
		OLED_Showdecimal(20,40, pz,1,2,12,1);          	

		OLED_Showdecimal(80,0,  rx,1,2,12,1); 
		OLED_Showdecimal(80,20, ry,1,2,12,1);
	}
	else
	{
		
		OLED_ShowString(0,0,"1:");
		OLED_ShowString(0,20,"2:");
		OLED_ShowString(0,40,"3:");
		OLED_ShowString(55,0,"4: ");
		OLED_ShowString(55,20,"5: ");
		
		OLED_ShowString(110,48,"fk"),
		OLED_Showdecimal(20,0,  move_angle[1],1,2,12,1); 
		OLED_Showdecimal(20,20, move_angle[2],1,2,12,1);          	
		OLED_Showdecimal(20,40, move_angle[3],1,2,12,1);          	

		OLED_Showdecimal(80,0,  move_angle[4],1,2,12,1); 
		OLED_Showdecimal(80,20, move_angle[5],1,2,12,1);    
	}
	
	
	//=============第1行显示内容=======================//	
          	
	
//		OLED_Showdecimal(20,0,  move_angle[1],1,2,12,1); 
//		OLED_Showdecimal(20,20, move_angle[2],1,2,12,1);          	
//		OLED_Showdecimal(20,40, move_angle[3],1,2,12,1);          	

//		OLED_Showdecimal(60,0,  move_angle[4],1,2,12,1); 
//		OLED_Showdecimal(60,20, move_angle[5],1,2,12,1);          	
//	
//	
	
	
//		OLED_Showdecimal(60,40, move_angle[3],1,2,12,1);          	

//		OLED_Showdecimal(100,0,  move_angle[4],1,2,12,1); 
//		OLED_Showdecimal(100,20, move_angle[5],1,2,12,1);          	
//	
//		OLED_Showdecimal(100,40, mode,1,2,12,1);
//else //舵机初始位置校正模式显示
//{
//	//=============第1行显示内容=======================//	
//		  if( move_angle[0]_init<0)		    OLED_ShowString(00,00,"-"),
//		                      OLED_ShowNumber(15,00,-move_angle[0]_init,4,12);
//		else                 	OLED_ShowString(0,00,"+"),
//		                      OLED_ShowNumber(15,00, move_angle[0]_init,4,12); 
//    OLED_ShowString(60,00,"SERVO");
//	//=============第2行显示内容=======================//	
//	  if( Angle2_init<0)		    OLED_ShowString(00,10,"-"),
//		                      OLED_ShowNumber(15,10,-Angle2_init,4,12);
//		else                 	OLED_ShowString(0,10,"+"),
//		                      OLED_ShowNumber(15,10, Angle2_init,4,12); 
//		OLED_ShowString(60,10,"INIT");
//	//=============第3行显示内容=======================//	
//		 if( Angle3_init<0)		  OLED_ShowString(00,20,"-"),
//		                      OLED_ShowNumber(15,20,-Angle3_init,4,12);
//		else                 	OLED_ShowString(0,20,"+"),
//		                      OLED_ShowNumber(15,20, Angle3_init,4,12); 
//		OLED_ShowString(60,20,"ANGLE");  
//	//=============第4行显示内容=======================//	
//		  if( Angle4_init<0)		  OLED_ShowString(00,30,"-"),
//		                      OLED_ShowNumber(15,30,-Angle4_init,4,12);
//		else                 	OLED_ShowString(0,30,"+"),
//		                      OLED_ShowNumber(15,30, Angle4_init,4,12); 
//		
//    OLED_ShowString(60,30,"ADJUST"); 
//		  
//	//=============第5行显示内容=======================//	
//		  if( Angle5_init<0)	  	OLED_ShowString(00,40,"-"),
//		                      OLED_ShowNumber(15,40,-Angle5_init,4,12);
//		else                 	OLED_ShowString(0,40,"+"),
//		                      OLED_ShowNumber(15,40, Angle5_init,4,12); 
//	  OLED_ShowString(60,40,"MODE"); 	
//		
//	//=============第6行显示内容=======================//	
//    if( move_angle[6]_init<0)	    	OLED_ShowString(00,50,"-"),
//		                      OLED_ShowNumber(15,50,-move_angle[6]_init,4,12);
//		else                 	OLED_ShowString(0,50,"+"),
//		                      OLED_ShowNumber(15,50, move_angle[6]_init,4,12); 
//	
OLED_Refresh_Gram();	

}
		//=============刷新=======================//
		

/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void APP_Show(void)
{    
		static u8 flag;
	  int app_2,app_3,app_4;
		app_4=(Voltage-1110)*2/3;		if(app_4<0)app_4=0;if(app_4>100)app_4=100;   //对电压数据进行处理
//    app_3=Moto1/50; if(app_3<0)app_3=-app_3;			                           //对编码器数据就行数据处理便于图形化
//		app_2=Moto2/50;  if(app_2<0)app_2=-app_2;
	  flag=!flag;
	if(PID_Send==1)//发送参数界面的参数
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
		  (int)((move_angle[0]+1.57)*100), //+1.57是因为调参界面不可以显示负数，*100是因为调参界面不现实浮点数的点数部分
			(int)((move_angle[2]+1.57)*100),
			(int)((move_angle[3]+1.57)*100),
			(int)((move_angle[4]+1.57)*100),
			(int)((move_angle[5]+1.57)*100),
			(int)((move_angle[6]+1.57)*100),
			(int)(Position_KP),
			(int)(Position_KD),
			(int)(Speed));//打印到APP上面	
		PID_Send=0;	 
	}	
   else	if(flag==0)// 
   printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,(u8)app_4,0);//打印到APP上面
	 else
	 printf("{B%d:%d:%d:%d:%d:%d}$",(int)Position1,(int)Position2,(int)Position3,(int)Position4,(int)Position5,(int)Position6);//打印到APP上面 显示波形
}
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DataScope(void)
{   
    Vol=(float)Voltage/100;
		DataScope_Get_Channel_Data(Position1, 1 );   //显示角度 单位：度（°）
		DataScope_Get_Channel_Data(Position2, 2 );   //显示超声波测量的距离 单位：CM 
		DataScope_Get_Channel_Data(Position3, 3 );   //显示电池电压 单位：V
		DataScope_Get_Channel_Data(Position4, 4 );   
		DataScope_Get_Channel_Data(Position5, 5 ); //用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(5);
		for( m = 0 ; m < Send_Count; m++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[m]; 
		}
}
