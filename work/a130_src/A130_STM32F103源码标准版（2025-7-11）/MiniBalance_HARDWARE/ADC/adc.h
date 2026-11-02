#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#define Battery_Ch 1
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
void  ccd_Init(void);
extern int Voltage;                //电池电压采样相关的变量
#define FILTER_TIMES  20
#endif 

























