#ifndef STM32_VISUAL_XY_H
#define STM32_VISUAL_XY_H

#include "sys.h"

/* Call this once for every received byte from the vision UART ISR. */
void VisualXY_RxByte(u8 byte);

/* Call this from the existing 10 ms control task/interrupt. */
void VisualXY_Task(void);

/* Optional status for an OLED/debug screen. */
extern volatile u8 visual_xy_valid;
extern volatile u8 visual_xy_enabled;

#endif
