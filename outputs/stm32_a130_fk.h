#ifndef STM32_A130_FK_H
#define STM32_A130_FK_H

#include "sys.h"

/*
 * Forward kinematics in the A130 base frame.
 * j1..j5 are geometric joint angles in radians, not raw PWM values.
 * J6 is the gripper opening and does not change the wrist position.
 */
void A130_FK(float j1, float j2, float j3, float j4, float j5,
             float *x, float *y, float *z);

#endif
