/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
版本V1.0
修改时间�?021-11-04
All rights reserved
***********************************************/
#include "control.h"	
#include <math.h>
#include "stdlib.h"

#define ARM_HOME_J1 (-0.7853981633974483f)  // -45
#define ARM_HOME_J2 1.047197f    //60 degrees
#define ARM_HOME_J3 (-1.57079f) /* -90 degrees */
#define ARM_HOME_J4 (-1.0471975511965976f)   /* -60degrees, inside the firmware limit */
#define ARM_HOME_J5 0.0f   
/* This hardware's J6 direction is reversed: 0.0 closes, 0.70 opens. */
#define ARM_GRIPPER_OPEN 0.70f
#define ARM_GRIPPER_CLOSE 0.0f
#define ARM_STARTUP_TICKS 300U
#define ARM_APPROACH_TICKS 250U  /* 2.5 s at 10 ms control period */
#define ARM_RETURN_TICKS 150U   /* 1.5 s at 10 ms control period */
#define ARM_GRIPPER_TICKS 80U

u8 mode=0;
u8 ik_flag=1;
int Servo1=0,Servo2=0,Servo3=0,Servo4=0,Servo5=0,Servo6=0; //舵机PWM变量
float Velocity1=0,Velocity2=0,Velocity3=0,Velocity4=0,Velocity5=0,Velocity6=0;     //电机PWM变量
float Position1=1500,Position2=1500,Position3=1500,Position4=1500,Position5=1500,Position6=1500;   
float move_angle[7]={0,ARM_HOME_J1,ARM_HOME_J2,ARM_HOME_J3,ARM_HOME_J4,ARM_HOME_J5,ARM_GRIPPER_OPEN};
short Angle1_init=0,Angle2_init=0,Angle3_init=0,Angle4_init=0,Angle5_init=0,Angle6_init=0; //舵机初始位置�?
float px=0,py=0,pz=0,rx=0,ry=0;

typedef enum {
    ARM_STARTUP_HOME = 0,
    ARM_TRACK_TARGET,
    ARM_GRASP_OPEN,
    ARM_WAIT_GRIPPER_OPEN,
    ARM_APPROACH_TARGET,
    ARM_WAIT_APPROACH_SETTLE,
    ARM_CLOSE_GRIPPER,
    ARM_HOLD_CLOSED_GRIPPER,
    ARM_RETURN_WITH_MANGO,
    ARM_RETURN_EMPTY,
    ARM_WAIT_RETURN_SETTLE,
    ARM_HOLD_AT_HOME
} VisionArmState;

static volatile int vision_target_valid = 0;
static volatile int vision_target_x = VISION_CENTER_X;
static volatile int vision_target_y = VISION_CENTER_Y;
static volatile unsigned int vision_age_ticks = 0;
static volatile u8 vision_target_pending = 0;
static volatile unsigned int vision_center_frames = 0;
static volatile u8 vision_pid_reset_pending = 0;
static volatile u8 vision_done_flag = 0;
static u8 vision_done_sent = 0;
static VisionArmState vision_arm_state = ARM_STARTUP_HOME;
static unsigned int vision_state_ticks = 0;
static unsigned int vision_settle_ticks = 0;
static unsigned int vision_rearm_silence_ticks = 0;
static u8 vision_rearm_ready = 0;
static float return_j1, return_j2, return_j3, return_j4;
static float aim_j1, aim_j2, aim_j3, aim_j4, end_j2, end_j3, end_j4;

typedef struct {
    float integral;
    float derivative;
    int previous_error;
    u8 initialized;
} VisionPID;

static VisionPID vision_pid_j1 = {0.0f, 0.0f, 0, 0};
static VisionPID vision_pid_j4 = {0.0f, 0.0f, 0, 0};
static unsigned int vision_track_elapsed_ticks = 0;
static void Vision_ResetPID(void);

/* USART3 publishes positions only; TIM2 owns motion and speed changes. */
void Vision_SetTarget(int target_x, int target_y)
{
    int centered;
    if (target_x < 0 || target_x > VISION_FRAME_WIDTH ||
        target_y < 0 || target_y > VISION_FRAME_HEIGHT) {
        Vision_SetNoTarget();
        return;
    }
    centered = abs(target_x - VISION_CENTER_X) <= VISION_CENTER_TOLERANCE_PX &&
               abs(target_y - VISION_CENTER_Y) <= VISION_CENTER_TOLERANCE_PX;
    if (!vision_target_valid || vision_age_ticks > VISION_TIMEOUT_TICKS || !centered)
        vision_center_frames = 0;
    if (centered && vision_center_frames < VISION_CENTER_REQUIRED_FRAMES)
        vision_center_frames++;
    vision_target_x = target_x;
    vision_target_y = target_y;
    vision_age_ticks = 0;
    vision_target_valid = 1;
    vision_target_pending = 1;
}

void Vision_SetNoTarget(void)
{
    unsigned int irq_mask = __get_PRIMASK();
    __disable_irq();
    vision_target_valid = 0;
    vision_target_pending = 0;
    vision_center_frames = 0;
    vision_age_ticks = 0;
    vision_pid_reset_pending = 1;
    __set_PRIMASK(irq_mask);
}

extern float angle_range[5][2];

static float Vision_Limit(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void Vision_ResetPID(void)
{
    vision_pid_j1.integral = 0.0f;
    vision_pid_j1.derivative = 0.0f;
    vision_pid_j1.previous_error = 0;
    vision_pid_j1.initialized = 0;
    vision_pid_j4.integral = 0.0f;
    vision_pid_j4.derivative = 0.0f;
    vision_pid_j4.previous_error = 0;
    vision_pid_j4.initialized = 0;
}

/* Image-space PID updates the commanded joint angle. There is no joint encoder
   feedback here; damping comes from the measured change in target pixel error. */
static float Vision_PIDUpdate(int error, float dt, float current,
                              float kp, float ki, float kd,
                              VisionPID *pid, float joint_min, float joint_max)
{
    float candidate_integral, raw_derivative, derivative;
    float velocity, integral_change, max_step, step, next_angle;
    int crossed_zero;

    if (error == 0) {
        pid->integral = 0.0f;
        pid->derivative = 0.0f;
        pid->previous_error = 0;
        pid->initialized = 0;
        return current;
    }

    if (dt < 0.01f) dt = 0.01f;
    if (dt > 0.01f * (float)VISION_PID_MAX_DT_TICKS)
        dt = 0.01f * (float)VISION_PID_MAX_DT_TICKS;

    crossed_zero = pid->initialized &&
        ((error > 0 && pid->previous_error < 0) ||
         (error < 0 && pid->previous_error > 0));
    candidate_integral = crossed_zero ? 0.0f : pid->integral;
    candidate_integral += (float)error * dt;
    candidate_integral = Vision_Limit(candidate_integral,
        -VISION_PID_INTEGRAL_LIMIT_PX_S, VISION_PID_INTEGRAL_LIMIT_PX_S);

    if (pid->initialized) {
        raw_derivative = ((float)error - (float)pid->previous_error) / dt;
        derivative = pid->derivative + VISION_PID_D_FILTER_ALPHA *
            (raw_derivative - pid->derivative);
    } else {
        derivative = 0.0f;
    }

    velocity = kp * (float)error + ki * candidate_integral + kd * derivative;
    integral_change = candidate_integral - pid->integral;
    if (fabsf(velocity) > VISION_PID_MAX_RATE_RAD_S &&
        ((velocity > 0.0f && ki * integral_change > 0.0f) ||
         (velocity < 0.0f && ki * integral_change < 0.0f))) {
        /* Do not wind the integral farther into the rate limit. */
        candidate_integral = pid->integral;
        velocity = kp * (float)error + ki * candidate_integral +
                   kd * derivative;
    }

    max_step = VISION_PID_MAX_RATE_RAD_S * dt;
    step = Vision_Limit(velocity * dt, -max_step, max_step);
    next_angle = current + step;
    if ((next_angle > joint_max && velocity > 0.0f) ||
        (next_angle < joint_min && velocity < 0.0f)) {
        /* Clear stored effort at a hard joint stop to prevent windup. */
        candidate_integral = 0.0f;
        velocity = kp * (float)error + kd * derivative;
        step = Vision_Limit(velocity * dt, -max_step, max_step);
        next_angle = current + step;
    }
    next_angle = Vision_Limit(next_angle, joint_min, joint_max);

    pid->integral = candidate_integral;
    pid->derivative = derivative;
    pid->previous_error = error;
    pid->initialized = 1;
    return next_angle;
}

/* PositionN is the PWM command generated for the hobby servo, not a
   measured joint angle. These checks confirm command output has settled. */
static u8 Vision_GripperOutputSettled(void)
{
    return (u8)(fabsf(Position6 - (float)Servo6) <= VISION_SERVO_SETTLE_PWM);
}

static u8 Vision_ApproachOutputsSettled(void)
{
    return (u8)(fabsf(Position1 - (float)Servo1) <= VISION_SERVO_SETTLE_PWM &&
                fabsf(Position2 - (float)Servo2) <= VISION_SERVO_SETTLE_PWM &&
                fabsf(Position3 - (float)Servo3) <= VISION_SERVO_SETTLE_PWM &&
                fabsf(Position4 - (float)Servo4) <= VISION_SERVO_SETTLE_PWM &&
                fabsf(Position5 - (float)Servo5) <= VISION_SERVO_SETTLE_PWM);
}

static void Vision_BeginReturn(u8 holding_mango)
{
    return_j1 = move_angle[1];
    return_j2 = move_angle[2];
    return_j3 = move_angle[3];
    return_j4 = move_angle[4];
    vision_state_ticks = 0;
    vision_arm_state = holding_mango ? ARM_RETURN_WITH_MANGO : ARM_RETURN_EMPTY;
}

extern float angle_range[5][2];
static u8 Vision_ComputeApproach(void)
{
    float j1 = move_angle[1], j2 = move_angle[2];
    float j3 = move_angle[3], j4 = move_angle[4], j5 = move_angle[5];
    const float link2 = 0.105f, link3 = 0.0975f;
    float theta234 = j2 + j3 + j4;
    float s234 = sinf(theta234), c234 = cosf(theta234);
    float distance = ARM_APPROACH_DISTANCE_MM * 0.001f;
    float target_x, target_y, target_z;
    float link_m, link_n, cos_j3, root, best_cost = 1.0e9f;
    float best_j2 = 0.0f, best_j3 = 0.0f, best_j4 = 0.0f;
    float old_j2 = j2, old_j3 = j3, old_j4 = j4;
    int branch, found = 0;

    fk_kinematics(j1, j2, j3, j4, j5);
    target_x = px + T05[0][2] * distance;
    target_y = py + T05[1][2] * distance;
    target_z = pz + T05[2][2] * distance;

    link_m = link2 * sinf(j2) + link3 * sinf(j2 + j3) + s234 * distance;
    link_n = link2 * cosf(j2) + link3 * cosf(j2 + j3) + c234 * distance;
    cos_j3 = (link_m*link_m + link_n*link_n - link2*link2 - link3*link3) /
             (2.0f * link2 * link3);
    if (cos_j3 < -1.0f || cos_j3 > 1.0f) return 0;
    root = acosf(cos_j3);

    for (branch = 0; branch < 2; branch++) {
        float candidate_j3 = (branch == 0) ? root : -root;
        float p = link2 + link3 * cosf(candidate_j3);
        float q = link3 * sinf(candidate_j3);
        float denom = p*p + q*q;
        float candidate_j2 = atan2f(p*link_m - q*link_n,
                                    q*link_m + p*link_n);
        float candidate_j4 = theta234 - candidate_j2 - candidate_j3;
        float cost;

        if (candidate_j2 < angle_range[1][0] || candidate_j2 > angle_range[1][1] ||
            candidate_j3 < angle_range[2][0] || candidate_j3 > angle_range[2][1] ||
            candidate_j4 < angle_range[3][0] || candidate_j4 > angle_range[3][1] ||
            denom < 1.0e-8f) continue;
        cost = (candidate_j2-j2)*(candidate_j2-j2) +
               (candidate_j3-j3)*(candidate_j3-j3) +
               (candidate_j4-j4)*(candidate_j4-j4);
        if (!found || cost < best_cost) {
            found = 1;
            best_cost = cost;
            best_j2 = candidate_j2;
            best_j3 = candidate_j3;
            best_j4 = candidate_j4;
        }
    }
    if (!found) return 0;

    move_angle[2] = best_j2;
    move_angle[3] = best_j3;
    move_angle[4] = best_j4;
    fk_kinematics(j1, best_j2, best_j3, best_j4, j5);
    if (sqrtf((px-target_x)*(px-target_x) +
              (py-target_y)*(py-target_y) +
              (pz-target_z)*(pz-target_z)) > 0.002f) {
        move_angle[2] = old_j2;
        move_angle[3] = old_j3;
        move_angle[4] = old_j4;
        return 0;
    }
    return 1;
}

u8 Vision_TakeDoneFlag(void)
{
    u8 flag;
    unsigned int irq_mask = __get_PRIMASK();
    __disable_irq();
    flag = vision_done_flag;
    vision_done_flag = 0;
    __set_PRIMASK(irq_mask);
    return flag;
}

float angle_range[5][2]={{-2.35,2.35},//{min,max} 方向请看DH坐标示意�?
												 {-1.57,1.57},
												 {-2.3,1.57},
												 {-1.57,1},
												 {-1.57,1.57}};

#define pi 3.14159
#define t1 theta_i[0]
#define t2 theta_i[1]
#define t3 theta_i[2]
#define t4 theta_i[3]
#define t5 theta_i[4]

#define a2 0.105f
#define a3 0.0975f
#define a4 0.02822f
#define d1 0.1085f
#define d5 0.17283f

float T0t[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 1}};
float T05[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 1}};
u8 i = 0; // 用以循环的变�?
u8 j = 0; // 用以循环的变�?
u8 k = 0; // 用以循环的变�?
	
static u8 angle_range_check(float j1, float j2, float j3, float j4, float j5)
{
    float angles[] = {j1, j2, j3, j4, j5};
    for (int i = 0; i < 5; ++i) {
        if (angles[i] < angle_range[i][0] || angles[i] > angle_range[i][1]) {
            return 1;
        }
    }
    return 0;
}
	
void fk_kinematics(float angle1,float angle2,float angle3,float angle4,float angle5)
{
		if (angle_range_check(angle1, angle2, angle3, angle4, angle5)) {
				return;
		}	  
		T05[0][0] = sin(angle1)*sin(angle5) - cos(angle2 + angle3 + angle4)*cos(angle1)*cos(angle5);
    T05[0][1] = cos(angle5)*sin(angle1) + cos(angle2 + angle3 + angle4)*cos(angle1)*sin(angle5);
    T05[0][2] = -sin(angle2 + angle3 + angle4)*cos(angle1);
    T05[0][3] = -cos(angle1)*(a3*sin(angle2 + angle3) + a2*sin(angle2) + a4*cos(angle2 + angle3 + angle4) + d5*sin(angle2 + angle3 + angle4));
    T05[1][0] = -cos(angle1)*sin(angle5) - cos(angle2 + angle3 + angle4)*cos(angle5)*sin(angle1);
    T05[1][1] = cos(angle2 + angle3 + angle4)*sin(angle1)*sin(angle5) - cos(angle1)*cos(angle5);
    T05[1][2] = -sin(angle2 + angle3 + angle4)*sin(angle1);
    T05[1][3] = -sin(angle1)*(a3*sin(angle2 + angle3) + a2*sin(angle2) + a4*cos(angle2 + angle3 + angle4) + d5*sin(angle2 + angle3 + angle4));
    T05[2][0] = -sin(angle2 + angle3 + angle4)*cos(angle5);
    T05[2][1] = sin(angle2 + angle3 + angle4)*sin(angle5);
    T05[2][2] = cos(angle2 + angle3 + angle4);
    T05[2][3] = d1 + a3*cos(angle2 + angle3) + a2*cos(angle2) + d5*cos(angle2 + angle3 + angle4) - a4*sin(angle2 + angle3 + angle4);
		px=T05[0][3];
		py=T05[1][3];
    pz=T05[2][3];
		rx=atan2(T05[2][1],T05[1][1])+3.14;
		ry=atan2(T05[0][0],T05[0][2]);
}

void ik_kinematics(float x,float y,float z,float rx,float ry)
{
    // 初始�?
//	  float T5t_inv[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -l}, {0, 0, 0, 1}};
		
    // 初始�?
    static float theta[8][5] = {0};
    float temp[8][5] = {0};
    float t234[8] = {0};
    float M[8] = {0};
    float N[8] = {0};
    float K[4] = {0};
    float P = 0;
    float Q = 0;
    float s2 = 0;
    float c2 = 0;
    float distance = 0;
    float distance_min = 1000;
    int label = 0;
    int num = 0;
    int count[8] = {0};

    T05[0][0]=sin(ry);
		T05[0][1]=0;
		T05[0][2]=cos(ry);
		T05[0][3]=x;
		
    T05[1][0]=-cos(ry)*sin(rx);
    T05[1][1]=-cos(rx);
    T05[1][2]=sin(rx)*sin(ry);
    T05[1][3]=y;

    T05[2][0]=cos(rx)*cos(ry);
    T05[2][1]=-sin(rx);
    T05[2][2]=-cos(rx)*sin(ry);
    T05[2][3]=z;

    // 求解关节�?
    temp[0][0] = atan2(T05[1][3],T05[0][3]);
    temp[1][0] = temp[0][0];
    temp[2][0] = atan2(-T05[1][3],-T05[0][3]);
    temp[3][0] = temp[2][0];

    // 求解关节�?
    if (fabs(T05[2][2]-1) < 1e-3)
    {
        for (i=0; i<4; i++)
        {
            temp[i][4] = atan2(-T05[1][0],-T05[0][0]) - temp[i][0];
        }
    }
    else if (fabs(T05[2][2]+1) < 1e-3)
    {
        for (i=0; i<4; i++)
        {
            temp[i][4] = temp[i][0] - atan2(T05[1][0],T05[0][0]);
        }
    }
    else
    {
        temp[0][4] = atan2(T05[2][1],-T05[2][0]);
        temp[2][4] = temp[0][4];
        temp[1][4] = atan2(-T05[2][1],T05[2][0]);
        temp[3][4] = temp[1][4];
    }

    // 求解关节�?+关节�?+关节�?
    for (i=0; i<4; i++)
    {
        if (fabs(sin(temp[i][4])) < 1e-3)
        {
            t234[i] = atan2(-T05[2][0]/cos(temp[i][4]),T05[2][2]);
        }
        else
        {
            t234[i] = atan2(T05[2][1]/sin(temp[i][4]),T05[2][2]);
        }
    }

    // 求解关节�?
    for (i=0; i<4; i++)
    {
        if (fabs(sin(temp[i][0])) < 1e-3)
        {
            M[i] = (-T05[0][3]/cos(temp[i][0])) - a4*cos(t234[i]) - d5*sin(t234[i]);
        }
        else
        {
            M[i] = (-T05[1][3]/sin(temp[i][0])) - a4*cos(t234[i]) - d5*sin(t234[i]);
        }
        N[i] = T05[2][3] - d1 + a4*sin(t234[i]) - d5*cos(t234[i]);
        K[i] = (M[i]*M[i] + N[i]*N[i] - a2*a2 - a3*a3) / (2*a2*a3);
        if (fabs(K[i]-1) < 1e-3) K[i] = 1;
        else if (fabs(K[i]+1) < 1e-3) K[i] = -1;
        if (fabs(K[i]) > 1) temp[i][2] = 100, num++;
        else temp[i][2] = acos(K[i]);
    }
    for (i=4; i<8; i++)
    {
        M[i] = M[i-4];
        N[i] = N[i-4];
        t234[i] = t234[i-4];
        for (j=0; j<5; j++)
        {
            if (j == 2) temp[i][j] = -temp[i-4][j];
            else temp[i][j] = temp[i-4][j];
        }
    }

    // 求解关节�?
    for (i=0; i<8; i++)
    {
        P = a2 + a3*cos(temp[i][2]);
        Q = a3*sin(temp[i][2]);
        s2 = (P*M[i] - Q*N[i])/(P*P + Q*Q);
        c2 = (Q*M[i] + P*N[i])/(P*P + Q*Q);
        temp[i][1] = atan2(s2,c2);
    }

    // 求解关节�?
    for (i=0; i<8; i++) temp[i][3] = t234[i] - temp[i][1] - temp[i][2];

    // 筛选合适的解，换算至合理的区间范围以及选取最优解.
    j = 0;
    for (i=0; i<8; i++)
    {
        if (fabs(temp[i][2]) != 100)
        {
            count[j] = i;
            j += 1;
        }
				
    }
		ik_flag=0;
    for (i=0; i<(8-2*num); i++)
    {
				ik_flag=1;
        distance = 0;
        for (j=0; j<5; j++)
        {
            k = count[i];
            theta[i][j] = temp[k][j];
            if (theta[i][j] > pi) theta[i][j] = theta[i][j] - 2*pi;
            else if (theta[i][j] < -pi) theta[i][j] = theta[i][j] + 2*pi;
            theta[i][j] = (int)(theta[i][j]*10000.0f)/10000.0f; // 结果保留四位小数
            distance += theta[i][j]*theta[i][j]; // 计算点到原点的距离（把一组解看成五维空间中的一个点，其坐标为五个关节角的值）
        }
        if (distance < distance_min) distance_min = distance, label = i; // 选取最优解
    }
//调试信息，默认不打印
// 打印所有可能的逆解
//    printf("\n");
//    printf("可能的关节角设置�?\t");
//    printf("\n");
//    for (i=0; i<(8-2*num); i++)
//    {
//        for (j=0; j<5; j++)
//        {
//            if (theta[i][j]>=0) printf(" %.4f\t\t",fabs(theta[i][j]));
//            else printf("-%.4f\t\t",fabs(theta[i][j]));
//        }
//        printf("\n");
//    }

//    // 打印最优的逆解
//    printf("\n");
//    printf("最优的关节角设置为:\t");
//    printf("\n");
//    for (j=0; j<5; j++)
//    {
//        if (theta[label][j]>=0) printf(" %.4f\t\t",fabs(theta[label][j]));
//        else printf("-%.4f\t\t",fabs(theta[label][j]));
//    }
//    printf("\n");
//调试信息，默认不打印
			for(u8 i=0;i<5;i++) 
		{
			if(theta[label][i]<angle_range[i][0] || theta[label][i]>angle_range[i][1])
			{
						ik_flag=0;
					  return;
			}
		}
		if(ik_flag)
		{
				for(i=0;i<5;i++)
			{
						move_angle[i+1]=theta[label][i];
			}
		}
		
}
	

/**************************************************************************
函数功能：定时中断函�?所有的控制代码都在这里�?	 
**************************************************************************/

static void VisionArm_Update_10ms(void)
{
    float phase, dt;
    u8 holding_mango, fresh, new_frame, centered, reset_pid;
    int target_x, target_y;
    unsigned int center_frames, irq_mask, dt_ticks;

    /* Snapshot coordinates and freshness atomically against USART3. */
    irq_mask = __get_PRIMASK();
    __disable_irq();
    if (vision_age_ticks < 1000U) vision_age_ticks++;
    fresh = vision_target_valid && vision_age_ticks <= VISION_TIMEOUT_TICKS;
    target_x = vision_target_x;
    target_y = vision_target_y;
    new_frame = vision_target_pending;
    vision_target_pending = 0;
    center_frames = vision_center_frames;
    reset_pid = vision_pid_reset_pending;
    vision_pid_reset_pending = 0;
    __set_PRIMASK(irq_mask);
    if (reset_pid) {
        Vision_ResetPID();
        vision_track_elapsed_ticks = 0;
    }
    if (vision_arm_state == ARM_TRACK_TARGET &&
        vision_track_elapsed_ticks < VISION_PID_MAX_DT_TICKS)
        vision_track_elapsed_ticks++;
    centered = fresh &&
        abs(target_x - VISION_CENTER_X) <= VISION_CENTER_TOLERANCE_PX &&
        abs(target_y - VISION_CENTER_Y) <= VISION_CENTER_TOLERANCE_PX &&
        center_frames >= VISION_CENTER_REQUIRED_FRAMES;

    switch (vision_arm_state) {
    case ARM_STARTUP_HOME:
        move_angle[1] = ARM_HOME_J1;
        move_angle[2] = ARM_HOME_J2;
        move_angle[3] = ARM_HOME_J3;
        move_angle[4] = ARM_HOME_J4;
        move_angle[5] = ARM_HOME_J5;
        move_angle[6] = ARM_GRIPPER_CLOSE;
        Speed = VISION_TRACK_SPEED;
        Vision_SetNoTarget();
        if (++vision_state_ticks >= ARM_STARTUP_TICKS) {
            vision_state_ticks = 0;
            vision_arm_state = ARM_TRACK_TARGET;
        }
        break;

    case ARM_TRACK_TARGET:
        Speed = VISION_TRACK_SPEED;
        move_angle[6] = ARM_GRIPPER_CLOSE;
        if (!fresh) {
            Vision_ResetPID();
            vision_track_elapsed_ticks = 0;
        }
        if (fresh && new_frame) {
            if (centered) {
                /* Hold the aligned pose; do not let the derivative kick it away. */
                Vision_ResetPID();
                vision_track_elapsed_ticks = 0;
                Speed = VISION_GRASP_SPEED;
                vision_done_sent = 0;
                vision_state_ticks = 0;
                vision_arm_state = ARM_GRASP_OPEN;
            } else {
                dt_ticks = vision_track_elapsed_ticks;
                if (dt_ticks == 0) dt_ticks = 1;
                dt = 0.01f * (float)dt_ticks;
                vision_track_elapsed_ticks = 0;
                move_angle[1] = Vision_PIDUpdate(
                    target_x - VISION_CENTER_X, dt, move_angle[1],
                    VISION_PID_KP_J1, VISION_PID_KI_J1, VISION_PID_KD_J1,
                    &vision_pid_j1, angle_range[0][0], angle_range[0][1]);
                move_angle[4] = Vision_PIDUpdate(
                    target_y - VISION_CENTER_Y, dt, move_angle[4],
                    VISION_PID_KP_J4, VISION_PID_KI_J4, VISION_PID_KD_J4,
                    &vision_pid_j4, angle_range[3][0], angle_range[3][1]);
            }
        }
        break;

    case ARM_GRASP_OPEN:
        if (!centered) {
            move_angle[6] = ARM_GRIPPER_CLOSE;
            Speed = VISION_TRACK_SPEED;
            vision_state_ticks = 0;
            vision_arm_state = ARM_TRACK_TARGET;
            break;
        }
        /* Request J6 open and hold every other joint still. */
        move_angle[6] = ARM_GRIPPER_OPEN;
        vision_state_ticks = 0;
        vision_settle_ticks = 0;
        vision_arm_state = ARM_WAIT_GRIPPER_OPEN;
        break;

    case ARM_WAIT_GRIPPER_OPEN:
        if (!centered) {
            move_angle[6] = ARM_GRIPPER_CLOSE;
            Speed = VISION_TRACK_SPEED;
            vision_state_ticks = 0;
            vision_settle_ticks = 0;
            vision_arm_state = ARM_TRACK_TARGET;
            break;
        }
        move_angle[6] = ARM_GRIPPER_OPEN;
        if (Vision_GripperOutputSettled()) {
            if (++vision_settle_ticks >= VISION_SERVO_SETTLE_TICKS) {
                vision_settle_ticks = 0;
                vision_state_ticks = 0;
                vision_arm_state = ARM_APPROACH_TARGET;
                break;
            }
        } else {
            vision_settle_ticks = 0;
        }
        if (++vision_state_ticks >= VISION_SERVO_SETTLE_TIMEOUT_TICKS) {
            Vision_BeginReturn(0);
        }
        break;

    case ARM_APPROACH_TARGET:
        if (vision_state_ticks == 0) {
            /* Recheck centering before moving the arm toward the target. */
            if (!centered) {
                move_angle[6] = ARM_GRIPPER_CLOSE;
                Speed = VISION_TRACK_SPEED;
                vision_arm_state = ARM_TRACK_TARGET;
                break;
            }
            aim_j1 = move_angle[1];
            aim_j2 = move_angle[2];
            aim_j3 = move_angle[3];
            aim_j4 = move_angle[4];
            if (!Vision_ComputeApproach()) {
                Vision_BeginReturn(0);
                break;
            }
            end_j2 = move_angle[2];
            end_j3 = move_angle[3];
            end_j4 = move_angle[4];
        }
        phase = (float)(vision_state_ticks + 1U) / (float)ARM_APPROACH_TICKS;
        move_angle[1] = aim_j1;
        move_angle[2] = aim_j2 + (end_j2 - aim_j2) * phase;
        move_angle[3] = aim_j3 + (end_j3 - aim_j3) * phase;
        move_angle[4] = aim_j4 + (end_j4 - aim_j4) * phase;
        vision_state_ticks++;
        if (vision_state_ticks >= ARM_APPROACH_TICKS) {
            move_angle[1] = aim_j1;
            move_angle[2] = end_j2;
            move_angle[3] = end_j3;
            move_angle[4] = end_j4;
            vision_state_ticks = 0;
            vision_settle_ticks = 0;
            vision_arm_state = ARM_WAIT_APPROACH_SETTLE;
        }
        break;

    case ARM_WAIT_APPROACH_SETTLE:
        /* Keep the IK endpoint commanded until the PWM outputs settle. */
        move_angle[1] = aim_j1;
        move_angle[2] = end_j2;
        move_angle[3] = end_j3;
        move_angle[4] = end_j4;
        if (Vision_ApproachOutputsSettled()) {
            if (++vision_settle_ticks >= VISION_SERVO_SETTLE_TICKS) {
                move_angle[6] = ARM_GRIPPER_CLOSE;
                vision_state_ticks = 0;
                vision_settle_ticks = 0;
                vision_arm_state = ARM_CLOSE_GRIPPER;
                break;
            }
        } else {
            vision_settle_ticks = 0;
        }
        if (++vision_state_ticks >= VISION_SERVO_SETTLE_TIMEOUT_TICKS) {
            /* Do not claim a grasp if the approach command cannot settle. */
            Vision_BeginReturn(0);
        }
        break;

    case ARM_CLOSE_GRIPPER:
        move_angle[6] = ARM_GRIPPER_CLOSE;
        /* Wait for J6 close output to settle before starting the grip hold. */
        if (Vision_GripperOutputSettled()) {
            if (++vision_settle_ticks >= VISION_SERVO_SETTLE_TICKS) {
                vision_state_ticks = 0;
                vision_settle_ticks = 0;
                vision_arm_state = ARM_HOLD_CLOSED_GRIPPER;
                break;
            }
        } else {
            vision_settle_ticks = 0;
        }
        if (++vision_state_ticks >= VISION_SERVO_SETTLE_TIMEOUT_TICKS)
            Vision_BeginReturn(0);
        break;

    case ARM_HOLD_CLOSED_GRIPPER:
        move_angle[6] = ARM_GRIPPER_CLOSE;
        /* Restart the close-settle phase if the commanded output drifts. */
        if (!Vision_GripperOutputSettled()) {
            vision_state_ticks = 0;
            vision_settle_ticks = 0;
            vision_arm_state = ARM_CLOSE_GRIPPER;
            break;
        }
        if (++vision_state_ticks >= ARM_GRIPPER_TICKS)
            Vision_BeginReturn(1);
        break;

    case ARM_RETURN_WITH_MANGO:
    case ARM_RETURN_EMPTY:
        holding_mango = (vision_arm_state == ARM_RETURN_WITH_MANGO);
        if (vision_state_ticks < ARM_RETURN_TICKS) {
            phase = (float)(vision_state_ticks + 1U) / (float)ARM_RETURN_TICKS;
            move_angle[1] = return_j1 + (ARM_HOME_J1 - return_j1) * phase;
            move_angle[2] = return_j2 + (ARM_HOME_J2 - return_j2) * phase;
            move_angle[3] = return_j3 + (ARM_HOME_J3 - return_j3) * phase;
            move_angle[4] = return_j4 + (ARM_HOME_J4 - return_j4) * phase;
            move_angle[5] = ARM_HOME_J5;
            /* Failed approach returns with J6 open; close only at home. */
            move_angle[6] = holding_mango ? ARM_GRIPPER_CLOSE : ARM_GRIPPER_OPEN;
            vision_state_ticks++;
        } else {
            move_angle[1] = ARM_HOME_J1;
            move_angle[2] = ARM_HOME_J2;
            move_angle[3] = ARM_HOME_J3;
            move_angle[4] = ARM_HOME_J4;
            move_angle[5] = ARM_HOME_J5;
            move_angle[6] = ARM_GRIPPER_CLOSE;
            vision_state_ticks = 0;
            vision_settle_ticks = 0;
            Speed = VISION_TRACK_SPEED;
            Vision_SetNoTarget();
            vision_arm_state = ARM_WAIT_RETURN_SETTLE;
        }
        break;

    case ARM_WAIT_RETURN_SETTLE:
        move_angle[1] = ARM_HOME_J1;
        move_angle[2] = ARM_HOME_J2;
        move_angle[3] = ARM_HOME_J3;
        move_angle[4] = ARM_HOME_J4;
        move_angle[5] = ARM_HOME_J5;
        move_angle[6] = ARM_GRIPPER_CLOSE;
        if (Vision_ApproachOutputsSettled() && Vision_GripperOutputSettled()) {
            if (++vision_settle_ticks >= VISION_SERVO_SETTLE_TICKS) {
                vision_settle_ticks = 0;
                vision_rearm_silence_ticks = 0;
                vision_rearm_ready = 0;
                Vision_SetNoTarget();
                vision_arm_state = ARM_HOLD_AT_HOME;
            }
        } else {
            vision_settle_ticks = 0;
        }
        break;

    case ARM_HOLD_AT_HOME:
        move_angle[1] = ARM_HOME_J1;
        move_angle[2] = ARM_HOME_J2;
        move_angle[3] = ARM_HOME_J3;
        move_angle[4] = ARM_HOME_J4;
        move_angle[5] = ARM_HOME_J5;
        move_angle[6] = ARM_GRIPPER_CLOSE;
        if (!vision_done_sent) {
            vision_done_flag = 1;
            vision_done_sent = 1;
            Speed = VISION_TRACK_SPEED;
        }
        if (!vision_rearm_ready) {
            if (new_frame) {
                vision_rearm_silence_ticks = 0;
            } else if (++vision_rearm_silence_ticks >= VISION_REARM_SILENCE_TICKS) {
                vision_rearm_ready = 1;
                Vision_SetNoTarget();
            }
        } else if (fresh && new_frame) {
            /* Preserve the first post-return observation for tracking. */
            irq_mask = __get_PRIMASK();
            __disable_irq();
            vision_target_pending = 1;
            __set_PRIMASK(irq_mask);
            vision_state_ticks = 0;
            vision_arm_state = ARM_TRACK_TARGET;
        }
        break;

    default:
        vision_arm_state = ARM_STARTUP_HOME;
        vision_state_ticks = 0;
        break;
    }
}
int TIM2_IRQHandler(void)
{ 
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断�?
	{   
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理�?TIM 中断�?
			//扫描按键变化	
			Key();
			mode=0;
			VisionArm_Update_10ms();
			fk_kinematics(move_angle[1],move_angle[2],move_angle[3],move_angle[4],move_angle[5]);

			Servo_angle_check();     //舵机旋转角度范围限制，单位弧�?
			
			//通过目标角度求解舵机对应的PWM�?
			Servo1=SERVO_PWM_VALUE_270(-move_angle[1])+Angle1_init;
			Servo2=SERVO_PWM_VALUE_180(move_angle[2])+Angle2_init;
			
			Servo3=SERVO_PWM_VALUE_180(-move_angle[3])+Angle3_init;
			
			Servo4=SERVO_PWM_VALUE_180(-move_angle[4])+Angle4_init;
			Servo5=SERVO_PWM_VALUE_180(-move_angle[5])+Angle5_init;
			Servo6=SERVO_PWM_VALUE_180(move_angle[6])+Angle6_init;
			Servo_Pwm_check();   //舵机旋转PWM范围限制
			//通过目标PWM值做位置PID控制
			Velocity1=Position_PID1(Position1,Servo1);//舵机PID控制
			Velocity2=Position_PID2(Position2,Servo2);//舵机PID控制
			Velocity3=Position_PID3(Position3,Servo3);//舵机PID控制
			Velocity4=Position_PID4(Position4,Servo4);//舵机PID控制
			Velocity5=Position_PID5(Position5,Servo5);//舵机PID控制
			Velocity6=Position_PID6(Position6,Servo6);//舵机PID控制
	    Servo_Speed_Check();
			Set_Pwm(Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6);    //赋值给PWM寄存�?
     		
   	}       	
	 return 0;	  
} 
/**************************************************************************
函数功能：将PWM值输出给舵机，实现舵机的控制	 
**************************************************************************/
void Set_Pwm(float velocity1,float velocity2,float velocity3,float velocity4,float velocity5,float velocity6)
{	
		Position1+=velocity1;		   //速度的积分，得到舵机的位�?
		Position2+=velocity2;
		Position3+=velocity3;
		Position4+=velocity4;
		Position5+=velocity5;	
		Position6+=velocity6;		
	  //赋值给STM32的寄存器
	  TIM4->CCR4=(int)Position1;   //云台
	  TIM4->CCR3=(int)Position3;   //云台上的第二个关�?
	  TIM4->CCR2=(int)Position4;   //云台上的第三个关�?
	  TIM4->CCR1=(int)Position5;   //控制夹爪旋转关节
	  TIM3->CCR1=(int)Position2;   //云台上的第一个关�?
	  TIM3->CCR2=(int)Position6;   //夹爪
}

/**************************************************************************
函数功能：异常关闭电�?
入口参数：电�?
返回  值：1：异�? 0：正�?
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1150)//电池电压过低  舵机将不能控�?
			{	                                                
      temp=1;      
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
函数功能：限制PWM赋�?
入口参数：无
返回  值：�?
**************************************************************************/
//舵机目标弧度极限�?
void Servo_angle_check(void)
{	
 
    if(move_angle[1]<angle_range[0][0])  move_angle[1]=angle_range[0][0];	
		if(move_angle[1]>angle_range[0][1])  move_angle[1]=angle_range[0][1];	
	
	  if(move_angle[2]<angle_range[1][0])  move_angle[2]=angle_range[1][0];	
		if(move_angle[2]>angle_range[1][1])  move_angle[2]=angle_range[1][1];	
	  
		if(move_angle[3]<angle_range[2][0])  move_angle[3]=angle_range[2][0];	
		if(move_angle[3]>angle_range[2][1])  move_angle[3]=angle_range[2][1];	
	
		if(move_angle[4]<angle_range[3][0])  move_angle[4]=angle_range[3][0];	
		if(move_angle[4]>angle_range[3][1])  move_angle[4]=angle_range[3][1];	//安装了摄像头的关�?
	
	  if(move_angle[5]<angle_range[4][0])  move_angle[5]=angle_range[4][0];	
		if(move_angle[5]>angle_range[4][1])  move_angle[5]=angle_range[4][1];	
	
		if(move_angle[6]<-0.9)         move_angle[6]=-0.9;	//夹爪
		if(move_angle[6]>0.9)          move_angle[6]=0.9;	  
}

//舵机舵机PWM极限�?
void Servo_Pwm_check(void)
{	
	  int Amplitude_H=2500, Amplitude_L=500;       
    if(Servo1<Amplitude_L)  Servo1=Amplitude_L;	
		if(Servo1>Amplitude_H)  Servo1=Amplitude_H;	
	  if(Servo2<Amplitude_L)  Servo2=Amplitude_L;	
		if(Servo2>Amplitude_H)  Servo2=Amplitude_H;	
		if(Servo3<Amplitude_L)  Servo3=Amplitude_L;	
		if(Servo3>Amplitude_H)  Servo3=Amplitude_H;	
		if(Servo4<Amplitude_L)  Servo4=Amplitude_L;	
		if(Servo4>Amplitude_H)  Servo4=Amplitude_H;	
	  if(Servo5<Amplitude_L)  Servo5=Amplitude_L;	
		if(Servo5>Amplitude_H)  Servo5=Amplitude_H;	
		if(Servo6<Amplitude_L)  Servo6=Amplitude_L;	
		if(Servo6>Amplitude_H)  Servo6=Amplitude_H;		
}
//舵机运动速度控制�?
void Servo_Speed_Check(void)
{	
	  int Amplitude_H=Speed, Amplitude_L=-Speed;   
    if(Velocity1<Amplitude_L)  Velocity1=Amplitude_L;	
		if(Velocity1>Amplitude_H)  Velocity1=Amplitude_H;	
	  if(Velocity2<Amplitude_L)  Velocity2=Amplitude_L;	
		if(Velocity2>Amplitude_H)  Velocity2=Amplitude_H;	
		if(Velocity3<Amplitude_L)  Velocity3=Amplitude_L;	
		if(Velocity3>Amplitude_H)  Velocity3=Amplitude_H;	
		if(Velocity4<Amplitude_L)  Velocity4=Amplitude_L;	
		if(Velocity4>Amplitude_H)  Velocity4=Amplitude_H;	
	  if(Velocity5<Amplitude_L)  Velocity5=Amplitude_L;	
		if(Velocity5>Amplitude_H)  Velocity5=Amplitude_H;	
		if(Velocity6<Amplitude_L)  Velocity6=Amplitude_L;	
		if(Velocity6>Amplitude_H)  Velocity6=Amplitude_H;	
}

/**************************************************************************
函数功能：按键修改机械臂使能状�?
入口参数：无
返回  值：�?
**************************************************************************/
void Key(void)
{	
	mode=0;
}

/**************************************************************************
函数功能：手柄控制机械臂
入口参数：控制速度
返回  值：�?
**************************************************************************/
void PS2_Control(float Step)
{ 	 
	 float positon_step=0.001;//单位m
	 float rotation_step=0.01;//单位rad
				if(PS2_KEY==6)    if(mode)py-=positon_step;else move_angle[3]-=Step;    //云台
	 else if(PS2_KEY==8)		if(mode)py+=positon_step;else move_angle[3]+=Step;
	
	 else if(PS2_KEY==11)		if(mode)pz+=positon_step;else move_angle[1]-=Step;    //机械手的张合
	 else if(PS2_KEY==9)		if(mode)pz-=positon_step;else move_angle[1]+=Step;
	
	 else if(PS2_KEY==5)		if(mode)px+=positon_step;else move_angle[2]+=Step;
	 else if(PS2_KEY==7)		if(mode)px-=positon_step;else move_angle[2]-=Step;
	 
	 else if(PS2_KEY==13)		 if(mode)ry+=rotation_step;else move_angle[5]+=Step;   //舵机3
	 else if(PS2_KEY==15)		 if(mode)ry-=rotation_step;else move_angle[5]-=Step;   
		
	 else if(PS2_KEY==14)		if(mode)rx+=rotation_step;else move_angle[4]+=Step;  //舵机4
	 else if(PS2_KEY==16)		if(mode)rx-=rotation_step;else move_angle[4]-=Step;
	
	 else if(PS2_KEY==12)		 move_angle[6]+=Step;
	 else if(PS2_KEY==10)		move_angle[6]-=Step;
}


/**************************************************************************
函数功能：绝对值函�?
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：输入舵机的目标角度输出对应的pwm�?
入口参数：舵机转向角度（单位：弧度）
返回  值：对应输出的pwm值（范围�?00—�?500�?
**************************************************************************/
int SERVO_PWM_VALUE_180(float angle)
{
			int K=1000;
			float Ratio=0.6369; //180°舵机
	    //float Ratio=0.4246; //270°舵机
			int pwm_value;
			
			pwm_value=(SERVO_INIT-angle*K*Ratio); //舵机目标PWM数�?
	    return pwm_value;
}

int SERVO_PWM_VALUE_270(float angle)
{
			int K=1000;
//			float Ratio=0.6369; //180°舵机
	    float Ratio=0.4246; //270°舵机
			int pwm_value;
			
			pwm_value=(SERVO_INIT-angle*K*Ratio); //舵机目标PWM数�?
	    return pwm_value;
}

/**************************************************************************
函数功能：输入舵机的pwm值输出对应的关节角度
入口参数：舵机的pwm值（范围�?00—�?500�?
返回  值：输出对应的关节角度（范围�?1.57 —�?1.57�?
**************************************************************************/
float PWM_Angle_VALUE(int pwm)
{
			int K=1000;
			float Ratio=0.6369; //180°舵机
	    float angle=0;
	      //float Ratio=0.4246; //270°舵机
			
			angle=(SERVO_INIT-pwm)/(K*Ratio); //舵机目标PWM数�?
	    return angle;
}
/**************************************************************************
函数功能：位置式PID控制�?
入口参数：编码器测量位置信息，目标位�?
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k�?e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积�?其中k�?,2,,k;
pwm代表输出
**************************************************************************/
float Position_PID1 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积�?
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制�?
	 Last_Bias=Bias;                                       //保存上一次偏�?
	 return Pwm;                                           //增量输出
}
float Position_PID2 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积�?
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制�?
	 Last_Bias=Bias;                                       //保存上一次偏�?
	 return Pwm;                                           //增量输出
}
float Position_PID3 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积�?
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制�?
	 Last_Bias=Bias;                                       //保存上一次偏�?
	 return Pwm;                                           //增量输出
}
float Position_PID4 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积�?
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制�?
	 Last_Bias=Bias;                                       //保存上一次偏�?
	 return Pwm;                                           //增量输出
}
float Position_PID5 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积�?
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制�?
	 Last_Bias=Bias;                                       //保存上一次偏�?
	 return Pwm;                                           //增量输出
}
float Position_PID6 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积�?
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制�?
	 Last_Bias=Bias;                                       //保存上一次偏�?
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：舵机初始位置调整函�?
入口参数：无
返回  值：�?
**************************************************************************/
void Servo_init_angle_adjust_function(void)
{
	int step=1; 
				if(PS2_KEY==9)     Angle1_init+=step;     //云台
	 else if(PS2_KEY==11)		 Angle1_init-=step;
	
	 else if(PS2_KEY==12)		 Angle6_init-=step;    //机械手的张合
	 else if(PS2_KEY==10)		 Angle6_init+=step;
	
	 else if(PS2_KEY==5)		Angle2_init-=step;     //云台上的�?个自由度
	 else if(PS2_KEY==7)		Angle2_init+=step;   
		
	 else if(PS2_KEY==6)		Angle3_init-=step;   //云台上的�?个自由度
	 else if(PS2_KEY==8)		Angle3_init+=step;   
		
	 else if(PS2_KEY==13)		Angle4_init-=step;   //云台上的�?个自由度
	 else if(PS2_KEY==15)		Angle4_init+=step;
	
	 else if(PS2_KEY==14)		Angle5_init+=step;   //控制机械手旋转的舵机
	 else if(PS2_KEY==16)		Angle5_init-=step;	
}
/**************************************************************************
函数功能：舵机初始位置调整模式检测函�?
入口参数：无
返回  值：�?
**************************************************************************/
void Servo_init_angle_adjust_mode_check(void)
{
	static int check_count1=0,check_count2=0;
	
		if(PS2_LX<100 && PS2_RX>200)   check_count1++;  //手柄左右两边摇杆往左右两边�?
	  else check_count1=0;
	
		if(PS2_LX>200 && PS2_RX<100)   check_count2++;  //手柄左右两边摇杆往中间�?
	  else check_count2=0;
	//进入模式检�?
	if(check_count1>500)//连续推住5�?
	{
		Beep_Control_Flag=1;  //蜂鸣器标志位�?，蜂鸣器�?�?
		Servo_init_angle_adjust=1; //进入舵机初始位置调整模式
		OLED_Refresh_Gram(); //刷新屏幕
    OLED_Clear();  //清除屏幕
		OLED_Refresh_Gram();
		check_count1=0;
		
	}
		//退出模式检�?
	if(check_count2>500 && Servo_init_angle_adjust==1)//连续推住5�?
	{
	  Flash_Write();     //将调整好的值写入flash
		OLED_Clear();  //清除屏幕
		OLED_Refresh_Gram();//刷新屏幕
		Servo_init_angle_adjust=0;
		check_count2=0;
	}

}


void Servo_init_pose(u8 j1,u8 j2,u8 j3,u8 j4,u8 j5,u8 j6)
{
	int step=5;
	if(j1==1)   		Angle1_init+=step;
	else if(j1==2)  Angle1_init-=step;
	
	if(j2==1)   		Angle2_init+=step;
	else if(j2==2)  Angle2_init-=step;
	
	if(j3==1)   		Angle3_init+=step;
	else if(j3==2)  Angle3_init-=step;
	
	if(j4==1)   		Angle4_init+=step;
	else if(j4==2)  Angle4_init-=step;
	
	if(j5==1)   		Angle5_init+=step;
	else if(j5==2)  Angle5_init-=step;
	
	if(j6==1)   		Angle6_init+=step;
	else if(j6==2)  Angle6_init-=step;
	
}



