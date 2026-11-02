#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
typedef unsigned char u8;
float Speed=15;
float angle_range[5][2]={{-2.35f,2.35f},{-1.57f,1.57f},{-2.3f,1.57f},{-1.57f,1.0f},{-1.57f,1.57f}};
unsigned int __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(unsigned int x) { (void)x; }
void Vision_SetNoTarget(void);
struct Port { unsigned int DR; } port;
#define USART3 (&port)
#define USART_IT_RXNE 1
#define RESET 0
int USART_GetITStatus(struct Port *p, int f) { (void)p; (void)f; return 1; }
static unsigned char sent_bytes[4];
static unsigned int sent_length=0;
void usart3_send(u8 value) { if (sent_length < sizeof(sent_bytes)) sent_bytes[sent_length]=value; sent_length++; }
#ifndef __VISION_CONFIG_H
#define __VISION_CONFIG_H

/* Pi continuous v002 uses 640 x 480 image-edge coordinates, origin bottom-left. */
#define VISION_FRAME_WIDTH 640
#define VISION_FRAME_HEIGHT 480
#define VISION_CENTER_X (VISION_FRAME_WIDTH / 2)
#define VISION_CENTER_Y (VISION_FRAME_HEIGHT / 2)
/* Allow +/-20 px on each axis; count received frames, not timer ticks.
   The same window must remain valid while waiting for the gripper to open. */
#define VISION_CENTER_TOLERANCE_PX 20
#define VISION_CENTER_REQUIRED_FRAMES 3U
#define VISION_TIMEOUT_TICKS 75U /* 750 ms */
#define VISION_REARM_SILENCE_TICKS 75U /* wait for the Pi stream to stop */
#define VISION_SERVO_SETTLE_PWM 12.0f
#define VISION_SERVO_SETTLE_TICKS 10U /* 100 ms stable output */
#define VISION_SERVO_SETTLE_TIMEOUT_TICKS 300U /* 3 s fail-safe */

/* Adjust here, rebuild and flash. Position packets never set speed. */
#define VISION_TRACK_SPEED 15.0f
#define VISION_GRASP_SPEED 22.0f
/* Outer-loop visual PID: output is joint command rate, applied per fresh frame.
   Signs retain the bench-verified direction: right error turns image right. */
#define VISION_PID_KP_J1 (-0.0012f) /* rad/s per pixel */
#define VISION_PID_KI_J1 (-0.00005f) /* rad/s per (pixel*s) */
#define VISION_PID_KD_J1 (-0.00015f) /* rad per pixel */
#define VISION_PID_KP_J4 ( 0.0012f)
#define VISION_PID_KI_J4 ( 0.00005f)
#define VISION_PID_KD_J4 ( 0.00015f)
#define VISION_PID_D_FILTER_ALPHA 0.35f
#define VISION_PID_INTEGRAL_LIMIT_PX_S 100.0f
#define VISION_PID_MAX_RATE_RAD_S 0.16f
#define VISION_PID_MAX_DT_TICKS 10U /* cap elapsed time at 100 ms */
#define ARM_APPROACH_DISTANCE_MM 50.0f

#endif
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
int Servo1=0,Servo2=0,Servo3=0,Servo4=0,Servo5=0,Servo6=0; //èµæºPWMåé
float Velocity1=0,Velocity2=0,Velocity3=0,Velocity4=0,Velocity5=0,Velocity6=0;     //çµæºPWMåé
float Position1=1500,Position2=1500,Position3=1500,Position4=1500,Position5=1500,Position6=1500;   
float move_angle[7]={0,ARM_HOME_J1,ARM_HOME_J2,ARM_HOME_J3,ARM_HOME_J4,ARM_HOME_J5,ARM_GRIPPER_OPEN};
short Angle1_init=0,Angle2_init=0,Angle3_init=0,Angle4_init=0,Angle5_init=0,Angle6_init=0; //èµæºåå§ä½ç½®ï¿½?
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
static int approach_ok = 1;
static u8 Vision_ComputeApproach(void)
{
    if (!approach_ok) return 0;
    move_angle[2] += 0.10f;
    move_angle[3] -= 0.05f;
    move_angle[4] += 0.03f;
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

void Vision_SendDone(void)
{
	usart3_send(0xEF);
	usart3_send(0xFE);
	usart3_send(0x7E);
	usart3_send(0x6B);
}static int emulate_settled_outputs = 1;
static void emulate_outputs(void)
{
    if (!emulate_settled_outputs) return;
    Servo1 = 1500 + (int)(move_angle[1] * 100.0f);
    Servo2 = 1500 + (int)(move_angle[2] * 100.0f);
    Servo3 = 1500 + (int)(move_angle[3] * 100.0f);
    Servo4 = 1500 + (int)(move_angle[4] * 100.0f);
    Servo5 = 1500 + (int)(move_angle[5] * 100.0f);
    Position1=(float)Servo1; Position2=(float)Servo2;
    Position3=(float)Servo3; Position4=(float)Servo4;
    Position5=(float)Servo5;
    Servo6=(move_angle[6] == ARM_GRIPPER_OPEN) ? 1000 : 2000;
    Position6=(float)Servo6;
}
static void tick(unsigned n)
{
    while(n--) {
        VisionArm_Update_10ms();
        emulate_outputs();
    }
}
static void byte(unsigned char b) { port.DR=b; USART3_IRQHandler(); }
static void frame(int x,int y)
{
    unsigned char f[7]={0xEF,0xFE,0,0,0,0,0}; int i;
    f[2]=x&255; f[3]=(x>>8)&255; f[4]=y&255; f[5]=(y>>8)&255;
    for(i=0;i<6;i++) f[6]+=f[i];
    for(i=0;i<7;i++) byte(f[i]);
}
static void center_frame(void) { frame(320,240); }
static void reset_controller(void)
{
    emulate_settled_outputs=1;
    approach_ok=1;
    vision_arm_state=ARM_STARTUP_HOME;
    vision_state_ticks=0;
    vision_settle_ticks=0;
    vision_done_flag=0;
    vision_done_sent=0;
    vision_rearm_silence_ticks=0;
    vision_rearm_ready=0;
    vision_frame_index=0;
    tick(ARM_STARTUP_TICKS);
    assert(vision_arm_state==ARM_TRACK_TARGET);
}
static void test_pid_controller(void)
{
    VisionPID damped={0.0f,0.0f,0,0}, proportional={0.0f,0.0f,0,0};
    VisionPID limited={0.0f,0.0f,0,0}, stopped={0.0f,0.0f,0,0};
    float a0=ARM_HOME_J1, a1, a2, p0=ARM_HOME_J1, p1, p2;
    float step;
    a1=Vision_PIDUpdate(100,0.05f,a0,VISION_PID_KP_J1,VISION_PID_KI_J1,
                        VISION_PID_KD_J1,&damped,angle_range[0][0],angle_range[0][1]);
    a2=Vision_PIDUpdate(50,0.05f,a1,VISION_PID_KP_J1,VISION_PID_KI_J1,
                        VISION_PID_KD_J1,&damped,angle_range[0][0],angle_range[0][1]);
    p1=Vision_PIDUpdate(100,0.05f,p0,VISION_PID_KP_J1,0.0f,0.0f,
                        &proportional,angle_range[0][0],angle_range[0][1]);
    p2=Vision_PIDUpdate(50,0.05f,p1,VISION_PID_KP_J1,0.0f,0.0f,
                        &proportional,angle_range[0][0],angle_range[0][1]);
    assert(fabsf(a2-a1)<fabsf(p2-p1)); /* D term brakes a closing error. */

    a1=Vision_PIDUpdate(32767,0.05f,a0,VISION_PID_KP_J1,VISION_PID_KI_J1,
                        VISION_PID_KD_J1,&limited,angle_range[0][0],angle_range[0][1]);
    assert(fabsf(a1-a0)<=VISION_PID_MAX_RATE_RAD_S*0.05f+0.000001f);
    step=Vision_PIDUpdate(-100,0.05f,angle_range[0][1]-0.001f,
                         VISION_PID_KP_J1,VISION_PID_KI_J1,VISION_PID_KD_J1,
                         &stopped,angle_range[0][0],angle_range[0][1]);
    assert(step<=angle_range[0][1] && step>=angle_range[0][0]);
    assert(stopped.integral==0.0f); /* no integral windup at the joint stop */
}
static void request_center(void)
{
    center_frame(); tick(1);
    center_frame(); tick(1);
    center_frame(); tick(1);
    assert(vision_arm_state==ARM_GRASP_OPEN);
}
static void begin_approach_and_check_open(void)
{
    request_center();
    tick(1);
    assert(vision_arm_state==ARM_WAIT_GRIPPER_OPEN);
    assert(move_angle[6]==ARM_GRIPPER_OPEN);
    tick(VISION_SERVO_SETTLE_TICKS);
    assert(vision_arm_state==ARM_APPROACH_TARGET);
    tick(ARM_APPROACH_TICKS);
    assert(vision_arm_state==ARM_WAIT_APPROACH_SETTLE);
    assert(move_angle[6]==ARM_GRIPPER_OPEN);
}
static void finish_close_and_return(void)
{
    tick(VISION_SERVO_SETTLE_TICKS);
    assert(vision_arm_state==ARM_HOLD_CLOSED_GRIPPER);
    assert(move_angle[6]==ARM_GRIPPER_CLOSE);
    tick(ARM_GRIPPER_TICKS);
    assert(vision_arm_state==ARM_RETURN_WITH_MANGO);
    tick(ARM_RETURN_TICKS+1+VISION_SERVO_SETTLE_TICKS+1);
    assert(vision_arm_state==ARM_HOLD_AT_HOME);
}
static int wire_check(const char *arg)
{
    unsigned int v[7], i;
    if (strcmp(arg,"--home")==0) {
        sent_length=0;
        Vision_SendDone();
        for(i=0;i<sent_length;i++) printf("%02X%s",sent_bytes[i],i+1==sent_length?"\n":" ");
        return sent_length==4 ? 0 : 1;
    }
    if (sscanf_s(arg,"%x %x %x %x %x %x %x",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6])!=7)
        return 2;
    reset_controller();
    for(i=0;i<7;i++) { if(v[i]>255) return 2; byte((unsigned char)v[i]); }
    tick(1);
    printf("%d %d %d %.8f %.8f\n",vision_target_valid,vision_target_x,vision_target_y,
           move_angle[1]-ARM_HOME_J1,move_angle[4]-ARM_HOME_J4);
    return 0;
}
static void test_relaxed_grasp_and_reply(void)
{
    int sx, sy;
    unsigned int k;
    for(sx=-1;sx<=1;sx+=2) for(sy=-1;sy<=1;sy+=2) {
        reset_controller();
        sent_length=0;
        /* All four inclusive corners can trigger; one frame cannot. */
        for(k=0;k<VISION_CENTER_REQUIRED_FRAMES;k++) {
            frame(320+sx*20,240+sy*20); tick(1);
            if(k+1<VISION_CENTER_REQUIRED_FRAMES)
                assert(vision_arm_state==ARM_TRACK_TARGET);
        }
        assert(vision_arm_state==ARM_GRASP_OPEN);
        tick(1);
        for(k=0;k<VISION_SERVO_SETTLE_TICKS;k++) {
            /* In-window detection jitter must not cancel opening. */
            frame(320+(k%2 ? -20 : 20),240+sy*20); tick(1);
        }
        assert(vision_arm_state==ARM_APPROACH_TARGET);
        tick(ARM_APPROACH_TICKS);
        tick(VISION_SERVO_SETTLE_TICKS);
        assert(vision_arm_state==ARM_CLOSE_GRIPPER);
        assert(!Vision_TakeDoneFlag());
        finish_close_and_return();
        /* Service the done flag as main() does, checking exact bytes once. */
        if(Vision_TakeDoneFlag()) Vision_SendDone();
        assert(sent_length==4);
        assert(sent_bytes[0]==0xEF && sent_bytes[1]==0xFE &&
               sent_bytes[2]==0x7E && sent_bytes[3]==0x6B);
        for(k=0;k<100;k++) {
            tick(1);
            if(Vision_TakeDoneFlag()) Vision_SendDone();
        }
        assert(sent_length==4);
    }
    for(sx=-1;sx<=1;sx+=2) for(sy=0;sy<=1;sy++) {
        reset_controller();
        for(k=0;k<5;k++) {
            frame(320+(sy ? 0 : sx*21),240+(sy ? sx*21 : 0)); tick(1);
        }
        assert(vision_arm_state==ARM_TRACK_TARGET);
        request_center(); tick(1);
        frame(320+(sy ? 0 : sx*21),240+(sy ? sx*21 : 0)); tick(1);
        assert(vision_arm_state==ARM_TRACK_TARGET);
    }
    reset_controller();
    frame(340,260); tick(1);
    frame(341,260); tick(1); /* excursion resets consecutive frame count */
    frame(340,260); tick(1);
    frame(340,260); tick(1);
    assert(vision_arm_state==ARM_TRACK_TARGET);
    frame(340,260); tick(1);
    assert(vision_arm_state==ARM_GRASP_OPEN);
    tick(1);
    emulate_settled_outputs=0; Position6=0; Servo6=1000;
    tick(VISION_TIMEOUT_TICKS+1);
    assert(vision_arm_state==ARM_TRACK_TARGET && move_angle[6]==ARM_GRIPPER_CLOSE);
}
int main(int argc,char **argv)
{
    float before1,before2,before3,before4;
    unsigned int k;
    if(argc==2) return wire_check(argv[1]);
    test_pid_controller();
    test_relaxed_grasp_and_reply();
    reset_controller();
    /* Pi v002 uses bottom-left coordinates: (335,255) is right/up. */
    frame(335,255); tick(1);
    assert(move_angle[1] < ARM_HOME_J1);
    assert(move_angle[4] > ARM_HOME_J4);
    reset_controller();
    frame(305,225); tick(1);
    assert(move_angle[1] > ARM_HOME_J1);
    assert(move_angle[4] < ARM_HOME_J4);
    frame(640,480); assert(vision_target_valid && vision_target_x==640 && vision_target_y==480);
    frame(0,0); assert(vision_target_valid && vision_target_x==0 && vision_target_y==0);
    frame(641,240); assert(!vision_target_valid);
    frame(320,481); assert(!vision_target_valid);
    frame(-1,240); assert(!vision_target_valid);
    reset_controller();
    frame(420,340); tick(1);
    assert(vision_arm_state==ARM_TRACK_TARGET);
    assert(move_angle[6]==ARM_GRIPPER_CLOSE);
    before1=move_angle[1]; before2=move_angle[2];
    before3=move_angle[3]; before4=move_angle[4];
    request_center();
    tick(1);
    assert(vision_arm_state==ARM_WAIT_GRIPPER_OPEN);
    assert(move_angle[6]==ARM_GRIPPER_OPEN);
    assert(move_angle[1]==before1 && move_angle[2]==before2 &&
           move_angle[3]==before3 && move_angle[4]==before4);
    tick(VISION_SERVO_SETTLE_TICKS);
    assert(vision_arm_state==ARM_APPROACH_TARGET);
    frame(341,240); tick(1);
    assert(vision_arm_state==ARM_TRACK_TARGET && Speed==VISION_TRACK_SPEED);
    reset_controller();
    begin_approach_and_check_open();
    assert(move_angle[6]==ARM_GRIPPER_OPEN);
    tick(VISION_SERVO_SETTLE_TICKS);
    assert(vision_arm_state==ARM_CLOSE_GRIPPER);
    assert(move_angle[6]==ARM_GRIPPER_CLOSE);
    tick(VISION_SERVO_SETTLE_TICKS);
    assert(vision_arm_state==ARM_HOLD_CLOSED_GRIPPER);
    tick(ARM_GRIPPER_TICKS);
    assert(vision_arm_state==ARM_RETURN_WITH_MANGO);
    tick(ARM_RETURN_TICKS+1);
    assert(vision_arm_state==ARM_WAIT_RETURN_SETTLE && !vision_done_flag);
    emulate_settled_outputs=0; Position1=0; Servo1=1000;
    tick(VISION_SERVO_SETTLE_TICKS+1);
    assert(vision_arm_state==ARM_WAIT_RETURN_SETTLE && !vision_done_flag);
    emulate_settled_outputs=1; tick(VISION_SERVO_SETTLE_TICKS+2);
    assert(vision_arm_state==ARM_HOLD_AT_HOME && Vision_TakeDoneFlag()==1);
    assert(Vision_TakeDoneFlag()==0 && Speed==VISION_TRACK_SPEED);
    for(k=0;k<25;k++) { center_frame(); tick(5); }
    assert(vision_arm_state==ARM_HOLD_AT_HOME && !vision_rearm_ready);
    tick(VISION_REARM_SILENCE_TICKS);
    assert(vision_rearm_ready && vision_arm_state==ARM_HOLD_AT_HOME);
    center_frame(); tick(1);
    assert(vision_arm_state==ARM_TRACK_TARGET);
    tick(1); center_frame(); tick(1); center_frame(); tick(1);
    assert(vision_arm_state==ARM_GRASP_OPEN);
    reset_controller();
    request_center(); tick(1);
    emulate_settled_outputs=0; Position6=0; Servo6=1000;
    for(k=0;k<VISION_SERVO_SETTLE_TIMEOUT_TICKS;k++) {
        if(k%5==0) center_frame();
        tick(1);
    }
    assert(vision_arm_state==ARM_RETURN_EMPTY);
    emulate_settled_outputs=1; tick(ARM_RETURN_TICKS+1+VISION_SERVO_SETTLE_TICKS+1);
    assert(vision_arm_state==ARM_HOLD_AT_HOME && vision_done_flag);
    reset_controller();
    request_center(); tick(1); tick(VISION_SERVO_SETTLE_TICKS);
    tick(ARM_APPROACH_TICKS);
    assert(vision_arm_state==ARM_WAIT_APPROACH_SETTLE);
    emulate_settled_outputs=0; Position1=0; Servo1=1000;
    tick(VISION_SERVO_SETTLE_TIMEOUT_TICKS);
    assert(vision_arm_state==ARM_RETURN_EMPTY && move_angle[6]==ARM_GRIPPER_OPEN);
    emulate_settled_outputs=1;
    tick(ARM_RETURN_TICKS-1);
    assert(vision_arm_state==ARM_RETURN_EMPTY && move_angle[6]==ARM_GRIPPER_OPEN);
    tick(2);
    assert(vision_arm_state==ARM_WAIT_RETURN_SETTLE && move_angle[6]==ARM_GRIPPER_CLOSE);
    tick(VISION_SERVO_SETTLE_TICKS+1);
    assert(vision_arm_state==ARM_HOLD_AT_HOME && vision_done_flag && Speed==VISION_TRACK_SPEED);
    reset_controller();
    request_center(); tick(1); tick(VISION_SERVO_SETTLE_TICKS);
    tick(ARM_APPROACH_TICKS); tick(VISION_SERVO_SETTLE_TICKS);
    assert(vision_arm_state==ARM_CLOSE_GRIPPER);
    emulate_settled_outputs=0; Position6=0; Servo6=2000;
    tick(VISION_SERVO_SETTLE_TIMEOUT_TICKS);
    assert(vision_arm_state==ARM_RETURN_EMPTY);
    emulate_settled_outputs=1; tick(ARM_RETURN_TICKS+1+VISION_SERVO_SETTLE_TICKS+1);
    assert(vision_arm_state==ARM_HOLD_AT_HOME && vision_done_flag);
    reset_controller();
    request_center(); tick(1); tick(VISION_SERVO_SETTLE_TICKS);
    approach_ok=0; tick(1);
    assert(vision_arm_state==ARM_RETURN_EMPTY);
    tick(ARM_RETURN_TICKS-1);
    assert(vision_arm_state==ARM_RETURN_EMPTY && move_angle[6]==ARM_GRIPPER_OPEN);
    tick(2);
    assert(vision_arm_state==ARM_WAIT_RETURN_SETTLE && move_angle[6]==ARM_GRIPPER_CLOSE);
    tick(VISION_SERVO_SETTLE_TICKS+1);
    assert(vision_arm_state==ARM_HOLD_AT_HOME && vision_done_flag && Speed==VISION_TRACK_SPEED);
    puts("PASS: Pi v002 coordinates, PID, grasp sequence, return and stream rearm");
    return 0;
}
