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
