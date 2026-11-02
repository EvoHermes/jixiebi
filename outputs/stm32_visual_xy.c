#include "stm32_visual_xy.h"
#include "control.h"

/*
 * Frame sent by the vision module (9 bytes, big endian):
 *   A5 5A  XH XL  YH YL  FLAGS  XOR  0A
 *
 * X/Y are signed millimetres in the A130 base coordinate system.
 * XOR is bytes 0..6 XORed together. FLAGS bit 0 enables tracking.
 */
#define VISUAL_HEADER_1       0xA5
#define VISUAL_HEADER_2       0x5A
#define VISUAL_TAIL           0x0A
#define VISUAL_FRAME_LENGTH   9

/* Set these after the camera-to-base calibration. */
#define VISUAL_X_OFFSET_M     0.0f
#define VISUAL_Y_OFFSET_M     0.0f
#define VISUAL_Z_FIXED_M      0.30f

/* Conservative workspace and per-update slew limits. */
#define VISUAL_X_MIN_M       -0.24f
#define VISUAL_X_MAX_M        0.24f
#define VISUAL_Y_MIN_M       -0.24f
#define VISUAL_Y_MAX_M        0.24f
#define VISUAL_Z_MIN_M        0.18f
#define VISUAL_Z_MAX_M        0.36f
#define VISUAL_MAX_STEP_M     0.004f

volatile u8 visual_xy_valid = 0;
volatile u8 visual_xy_enabled = 0;

static u8 rx_state = 0;
static u8 rx_index = 0;
static u8 rx_frame[VISUAL_FRAME_LENGTH];
static volatile s16 pending_x_mm = 0;
static volatile s16 pending_y_mm = 0;
static volatile u8 pending_flags = 0;
static volatile u8 pending_update = 0;
static u8 missed_ticks = 0;

static float limitf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

void VisualXY_RxByte(u8 byte)
{
    u8 i;
    u8 checksum = 0;

    if (rx_state == 0)
    {
        if (byte == VISUAL_HEADER_1)
        {
            rx_frame[0] = byte;
            rx_state = 1;
        }
        return;
    }

    if (rx_state == 1)
    {
        if (byte == VISUAL_HEADER_2)
        {
            rx_frame[1] = byte;
            rx_index = 2;
            rx_state = 2;
        }
        else
        {
            rx_state = (byte == VISUAL_HEADER_1) ? 1 : 0;
        }
        return;
    }

    rx_frame[rx_index++] = byte;
    if (rx_index < VISUAL_FRAME_LENGTH) return;
    rx_state = 0;

    if (rx_frame[8] != VISUAL_TAIL) return;
    for (i = 0; i < 7; ++i) checksum ^= rx_frame[i];
    if (checksum != rx_frame[7]) return;

    pending_x_mm = (s16)(((u16)rx_frame[2] << 8) | rx_frame[3]);
    pending_y_mm = (s16)(((u16)rx_frame[4] << 8) | rx_frame[5]);
    pending_flags = rx_frame[6];
    pending_update = 1;
}

void VisualXY_Task(void)
{
    float target_x;
    float target_y;
    float dx;
    float dy;

    if (pending_update)
    {
        /* Copy the ISR mailbox once; the next frame can arrive asynchronously. */
        target_x = ((float)pending_x_mm * 0.001f) + VISUAL_X_OFFSET_M;
        target_y = ((float)pending_y_mm * 0.001f) + VISUAL_Y_OFFSET_M;
        visual_xy_enabled = (pending_flags & 0x01) ? 1 : 0;
        pending_update = 0;
        visual_xy_valid = 1;
        missed_ticks = 0;

        target_x = limitf(target_x, VISUAL_X_MIN_M, VISUAL_X_MAX_M);
        target_y = limitf(target_y, VISUAL_Y_MIN_M, VISUAL_Y_MAX_M);

        if (visual_xy_enabled)
        {
            dx = limitf(target_x - px, -VISUAL_MAX_STEP_M, VISUAL_MAX_STEP_M);
            dy = limitf(target_y - py, -VISUAL_MAX_STEP_M, VISUAL_MAX_STEP_M);
            px += dx;
            py += dy;
            pz = limitf(VISUAL_Z_FIXED_M, VISUAL_Z_MIN_M, VISUAL_Z_MAX_M);
            mode = 1;
        }
    }
    else if (missed_ticks < 100)
    {
        ++missed_ticks;
    }

    /* Stop accepting motion after about one second without a valid frame. */
    if (missed_ticks >= 100)
    {
        visual_xy_valid = 0;
        visual_xy_enabled = 0;
    }
}
