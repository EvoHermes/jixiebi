from pathlib import Path

repo = Path(__file__).resolve().parents[2]
firmware = next((repo / "work/a130_src").glob("*J1-270*"))

def source(path):
    return path.read_bytes().decode("latin1").replace("\r\n", "\n")

control = source(firmware / "MiniBalance/CONTROL/control.c")
config = source(firmware / "MiniBalance/CONTROL/vision_config.h")
usart = source(firmware / "MiniBalance_HARDWARE/USART_X/usart_x.c")
prefix = """#include <assert.h>
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
"""
core = control[control.index("#define ARM_HOME_J1"):control.index("static u8 Vision_ComputeApproach")]
stub = """static int approach_ok = 1;
static u8 Vision_ComputeApproach(void)
{
    if (!approach_ok) return 0;
    move_angle[2] += 0.10f;
    move_angle[3] -= 0.05f;
    move_angle[4] += 0.03f;
    return 1;
}
"""
state = control[control.index("static void VisionArm_Update_10ms"):control.index("int TIM2_IRQHandler")]
take_done = control[control.index("u8 Vision_TakeDoneFlag"):control.index("float angle_range[5][2]={{")]
parser = usart[usart.index("static u8 vision_frame_index"):usart.index("void usart1_send")]
send_done_start = usart.index("void Vision_SendDone(void)")
send_done = usart[send_done_start:usart.index("\n}", send_done_start)+2]
tests = """static int emulate_settled_outputs = 1;
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
        for(i=0;i<sent_length;i++) printf("%02X%s",sent_bytes[i],i+1==sent_length?"\\n":" ");
        return sent_length==4 ? 0 : 1;
    }
    if (sscanf_s(arg,"%x %x %x %x %x %x %x",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6])!=7)
        return 2;
    reset_controller();
    for(i=0;i<7;i++) { if(v[i]>255) return 2; byte((unsigned char)v[i]); }
    tick(1);
    printf("%d %d %d %.8f %.8f\\n",vision_target_valid,vision_target_x,vision_target_y,
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
"""
output = prefix + config + core + stub + take_done + state + parser + send_done + tests
(repo / "work/vision_tests/vision_regression.c").write_text(output, encoding="utf-8", newline="")
print("Generated current-source regression snapshot.")
