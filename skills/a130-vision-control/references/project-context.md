# 项目上下文与验证入口

本文件为 2026-10-31 快照。使用前核对实际源码；路径仅为本机定位线索。

## 位置

- 项目：C:/Users/Evo/Documents/Codex/2026-09-24/d-wheeltec-a130
- Git 远端：https://github.com/EvoHermes/gpt.git
- 固件相对目录：work/a130_src/A130_STM32F103源码J1-270度(2025-8-8)
- Pi v002：E:/树莓派农装/arm_continuous_v002_bottom_left_release_20261029_210643_187015
- 配置：MiniBalance/CONTROL/vision_config.h
- 状态机和视觉 PID：MiniBalance/CONTROL/control.c
- 串口解析：MiniBalance_HARDWARE/USART_X/usart_x.c
- 启动设置：USER/MiniBalance.c

## 当前协议及行为

USART3，115200，8N1。帧为 EF FE xL xH yL yH SUM，坐标为小端，SUM 是前六字节总和的低八位。Pi v002 使用 640×480，原图左下角边界为原点，X 向右、Y 向上，有效范围含边界：0..640、0..480，中心为 (320,240)。早期 680×480 的中心 (340,240) 不适用于这个版本。

| 用途 | 坐标 | 完整帧 | SUM |
|---|---|---|---|
| 中心 | (320,240) | EF FE 40 01 F0 00 1E | 798 & 255 = 0x1E |
| 右上 | (335,255) | EF FE 4F 01 FF 00 3C | 828 & 255 = 0x3C |
| 左下 | (305,225) | EF FE 31 01 E1 00 00 | 768 & 255 = 0x00 |

连续三帧进入中心窗口才允许抓取，X/Y 容差各为 ±20 px（x=300..340、y=220..260）；坐标超时为 750 ms。开爪等待期间仍需新鲜且在该窗口内的坐标。中心帧不是独立抓取指令。

完成回包 EF FE 7E 6B 在成功流程回程输出稳定后发送，只说明流程完成，不代表检测到抓取成功。Pi 收到后停发并等待人工重新启动；STM32 要求坐标静默 750 ms 后再允许下一轮。失败路径目前没有专门失败回包。

J6 实测定义为 0.0 rad 闭合、0.70 rad 张开。稳定判断使用 PWM 输出误差不超过 12、连续 100 ms；阶段超时 3 s。前伸距离 50 mm，仍需按实物验证。

视觉 PID 起始参数：J1 Kp/Ki/Kd 为 -0.0012/-0.00005/-0.00015，J4 为 +0.0012/+0.00005/+0.00015。输出是关节命令角速度，限幅 0.16 rad/s；用于积分的帧间隔最多 100 ms；微分低通系数 0.35；积分限幅 100 px·s。跟踪/抓取 Speed 为 15/22，它们是内层 PWM 步进限制，不能解释为实际关节角速度。具体单位和实现以 vision_config.h、control.c 为准。

## 验证入口

在项目 work/vision_tests 目录依次执行：

```powershell
python generate_regression.py
.\run_tests.cmd
python -B verify_pi_v002.py 'E:\树莓派农装\arm_continuous_v002_bottom_left_release_20261029_210643_187015'
```

run_tests.cmd 依赖本机 Visual Studio。更换 Pi 发布版时先检查对照脚本是否仍适用。回归覆盖坐标边界/方向/错误校验、PID 限速和限位、夹爪顺序、阶段超时、回程以及连续流重新使能；不验证物理运动。

Keil 程序为 D:/UV4/UV4.exe；固件工程为 USER/MiniBalance.uvprojx，输出为 OBJ/MiniBalance.hex。使用 UV4 的 -b 参数编译，把 -o 日志写到独立文本文件，避免覆盖 IDE 自身生成的 HTML 日志。检查错误/警告计数和输出是否对应当前源码。不要将包含许可证信息的完整构建日志公开上传。

已有验证记录：主机回归、Pi v002 编码器对照及 Keil 编译通过，0 错误、0 警告。尚无本轮烧录或实机联调证据；PID 参数、50 mm 前伸和方向仍需结合板载版本实测。
