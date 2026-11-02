# WHEELTEC A130 视觉抓取工程

STM32F103 + 树莓派视觉控制，当前使用 J1 270° 版固件。

## 当前工程

- 固件目录：`work/a130_src/A130_STM32F103源码J1-270度(2025-8-8)/`
- Keil 工程：该目录下 `USER/MiniBalance.uvprojx`
- 最新可烧录文件：该目录下 `OBJ/MiniBalance.hex`
- 控制参数：`MiniBalance/CONTROL/vision_config.h`
- 视觉 PID 和抓取状态机：`MiniBalance/CONTROL/control.c`

当前抓取条件为连续三帧进入中心窗口：X=300..340、Y=220..260（中心 320,240，各轴 ±20 像素）。开爪等待期间目标也需在窗口内，坐标超时 750 ms。流程为跟踪、开爪、前伸 50 mm、闭爪、收回归位。

## 树莓派通信

- USART3，115200，8N1；PB10 为 TX，PB11 为 RX。
- 坐标帧：`EF FE xL xH yL yH SUM`；小端坐标，SUM 为前六字节和的低八位。
- 图像为 640×480，左下角边界原点，X 向右、Y 向上。
- 中心帧：`EF FE 40 01 F0 00 1E`。
- 流程结束、归位输出稳定后发送一次 `EF FE 7E 6B`。此回包表示流程结束，不代表检测到了实际抓取成功；失败回程也使用此回包。
- 下一轮需坐标流先静默 750 ms，再接收新坐标。

## 验证

2026-10-31 的 ±20 像素版本已通过主机回归及 Keil 编译（0 错误、0 警告）。主机测试使用硬件和逆解桩函数，不能代替实机验证。该次交付未执行烧录。

在 `work/vision_tests` 内运行 `python generate_regression.py`，再运行 `run_tests.cmd`（需要 Visual Studio 2022 Community C 编译工具）。

## 目录说明

- `PROJECT_MEMORY.md`：交接记录及本机路径线索。
- `work/vision_tests/`：状态机及串口协议回归。
- `work/matlab_src/`：厂家 MATLAB 运动学和模型示例。
- `work/pdftext/` 及资料图片：厂家说明资料的文本和图像参考。
- `outputs/`：早期方案，协议与当前固件不同，使用前请区分。
- `skills/`：项目维护说明。

保留厂家源文件原有版权声明。仓库不包含编译缓存、许可证日志及个人 IDE 窗口状态。
