# STM32 视觉平面坐标接口

## 坐标约定

视觉模块发送的 `x/y` 必须是 **A130 基座坐标系** 下的毫米值，不是图像像素：

- `x_mm` 对应固件 `px`，单位 mm；
- `y_mm` 对应固件 `py`，单位 mm；
- 正方向必须和资料包坐标系图一致；
- `z` 暂时固定在 `VISUAL_Z_FIXED_M`，默认 0.30 m；
- 末端姿态固定为固件中的 `rx=0, ry=0`。

## 数据帧

```text
A5 5A XH XL YH YL FLAGS XOR 0A
```

`X/Y` 是大端有符号 `int16`，单位毫米。`FLAGS bit0=1` 开启跟随，`0` 停止跟随。`XOR` 是前 7 个字节的异或。

例如 `(x,y)=(230,0) mm`：

```text
A5 5A 00 E6 00 00 01 18 0A
```

## 加入工程

1. 将 `stm32_visual_xy.c/.h` 加入 Keil 工程，并确认头文件能包含 `sys.h`、`control.h`。
2. 在 `usart_x.c` 顶部加入：

```c
#include "stm32_visual_xy.h"
```

3. 在视觉模块使用的串口接收中断，读取到字节后调用：

```c
VisualXY_RxByte(Usart_Receive);
```

资料包默认 `USART1` 是 115200，`USART3` 是 9600。建议视觉模块接 `USART1`，避免和原有 16 字节 MoveIt 控制帧冲突；如果接 `USART3`，同样在 `USART3_IRQHandler()` 中调用该函数即可。

4. 在已有 10 ms 控制中断 `TIM2_IRQHandler()` 的主体开头调用：

```c
VisualXY_Task();
```

原有代码随后会执行：

```c
if(mode) ik_kinematics(px, py, pz, rx, ry);
```

所以不需要重复写逆运动学。

## 参数需要调整的位置

在 `stm32_visual_xy.c` 顶部修改：

```c
#define VISUAL_X_OFFSET_M  0.0f
#define VISUAL_Y_OFFSET_M  0.0f
#define VISUAL_Z_FIXED_M   0.30f
```

如果视觉坐标原点是相机而不是机械臂基座，需要先做手眼/平面标定，得到两个偏置后填入 `VISUAL_X_OFFSET_M`、`VISUAL_Y_OFFSET_M`。代码还带有工作空间限幅和每次最多 4 mm 的移动限速。

## 根据舵机角度计算夹爪位置

资料包原有的 `control.c` 已经有 `fk_kinematics()`，但它会直接改写全局 `px/py/pz`。如果只想读取位置而不影响当前控制，可把 `stm32_a130_fk.c/.h` 加入工程，然后调用：

```c
float hand_x, hand_y, hand_z;
A130_FK(move_angle[1], move_angle[2], move_angle[3],
        move_angle[4], move_angle[5],
        &hand_x, &hand_y, &hand_z);
```

输入必须是 J1～J5 的机械关节角，单位弧度；J6 只是夹爪开合，不参与位置计算。舵机 PWM 中的零点偏置、方向和安装误差必须先通过 `Angle*_init` 等参数校准，否则解算的是理论位置而不是实际夹爪尖端位置。

## 安全测试

先不接舵机或架空机械臂，发送：

```text
python visual_xy_protocol.py
```

确认模块能发出数据后，再发送几个小范围点，例如 `(230,0)`、`(220,20)`、`(210,-20)`。方向相反时，不要先改逆运动学，优先交换视觉模块的 `x/y` 符号或修改偏置/坐标转换。
