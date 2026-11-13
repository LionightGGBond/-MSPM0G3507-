# STM32F103C8T6 与 K230 云台控制

本工程是 2025 年全国大学生电子设计竞赛 E 题的云台控制部分。STM32 通过 USART1 接收 K230 的图像偏差，通过 USART2 向双轴 F32C 电机发送速度指令。此工作区包含 STM32 固件，K230 端的识别程序及模型需单独提供。

## 工程入口

- 在 STM32CubeIDE 中导入现有工程。`K230_Target_Track.ioc` 是 CubeMX 配置。
- `Core/Src/main.c` 初始化串口，并在主循环调用云台控制任务。
- `Core/Src/target_track.c` 处理视觉数据和双轴控制。`Core/Src/f32c_motor.c` 实现电机串口通信。

循迹小车代码位于 [main 分支](https://github.com/LionightGGBond/-MSPM0G3507-/tree/main)。
