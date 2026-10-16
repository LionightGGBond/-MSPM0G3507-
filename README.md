# 天猛星 MSPM0G3507 循迹小车

## V1.0.0

实现了平滑循迹，PID调节正确，未完成MPU6050，显示器未接入KK

本仓库保存当前CCS工程完整快照，包含源码、SysConfig、工程配置、说明文档和Debug构建产物。使用的SDK及编译器安装版本见下文。

从 STM32F103 迁移到 TI DriverLib，包含五路循迹、双编码器电机、速度 PID、TB6612、SSD1306、MPU6050、LED和有源蜂鸣器。中文注释，参数集中管理。

先看 [完整 SysConfig 配置清单](docs/SysConfig配置清单.md)，按名称和引脚配置 `empty.syscfg`；配置由用户在图形界面完成。不要手改生成的 `ti_msp_dl_config.c/h`。

已用独立临时配置通过引脚求解、全部源码严格编译和固件链接。本机环境：CCS 21.0.1、MSPM0 SDK 2.11.0.07、SysConfig 1.26.2、TI Arm Clang 5.1.1 LTS。

## 行为与调参

**当前默认是连续循迹模式**（`ROBOT_MOTOR_TEST_MODE=0`）：初始化完成后等待2秒，五路数字循迹决定双轮差速目标，10ms速度PID根据编码器反馈调节PWM。每轮正向PWM最低10%，持续运行，不再100秒自动停车。IMU仅用于显示与诊断，离线不会阻止循迹启动。

丢线、全黑及其他非标准多线状态沿用最近一次有效差速；首次尚无有效线型时，以基础目标的一半（10计数/10ms）直行。线型恢复后下一拍直接按新线型转向，无需复位。中间探头对准黑线时，OLED的LINE应为 `00100`，左右目标均为20计数/10ms；黑线偏左降低左轮目标，偏右降低右轮目标。串口显示“模式=循迹”，OLED显示WAIT→RUN；`HOLD:1`表示无效线型时保持差速，恢复后为0。

PWM最低10%是驱动命令下限，不代表实际转速为满速的10%，也不能保证克服负载静摩擦。连续模式PID只输出正向10%..100%，超速时最低仍为10%，不主动反转或制动。编码器反馈缺失默认只统计、不停车，失去反馈时PID仍可能升到最大PWM。配置错误、控制时基异常和主循环超时仍锁存停车；启动等待2秒仍保持刹车。务必验证方向和编码器计数，再落地调试。

若需要重新启用台架测试，将 `robot_config.h` 中 `ROBOT_MOTOR_TEST_MODE` 改为1，重新编译下载，即恢复两轮固定30% PWM转2秒/停2秒循环。Type-C串口诊断的完整配置及测试模式操作见 [电机循环测试与串口诊断](docs/电机循环测试与串口诊断.md)。s/r目前只用于台架测试，循迹模式不通过它们暂停/继续。

常用参数全部在 [robot_config.h](robot_config.h)。

```c
#define ROBOT_RUN_MS                 0U  /* 0=不按时长停车 */
#define ROBOT_LINE_MIN_PWM           10U /* 每轮正向最低PWM%，需大于驱动死区 */
#define ROBOT_STOP_ON_INVALID_LINE   0   /* 0=保持差速；1=无效线型停车锁存 */
#define ROBOT_STOP_ON_ENCODER_STALL  0   /* 0=只统计反馈缺失；1=停车锁存 */
```

修改后重新编译下载。运行时限、无效线型、反馈缺失停车分别由各自开关控制；主循环超时和控制周期异常检查始终保留。希望恢复100秒结束时设置 `ROBOT_RUN_MS=100000U`。

速度单位是编码器四倍频计数/10ms，不是PWM百分比或cm/s。沿用原九档差速表、PID初始系数、积分限幅30、积分分离阈值30、PWM死区5；修正了零目标积分和积分分离逻辑，增加条件积分抗饱和，首周期不产生微分突跳。当前硬件仍需重新整定PID。

OLED使用 I2C1：**PB2=SCL、PB3=SDA**；MPU6050使用 I2C0：PA1=SCL、PA0=SDA。左轮为TB6612 B通道，右轮为A通道，STBY固定3.3V。左编码器TIMG8硬件QEI，右编码器GPIO AB双边沿解码。

OLED分三行显示Pitch、Roll、Yaw，保留正负号和一位小数；右上角显示 `H:000001*`，计数每秒加一，星号与PA22心跳灯同相（亮500ms、灭500ms），故障停车时仍更新。状态、循迹位和左右轮目标/实测速度、PWM命令同时显示；定时器故障时底部改为LOAD、IRQ次数和DT诊断。IMU使用四元数互补滤波，显示静止标定进度及ACC/GYR校正状态，支持0x68/0x69地址探测及初始化错误诊断。右编码器非法跳变次数仍可在调试器的快照中查看。ASCII字库复用原M0use工程。姿态只用于显示，Yaw为相对角，会漂移。算法与参数见 [IMU角度与互补滤波](docs/IMU角度与互补滤波.md)。K230和无刷云台目前预留资源，接口或协议确定后再接入。

## 代码结构

| 文件/目录 | 职责 |
| --- | --- |
| `empty.c` | 唯一main，初始化及主循环 |
| `robot_config.h` | PID、速度、时间、极性和功能开关 |
| `Motion/` | 控制状态机、PID、九档循迹表、AB状态表 |
| `Bsp/` | GPIO、编码器、TB6612、硬件I2C、SysTick；启动包装仅用于独立命令行验证，CCS自动加入官方启动文件 |
| `Devices/` | SSD1306和MPU6050 |
| `App/robot_app.c` | 10ms中断和主循环分工、一致性快照、显示 |
| `robot_link_options.cmd` | 2KB栈及向量表保留 |
| `makefile.defs` | CCS链接顺序钩子，确保2KB栈设置最后生效 |
| `tools/sysconfig_reference.txt` | 已验证的临时配置参考，不覆盖用户syscfg |
| `tools/verify_build.py` | 真实SysConfig及TI编译器的离线验证脚本 |

## 编译验证

输出目录必须在CCS工程外，避免重复编译临时生成的C文件：

```powershell
& 'C:/Users/lenovo/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' tools/verify_build.py --output 'C:/Users/lenovo/.codex/visualizations/2026/10/16/01a14328-1290-79c0-bb2e-8247dec63166/robot_build'
```

安装目录改变时传入 `--sdk`、`--compiler`、`--sysconfig`。脚本启用 `-Wall -Wextra -Werror`。

编译通过不等于上板验证。接线、方向核验、计数速率与调试步骤见 [调试与扩展](docs/调试与扩展.md)。原STM32工程及旧M0use移植稿未改动。
