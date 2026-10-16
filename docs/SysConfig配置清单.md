# SysConfig 完整配置清单

适用标准天猛星 MSPM0G3507、LQFP-64(PM)，SDK 2.11.0.07 / SysConfig 1.26.2。以下当前资源已用本机SysConfig实际求解、生成并编译。
在 `empty.syscfg` 图形界面配置；名称是**实例名和子引脚名**，不能只改备注。精确参考值也见 `tools/sysconfig_reference.txt`。

## 1. 时钟、调试和运行模式

| 项目 | 配置 |
| --- | --- |
| Device / Package | MSPM0G3507 / LQFP-64(PM) |
| SYSCTL | Force Default Clock Configuration，SYSOSC=32 MHz |
| MCLK、ULPCLK、BUSCLK | 均32 MHz，分频1 |
| SYSPLL / HFXT / LFXT | 不使用，不依赖外部晶振 |
| DEBUG | 保留PA19=SWDIO、PA20=SWCLK |
| SysTick | 代码已配置1ms；不要重复创建 |
| 低功耗 | 主循环不进入STOP/STANDBY |

先固定32MHz，未来改80MHz要同步修改定时器、I2C和代码时钟校验。不要启用“全部未使用引脚输出低”占用扩展脚。

## 2. 接线总表

| 模块信号 | MCU引脚 | 功能 | 说明 |
| --- | --- | --- | --- |
| 右轮PWMA | PB15 | TIMG7.CCP0 | TB6612 A通道 |
| 左轮PWMB | PB16 | TIMG7.CCP1 | TB6612 B通道 |
| AIN1 / AIN2 | PA15 / PA16 | GPIO输出 | 右轮方向 |
| BIN1 / BIN2 | PB0 / PB1 | GPIO输出 | 左轮方向 |
| STBY | 固定3.3V | 不占GPIO | 按用户硬件接法 |
| 左编码器A / B | PB6 / PB7 | TIMG8.CCP0 / CCP1 | 硬件QEI |
| 右编码器A / B | PA24 / PA25 | GPIO双边沿 | 软件AB解码 |
| 循迹L2、L1、MID、R1、R2 | PB13、PB12、PB11、PB10、PB9 | GPIO输入 | 从车头向前时的左侧到右侧 |
| OLED SCL / SDA | **PB2 / PB3** | **I2C1** | 注意顺序 |
| MPU6050 SCL / SDA | PA1 / PA0 | I2C0 | AD0低，地址0x68 |
| 心跳LED | PA22 | GPIO输出 | 高有效 |
| 左、右LED | PB14、PB18 | GPIO输出 | 高有效 |
| 有源蜂鸣器 | PA21 | GPIO输出 | 高电平响 |
| Type-C调试TX / RX | PA10 / PA11 | UART0 | 板载CH340E，实例DEBUG_UART |

MCU与外设共地。逻辑VCC和信号按已确认的3.3V设计；TB6612 VM按电机规格供电。两路I2C的SCL/SDA各需上拉到3.3V，常用4.7kΩ，模块自带时核对等效阻值。

## 3. GPIO输出：三个组

创建以下GPIO组，均Output、推挽、不反相、内部上下拉None、不开中断。

| Group实例名 | 子引脚名 | Pin | Initial Value |
| --- | --- | --- | --- |
| `GPIO_MOTOR` | `AIN1` | PA15 | Set / High |
| `GPIO_MOTOR` | `AIN2` | PA16 | Set / High |
| `GPIO_MOTOR` | `BIN1` | PB0 | Set / High |
| `GPIO_MOTOR` | `BIN2` | PB1 | Set / High |
| `GPIO_LED` | `HEART` | PA22 | Cleared / Low |
| `GPIO_LED` | `LEFT` | PB14 | Cleared / Low |
| `GPIO_LED` | `RIGHT` | PB18 | Cleared / Low |
| `GPIO_BUZZER` | `OUT` | PA21 | Cleared / Low |

电机IN初始11为短路制动。混合端口组会为每个引脚生成 `_PORT` 宏；不要把组拆成不同名字。
若外接LED模块是低有效，改 `ROBOT_LED_ACTIVE_LEVEL=0`，并将LED初值改High。

## 4. GPIO输入：两个组

创建 `GPIO_LINE`：

| 子引脚名 | Pin | Direction | Internal Resistor | Interrupt |
| --- | --- | --- | --- | --- |
| `L2` | PB13 | Input | Pull-down | Disabled |
| `L1` | PB12 | Input | Pull-down | Disabled |
| `MID` | PB11 | Input | Pull-down | Disabled |
| `R1` | PB10 | Input | Pull-down | Disabled |
| `R2` | PB9 | Input | Pull-down | Disabled |

不反相，保留器件默认数字输入迟滞。黑线高，bit0最左、bit4最右。弱下拉使断线输入趋向丢线。

创建 `GPIO_ENCODER`：

| 子引脚名 | Pin | Direction | Internal Resistor | Interrupt | Polarity |
| --- | --- | --- | --- | --- | --- |
| `RIGHT_A` | PA24 | Input | Pull-down | Enabled | Rise and Fall |
| `RIGHT_B` | PA25 | Input | Pull-down | Enabled | Rise and Fall |

不反相，保留默认迟滞。当前按编码器已有3.3V输出/上拉配置；裸开集电极AB若没有上拉，需外接上拉到3.3V，不能靠下拉产生高电平。
右轮ISR为 `GROUP1_IRQHandler`，工程中只保留这一份；NVIC由代码开启。

## 5. 电机PWM：实例 `MOTOR_PWM`

Add PWM，实例名必须为 `MOTOR_PWM`。

| 字段 | 值 |
| --- | --- |
| Peripheral | TIMG7 |
| PWM Mode | Edge-aligned Down Counting / `EDGE_ALIGN` |
| Timer Clock Source / Divider | BUSCLK / 1 |
| Clock Prescale | 1，生成寄存器prescale=0 |
| PWM Period Count | **1600**，生成LOAD=1599；配置脚本中的属性名是 `timerCount` |
| Channel 0 Pin | PB15，右轮PWMA |
| Channel 1 Pin | PB16，左轮PWMB |
| 两通道Initial Value | Low |
| 两通道Invert | False |
| 两通道Desired Duty Cycle | 0% |
| 两通道Counter Compare Value | **1599**，设置Duty后手动核对此值 |
| 两通道Channel Update Mode | Zero Event / `ZERO_EVT` |
| Start Timer | Disabled / Stop，由代码启动 |
| Complementary / Deadband / Interrupt / DMA / Event | 均关闭 |

`PWM Period Count` 位于Basic Configuration中、Clock Configuration下面，紧邻 `Calculated PWM Frequency (Hz)`；它与 `Timer Clock Prescale` 是不同字段。设为1600后，Calculated PWM Frequency应显示20000 Hz。

32,000,000 / 1600 = **20 kHz**。本机工具0%时可能生成比较值1600，参考配置显式设1599；停车由IN1=IN2=1可靠制动。
下数PWM的比较值越小，占空比越大，不能照搬STM32的CCR公式。非零驱动避开装载边界，最大1599/1600≈99.94%，不是严格100%。

## 6. 左轮QEI：实例 `ENCODER_LEFT`

在左侧Software列表找到 **TIMER - QEI**，点旁边的“+”添加一个实例，把Name改为 `ENCODER_LEFT`。这是单独的QEI模块，不是在已有的TIMER - PWM中切换。

| 字段 | 值 |
| --- | --- |
| Timer Clock Source | BUSCLK |
| Timer Clock Divider | Divided by 1 |
| Timer Clock Prescale | 1 |
| Start Timer | 不勾选，由Encoder_Init启动 |
| Enable QEI 3 Pin Mode | 不勾选，即PHA/PHB两输入，不使用IDX |
| Enable QEI Hall Input Mode（若显示） | 不勾选 |
| Load Value | 65535（或0xFFFF），16bit回绕 |
| Enable Cross Trigger | 不勾选 |
| Interrupts Configuration → Enable Interrupts | 不选择任何事件 |
| Event Configuration | 保持未启用 |
| PinMux → QEI Peripheral | **TIMG8** |
| PinMux → Capture/Compare Pin 0 | **PB6**，左轮编码器A/PHA |
| PinMux → Capture/Compare Pin 1 | **PB7**，左轮编码器B/PHB |

界面没有单独的“Mode=2 Input”下拉框；不勾选3 Pin Mode就是两输入。生成代码默认两路不反相，无需另找Input Inversion字段。PB6/PB7已交给QEI后，不再添加普通GPIO输入或GPIO中断来占用这两脚。

生成 `DL_TimerG_configQEI`，不存在旧移植稿所用的 `DL_QEI_*` API。只有TIMG8支持硬件QEI，不能添加第二个。
代码读取相邻计数差，不每10ms清零硬件计数器。两轮均按四倍频统计。

## 7. 控制定时器：实例 `CONTROL_TIMER`

在左侧Software列表的 **TIMER** 旁点“+”新增一个实例，Name改为 `CONTROL_TIMER`。本实例是普通周期定时器，使用独立的TIMG0，不在已有的TIMER - PWM或TIMER - QEI中配置。

| 字段 | 值 |
| --- | --- |
| PinMux中的Timer Peripheral | TIMG0 |
| Timer Mode | Periodic Down Counting / `PERIODIC` |
| Timer Clock Source | BUSCLK |
| Timer Clock Divider | Divided by 1 |
| Timer Clock Prescaler | **32**，生成寄存器prescale=31 |
| Desired Timer Period | **10 ms** |
| Actual Timer Clock | 1,000,000 Hz |
| Generated LOAD | **9999** |
| Interrupts Configuration → Enable Interrupts | 仅勾选 **Zero event** |
| Start Timer | 不勾选，由代码启动 |
| CCP引脚 / Trigger / DMA / Event | 不需要 |

在Basic Configuration→Clock Configuration中设置时钟；Desired Timer Period填 `10 ms` 或 `0.01 s`，不能只填无单位的10（会被解析为10秒）。Calculated Timer Clock/Timer Clock Frequency应为1000000 Hz，Actual Timer Period应为10ms。LOAD=9999是生成结果，不是PWM Period Count，也不需要手工寻找输入框。

不要分配外部CCP脚。生成的 `CONTROL_TIMER_INST_IRQHandler` 展开为 `TIMG0_IRQHandler`。
ISR执行：编码器采样→循迹状态与目标→PID→电机输出。

## 8. 两条I2C

Add I2C两次，均启用Controller，不用Target模式。

| 字段 | `OLED_I2C` | `IMU_I2C` |
| --- | --- | --- |
| Peripheral | I2C1 | I2C0 |
| SCL / SDA | **PB2 / PB3** | PA1 / PA0 |
| Controller Bus Speed | 400000 Hz | 400000 Hz |
| Clock Source | BUSCLK 32MHz | BUSCLK 32MHz |
| Basic Configuration → Clock Configuration → Clock Source Divider | Divide By 1 | Divide By 1 |
| Basic Configuration → Enable Controller Mode | 勾选 | 勾选 |
| TX / RX FIFO | 代码使用，硬件深度固定8字节，不需填写8 | 同左 |
| Analog / Digital Glitch Filter | SDK默认 | SDK默认 |
| Interrupt / DMA | 关闭 | 关闭 |
| 软件7位器件地址 | 默认0x3C | 默认0x68 |

器件地址由代码发起事务时传入，不能用MCU自身Target Address替代。OLED地址不能填0x78。
`Enable Target Mode` 不勾选。总线频率在I2C Controller Basic Configuration的 `Custom Bus Speed (Hz)` 填400000；若显示 `Standard Bus Speed` 下拉框，则选 `Fast Mode (400kHz)`。
滤波项在Advanced Configuration中，保持默认；其中FIFO Trigger Level是触发阈值，不是FIFO容量，不要因为深度8就全部改成8。
Interrupt Configuration里的Controller/Target/Timeout Interrupts均不选；DMA Configuration里的Enable DMA Event 1/2 Trigger不勾选。
软件器件地址在 `robot_config.h` 的 `ROBOT_OLED_ADDRESS`（0x3C）和 `ROBOT_MPU_ADDRESS`（0x68），不在SysConfig中填写。
OLED分成控制字节+最多7字节数据的小包；MPU6050的14字节接收持续排空RX FIFO。超时会结束本次事务并返回错误，不会无限等待。

## 9. 中断及工程核对

| 中断 | 优先级 | 来源 |
| --- | --- | --- |
| GPIOA / GROUP1 | 0，最高 | GUI开双边沿，代码开NVIC |
| SysTick | 1 | 代码1ms时基 |
| TIMG0 | 2 | GUI开Zero，代码开NVIC并启动 |
| TIMG7 / TIMG8 / I2C0 / I2C1 | 不开NVIC | 不需要ISR |

保存后，生成头文件应含 `MOTOR_PWM_INST`、`CONTROL_TIMER_INST`、`ENCODER_LEFT_INST`、`OLED_I2C_INST`、`IMU_I2C_INST`。
单端口组生成 `GPIO_LINE_PORT`、`GPIO_ENCODER_PORT`、`GPIO_BUZZER_PORT`；混合端口组生成例如 `GPIO_MOTOR_AIN1_PORT`。改端口组织可能改变宏名。

原空工程已有根目录、构建目录、SDK/source和CMSIS include path。源码使用相对include，不需逐个加子目录。
仅保留 `empty.c` 一份main。CCS会自动加入SDK官方启动文件；`Bsp/startup.c` 在CCS中不生成第二份向量表，只在独立验证脚本定义 `ROBOT_STANDALONE_STARTUP` 时引用官方实现。链接生成的 `device_linker.cmd` 和根目录 `robot_link_options.cmd`。
根目录 `makefile.defs` 利用CCS预留的构建钩子，将 `robot_link_options.cmd` 放在生成的链接文件之后，避免默认512覆盖2048。不要删去此文件，也不要手改Debug中的自动生成makefile。Build后map中 `.stack` 长度应为 `0x800`（2048）。

## 10. 后续预留

| 用途 | 资源 | 当前处理 |
| --- | --- | --- |
| Type-C调试 | UART0：PA10=TX、PA11=RX | 当前调试占用，按下方说明新增DEBUG_UART |
| K230 | UART需重新选择 | 原UART0现用于Type-C，不能同时接收两端TX |
| 云台串口候选 | UART1：PA8=TX、PA9=RX | 与当前引脚无冲突 |
| 云台CAN候选 | CANFD0：PA12=TX、PA13=RX | 需要CAN收发器 |
| 云台PWM/功率控制候选 | TIMA0、TIMA1 | 均未用，最终需重新分配通道 |
| 其他计时 | TIMG6、TIMG12 | 未用 |
| SPI | SPI0、SPI1 | 未启用；接口确定后再求解引脚 |
| 调试/启动/板载按键 | PA19/PA20、PA18、PB21 | 保留 |

Type-C调试实例名为 `DEBUG_UART`：UART0、TX=PA10、RX=PA11、BUSCLK32MHz分频1、115200、8N1、TX/RX、无流控、FIFO启用、内部回环关闭、Interrupt/DMA关闭。完整GUI步骤及命令见 [电机循环测试与串口诊断](电机循环测试与串口诊断.md)。UART0已连板载CH340E；K230后续另选串口或明确隔离CH340的RX来源，不能将两个TX直接并到PA11。
两个高级定时器空闲不意味着任意PWM脚都空闲，后续不可未经核对直接启用。
TB6612用于有刷底盘，不能驱动三相无刷云台；云台驱动器未定，本次不猜测驱动协议。

## 11. 依据

- [TI数据手册](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf)：引脚复用及TIMG8 QEI。
- 本机SDK的 `mspm0g350x.h`、`dl_timer.h`、`dl_i2c.h`，以及 `timg_qei_mode`、`gpio_simultaneous_interrupts`、`timx_timer_mode_periodic_sleep`、`i2c_controller_rw_multibyte_fifo_poll` 官方例程。
- [TI GPIO/QEI解码演示](https://www.ti.com/video/6376539608112)。
- [立创天猛星文档入口](https://wiki.lckfb.com/zh-hans/tmx-mspm0g3507/)。
