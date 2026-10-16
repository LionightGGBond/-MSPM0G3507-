#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* 集中调参入口。速度单位始终为“四倍频编码器计数 / 10 ms”，不是百分比。 */
#define ROBOT_CONTROL_MS          10U
#define ROBOT_START_DELAY_MS      2000U
#define ROBOT_RUN_MS              0U /* 0=持续循迹，不按运行时长停车。 */
#define ROBOT_BASE_SPEED          20.0f
#define ROBOT_PWM_LIMIT           100.0f
#define ROBOT_PWM_DEADBAND        5.0f
#define ROBOT_LINE_MIN_PWM        10U /* 循迹运行时每轮最小正向PWM百分比。 */

/* 台架测试：1=固定PWM转2秒/刹车2秒循环；0=原循迹+速度PID。
 * 测试模式不检查循迹和编码器堵转，便于排查反馈接线；请架空车轮。
 * 固定PWM不是恒转速。负值可测试反转，0可单独停止某一轮。 */
#ifndef ROBOT_MOTOR_TEST_MODE
#define ROBOT_MOTOR_TEST_MODE      0
#endif
#define ROBOT_TEST_ON_MS           2000U
#define ROBOT_TEST_OFF_MS          2000U
#define ROBOT_TEST_LEFT_PWM        30
#define ROBOT_TEST_RIGHT_PWM       30
#define ROBOT_DEBUG_REPORT_MS      500U

/* 沿用 STM32 初始系数：离散 PID 的 Ki、Kd 已包含固定采样周期。
 * 改采样周期或编码器倍频后必须重新整定，不能直接沿用这些系数。 */
#define ROBOT_LEFT_KP             1.413f
#define ROBOT_LEFT_KI             0.79f
#define ROBOT_LEFT_KD             0.03f
#define ROBOT_RIGHT_KP            1.40f
#define ROBOT_RIGHT_KI            0.81f
#define ROBOT_RIGHT_KD            0.06f
#define ROBOT_INTEGRAL_LIMIT      30.0f
#define ROBOT_I_SEPARATE          30.0f

/* 电机接线和编码器相序分别校正；务必架空车轮验证“向前=正计数”。 */
#define ROBOT_LEFT_MOTOR_SIGN     1
#define ROBOT_RIGHT_MOTOR_SIGN    1
/* TB6612四个方向输入的统一设置位置：0=低电平，1=高电平。
 * 以下为驱动正向时的电平；反向时同一通道IN1/IN2互换。
 * A通道=右轮，B通道=左轮。停车始终强制四脚全高，保持短路制动。
 * 每组正向电平须一高一低。改某轮方向时也可以只改上方MOTOR_SIGN。 */
#define ROBOT_TB6612_AIN1_FORWARD_LEVEL  0U  /* PA15，右轮AIN1 */
#define ROBOT_TB6612_AIN2_FORWARD_LEVEL  1U  /* PA16，右轮AIN2 */
#define ROBOT_TB6612_BIN1_FORWARD_LEVEL  0U  /* PB0， 左轮BIN1 */
#define ROBOT_TB6612_BIN2_FORWARD_LEVEL  1U  /* PB1， 左轮BIN2 */
#define ROBOT_LEFT_ENCODER_SIGN   1
#define ROBOT_RIGHT_ENCODER_SIGN  1
#define ROBOT_LINE_ACTIVE_LEVEL   1U
/* 1=丢线/多线停车锁存；0=沿用最近有效差速，首次无有效线型时低速直行。 */
#ifndef ROBOT_STOP_ON_INVALID_LINE
#define ROBOT_STOP_ON_INVALID_LINE 0
#endif

/* 连续有驱动却无反馈的诊断门限；STOP_ON_ENCODER_STALL=1才停车。 */
#define ROBOT_STALL_PWM           35.0f
#define ROBOT_STALL_MS            500U
#ifndef ROBOT_STOP_ON_ENCODER_STALL
#define ROBOT_STOP_ON_ENCODER_STALL 0 /* 0=仅诊断反馈缺失，不因此停车。 */
#endif
#define ROBOT_MAIN_TIMEOUT_MS     250U

/* 器件地址均为 7 位地址，不能填 0x78。AD0 接高时 MPU 地址改为 0x69。 */
#define ROBOT_OLED_ADDRESS        0x3CU
#define ROBOT_MPU_ADDRESS         0x68U
/* AD0接高时为0x69。先尝试上面的地址，未应答时可自动探测另一地址。 */
#define ROBOT_IMU_AUTO_ADDRESS    1
#define ROBOT_DISPLAY_MS          100U
#define ROBOT_IMU_MS              10U
#define ROBOT_I2C_TIMEOUT_MS       5U
#define ROBOT_LED_ACTIVE_LEVEL    1U
#define ROBOT_BUZZER_ACTIVE_LEVEL 1U
#define ROBOT_IMU_CALIBRATION_SAMPLES 100U
/* IMU四元数互补滤波：Kp越大越快跟随重力，但更易受车身振动影响。
 * Ki用于缓慢修正残余零偏；设为0可关闭在线积分修正。 */
#define ROBOT_IMU_CF_KP           2.0f
#define ROBOT_IMU_CF_KI           0.02f
#define ROBOT_IMU_CF_I_LIMIT_DPS  3.0f
#define ROBOT_IMU_ACCEL_MIN_G     0.85f
#define ROBOT_IMU_ACCEL_MAX_G     1.15f
#define ROBOT_IMU_MAX_DT_MS       50U
/* 标定必须静止：允许角速率上限及窗口内标准差上限，单位°/s。 */
#define ROBOT_IMU_CAL_MAX_DPS     3.0f
#define ROBOT_IMU_CAL_STD_DPS     0.5f

#if ROBOT_CONTROL_MS != 10U
#error "采样周期改变时，请同步修改 SysConfig 定时器和重新整定 PID。"
#endif
#if ROBOT_LINE_MIN_PWM < 1U || ROBOT_LINE_MIN_PWM > 100U
#error "循迹最低PWM必须为1..100的整数，并需大于驱动PWM死区。"
#endif
#if ROBOT_TEST_ON_MS == 0U || ROBOT_TEST_OFF_MS == 0U || \
    ROBOT_TEST_ON_MS > 60000U || ROBOT_TEST_OFF_MS > 60000U
#error "测试转动/停车时长须为1..60000毫秒。"
#endif
#if ROBOT_TEST_LEFT_PWM < -100 || ROBOT_TEST_LEFT_PWM > 100 || \
    ROBOT_TEST_RIGHT_PWM < -100 || ROBOT_TEST_RIGHT_PWM > 100
#error "测试PWM须为-100..100的整数。"
#endif
#if ROBOT_MOTOR_TEST_MODE != 0 && ROBOT_MOTOR_TEST_MODE != 1
#error "ROBOT_MOTOR_TEST_MODE只能为0或1。"
#endif
#if ROBOT_TB6612_AIN1_FORWARD_LEVEL > 1U || ROBOT_TB6612_AIN2_FORWARD_LEVEL > 1U || \
    ROBOT_TB6612_BIN1_FORWARD_LEVEL > 1U || ROBOT_TB6612_BIN2_FORWARD_LEVEL > 1U
#error "TB6612方向输入只能设置为0或1。"
#endif
#if ROBOT_TB6612_AIN1_FORWARD_LEVEL == ROBOT_TB6612_AIN2_FORWARD_LEVEL || \
    ROBOT_TB6612_BIN1_FORWARD_LEVEL == ROBOT_TB6612_BIN2_FORWARD_LEVEL

#endif
#if ROBOT_IMU_CALIBRATION_SAMPLES == 0U
#error "IMU零偏标定样本数必须大于0。"
#endif
#if ROBOT_IMU_MAX_DT_MS < ROBOT_IMU_MS
#error "IMU最大积分间隔不能小于采样周期。"
#endif
#if (ROBOT_LEFT_ENCODER_SIGN != 1 && ROBOT_LEFT_ENCODER_SIGN != -1) || \
    (ROBOT_RIGHT_ENCODER_SIGN != 1 && ROBOT_RIGHT_ENCODER_SIGN != -1)
#error "编码器极性只能为 +1 或 -1。"
#endif
#if (ROBOT_LEFT_MOTOR_SIGN != 1 && ROBOT_LEFT_MOTOR_SIGN != -1) || \
    (ROBOT_RIGHT_MOTOR_SIGN != 1 && ROBOT_RIGHT_MOTOR_SIGN != -1)
#error "电机极性只能为 +1 或 -1。"
#endif
#endif
