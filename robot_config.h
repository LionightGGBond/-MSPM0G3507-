/**
 * @file    robot_config.h
 * @brief   全局运行模式选择与控制周期约定。
 * @details PID、速度、方向、器件地址分别在对应.c文件顶部调节，避免跨文件找参数。
 *          本文件保留会影响多个模块的编译开关，支持通过编译器-D参数覆盖。
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* 固定10ms，须与SysConfig的CONTROL_TIMER一致；改变后必须重新整定PID。 */
#define ROBOT_CONTROL_MS 10U

/* 0=当前连续循迹；1=转2秒/停2秒台架测试，具体测试参数在controller.c顶部。 */
#ifndef ROBOT_MOTOR_TEST_MODE
#define ROBOT_MOTOR_TEST_MODE 0
#endif

/* 0=无效线型保持最近差速；1=无效线型停车并锁存。 */
#ifndef ROBOT_STOP_ON_INVALID_LINE
#define ROBOT_STOP_ON_INVALID_LINE 0
#endif

/* 0=反馈缺失只统计；1=达到controller.c中的门限时停车并锁存。 */
#ifndef ROBOT_STOP_ON_ENCODER_STALL
#define ROBOT_STOP_ON_ENCODER_STALL 0
#endif

#if ROBOT_CONTROL_MS != 10U
#error "控制周期须保持10ms；更改时请同步SysConfig并重新整定PID。"
#endif
#if (ROBOT_MOTOR_TEST_MODE != 0 && ROBOT_MOTOR_TEST_MODE != 1) || \
    (ROBOT_STOP_ON_INVALID_LINE != 0 && ROBOT_STOP_ON_INVALID_LINE != 1) || \
    (ROBOT_STOP_ON_ENCODER_STALL != 0 && ROBOT_STOP_ON_ENCODER_STALL != 1)
#error "运行模式开关只能为0或1。"
#endif
#endif
