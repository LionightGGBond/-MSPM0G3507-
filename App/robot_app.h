#ifndef ROBOT_APP_H
#define ROBOT_APP_H
#include "../Motion/controller.h"
#include "../Bsp/encoder.h"
#include "../Bsp/tb6612.h"
#include <stdbool.h>

typedef struct {
    RobotState state;
    uint8_t line_bits; /* 当前GPIO采样，故障停车时也更新，不是故障瞬间的锁存值。 */
    bool line_fallback;
    float left_target, right_target, left_speed, right_speed, left_pwm, right_pwm;
    EncoderSample encoder;
    TB6612_InputLevels motor_inputs;
    uint32_t now_ms;
    uint32_t heartbeat_seconds, control_irq_count, control_dt_ms;
    bool heartbeat_led_on;
} RobotSnapshot;

void RobotApp_Init(bool oled_ready, bool imu_ready);
void RobotApp_Poll(void);
void RobotApp_Snapshot(RobotSnapshot *snapshot);
#endif
