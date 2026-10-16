#ifndef ROBOT_OLED_H
#define ROBOT_OLED_H
#include <stdbool.h>
#include <stdint.h>
bool Oled_Init(void);
void Oled_Clear(void);
/* page=0..7，column=0..20（字符列），ASCII 6x8。 */
void Oled_Text(uint8_t page, uint8_t column, const char *text);
/* 一次只刷一页；由主循环分时调用，避免整屏阻塞 IMU 采样。 */
bool Oled_FlushPage(uint8_t page);
#endif
