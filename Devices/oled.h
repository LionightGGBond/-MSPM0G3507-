/**
 * @file    Devices/oled.h
 * @brief   OLED设备初始化、帧缓存和逐页刷新接口。
 * @details Text只改RAM缓存；FlushPage才发送I2C；OLED当前不含中文字库。
 */
#ifndef ROBOT_OLED_H
#define ROBOT_OLED_H
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief 发送SSD1306初始化命令并清除RAM帧缓存。
 * @retval true初始化命令全部成功；false存在I2C失败。
 * @note 包含上电等待，仅在初始化或停车重试阶段调用。
 */
bool Oled_Init(void);
/**
 * @brief 清空1024字节RAM帧缓存。
 * @note 不立即写屏幕，随后由主循环分时刷新。
 */
void Oled_Clear(void);
/* page=0..7，column=0..20（字符列），ASCII 6x8。 */
/**
 * @brief 把ASCII文本绘制到RAM帧缓存。
 * @param[in] page 页号0..7，每页8像素高。
 * @param[in] column 字符列0..20，每字符6像素宽。
 * @param[in] text 零结尾ASCII文本；不支持的字符显示问号。
 */
void Oled_Text(uint8_t page, uint8_t column, const char *text);
/* 一次只刷一页；由主循环分时调用，避免整屏阻塞 IMU 采样。 */
/**
 * @brief 把指定页RAM内容通过I2C发送到屏幕。
 * @param[in] page 页号0..7。
 * @retval true该页成功；false页号非法或I2C失败。
 * @note 128列分包传输，单包不超过硬件FIFO容量；仅主循环调用。
 */
bool Oled_FlushPage(uint8_t page);
#endif
