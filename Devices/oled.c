/**
 * @file    Devices/oled.c
 * @brief   SSD1306 128×64：6×8 ASCII绘制、帧缓存与逐页I2C刷新。
 * @details 顶部设置7位I2C地址；界面布局由App/robot_app.c决定，刷新节拍在empty.c。
 */
#include "oled.h"
#include "../Bsp/i2c_bus.h"
#include "font6x8.h"
#include <string.h>

/* ======================== 本模块参数（修改后重新编译） ======================== */
#define ROBOT_OLED_ADDRESS        0x3CU /* SSD1306的7位I2C地址，不能填左移后的0x78。 */

static uint8_t s_frame[8][128];

/**
 * @brief 发送SSD1306初始化命令并清除RAM帧缓存。
 * @retval true初始化命令全部成功；false存在I2C失败。
 * @note 包含上电等待，仅在初始化或停车重试阶段调用。
 */
bool Oled_Init(void)
{
    static const uint8_t commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, /* 页寻址模式，配合 B0..B7 定位命令 */
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF
    };
    Board_DelayMs(100U);
    for (unsigned int i = 0U; i < sizeof(commands); ++i) {
        uint8_t packet[2] = {0x00U, commands[i]};
        if (I2c_Write(OLED_I2C_INST, ROBOT_OLED_ADDRESS, packet, 2U) != I2C_OK) return false;
    }
    Oled_Clear();
    return true;
}

/**
 * @brief 清空1024字节RAM帧缓存。
 * @note 不立即写屏幕，随后由主循环分时刷新。
 */
void Oled_Clear(void)
{
    memset(s_frame, 0, sizeof(s_frame));
}

/**
 * @brief 把ASCII文本绘制到RAM帧缓存。
 * @param[in] page 页号0..7，每页8像素高。
 * @param[in] column 字符列0..20，每字符6像素宽。
 * @param[in] text 零结尾ASCII文本；不支持的字符显示问号。
 */
void Oled_Text(uint8_t page, uint8_t column, const char *text)
{
    if (page >= 8U || column >= 21U || text == 0) return;
    uint16_t x = (uint16_t)column * 6U;
    while (*text != '\0' && x + 6U <= 128U) {
        uint8_t ch = (uint8_t)*text++;
        if (ch < 32U || ch > 126U) ch = (uint8_t)'?';
        for (unsigned int i = 0U; i < 6U; ++i) s_frame[page][x++] = FONT_6X8[ch - 32U][i];
    }
}

/**
 * @brief 把指定页RAM内容通过I2C发送到屏幕。
 * @param[in] page 页号0..7。
 * @retval true该页成功；false页号非法或I2C失败。
 * @note 128列分包传输，单包不超过硬件FIFO容量；仅主循环调用。
 */
bool Oled_FlushPage(uint8_t page)
{
    if (page >= 8U) return false;
    uint8_t position[4] = {0x00U, (uint8_t)(0xB0U | page), 0x00U, 0x10U};
    if (I2c_Write(OLED_I2C_INST, ROBOT_OLED_ADDRESS, position, 4U) != I2C_OK) return false;
    for (uint8_t x = 0U; x < 128U;) {
        uint8_t count = (uint8_t)(128U - x);
        if (count > 7U) count = 7U;
        uint8_t packet[8] = {0x40U};
        memcpy(&packet[1], &s_frame[page][x], count);
        if (I2c_Write(OLED_I2C_INST, ROBOT_OLED_ADDRESS, packet, (uint8_t)(count + 1U)) != I2C_OK) return false;
        x = (uint8_t)(x + count);
    }
    return true;
}
