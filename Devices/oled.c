#include "oled.h"
#include "../Bsp/i2c_bus.h"
#include "../robot_config.h"
#include "font6x8.h"
#include <string.h>

static uint8_t s_frame[8][128];

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

void Oled_Clear(void)
{
    memset(s_frame, 0, sizeof(s_frame));
}

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
