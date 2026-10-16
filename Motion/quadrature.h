#ifndef ROBOT_QUADRATURE_H
#define ROBOT_QUADRATURE_H
#include <stdint.h>
/* AB 编码为 (A<<1)|B。非法跳变返回 0；调用者记录诊断计数。 */
int8_t Quadrature_Delta(uint8_t previous_ab, uint8_t current_ab);
#endif
