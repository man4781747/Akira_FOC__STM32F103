#ifndef M24C64_H
#define M24C64_H

#include <stdint.h>
#include "main.h"

// ST M24C64 (8 KB) EEPROM，I2C1，8-bit 位址 0xA0。
// 這一版只保留舊程式 (M24C64_ctrl.h) 的讀寫與齒槽補償表 exf_map 的存取，參數區塊 (EE_SAVE 等) 是之後的階段 4。
// 「page」是 64 byte 一格 (位址 = page << 6)，每次最多讀寫 32 byte，正好是晶片的頁大小，不會跨頁

#define M24C64_I2C_ADDR   0xA0
#define M24C64_TIMEOUT_MS 10
#define PAGE__ANG_SHIFT   0   // 舊的電氣零點 (float)，2026-10-14 起開機改為每次對齊，不再讀取
#define PAGE__PRE_F       1   // 齒槽補償表 exf_map[360] 的起始 page
#define PRE_F_LEN         360
#define EE_ADDR_CAN       0x0020  // CAN ID 與回報週期 (8 byte，main.c 的 CanCfg)，位址 0x0020 ~ 0x0027

extern I2C_HandleTypeDef hi2c1;

// 以 byte 位址讀寫。寫入不能跨過 32 byte 的頁邊界 (跨頁回 HAL_ERROR)
HAL_StatusTypeDef M24C64_Read(uint16_t addr, uint8_t *buf, uint16_t size);
HAL_StatusTypeDef M24C64_Write(uint16_t addr, const uint8_t *buf, uint16_t size);
HAL_StatusTypeDef M24C64_ReadData(uint8_t *dataBuffer, uint16_t page, uint16_t size);
HAL_StatusTypeDef M24C64_WriteData(uint8_t *dataBuffer, uint16_t page, uint16_t size);
HAL_StatusTypeDef ReadPreF(float PreFArray[PRE_F_LEN]);
HAL_StatusTypeDef WritePreF(const float PreFArray[PRE_F_LEN]);

#endif
