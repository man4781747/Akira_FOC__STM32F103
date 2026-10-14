#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include "main.h"

// ams AS5600 12-bit 磁編碼器，I2C 7-bit 位址 0x36。
// 開機時以阻塞方式讀 STATUS / CONF 等暫存器，之後由 AS5600_Kick() 在背景以 DMA 連續讀 ANGLE。
// ANGLE (0x0E/0x0F) 是特殊暫存器：位址指標指到 0x0E 之後，讀完兩個 byte 不會往後遞增，
// 所以重複讀取只要送「讀」，不用每次重新寫入暫存器位址 (規格書 "Automatic increment of the address pointer")。

#define AS5600_I2C_ADDR     (0x36 << 1)
#define AS5600_CPR          4096

#define AS5600_REG_ZPOS     0x01
#define AS5600_REG_MPOS     0x03
#define AS5600_REG_CONF     0x07
#define AS5600_REG_STATUS   0x0B
#define AS5600_REG_RAWANGLE 0x0C
#define AS5600_REG_ANGLE    0x0E
#define AS5600_REG_AGC      0x1A

#define AS5600_STATUS_MH    0x08  // 磁場太強
#define AS5600_STATUS_ML    0x10  // 磁場太弱
#define AS5600_STATUS_MD    0x20  // 偵測到磁鐵

typedef enum {
  AS5600_OK = 0,
  AS5600_ERR_I2C,
  AS5600_ERR_BUSY,
} AS5600_Status;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  GPIO_TypeDef *scl_port;      // 匯流排卡住時以 GPIO 送 SCL 脈衝用
  uint16_t scl_pin;
  GPIO_TypeDef *sda_port;
  uint16_t sda_pin;
  uint8_t rx[2];               // DMA 接收緩衝區
  volatile uint16_t raw;       // 最近一次讀到的 ANGLE (0 ~ 4095)
  volatile uint32_t t_us;      // 這一筆讀完的時間 (micros() 低 32 bit)
  volatile uint32_t seq;       // 成功讀到的筆數，每筆加 1
  volatile uint32_t err;       // I2C 錯誤次數
  volatile uint8_t paused;     // 1 = AS5600_Kick() 不啟動新的讀取 (阻塞讀寫或重新初始化期間)
} AS5600_t;

// 阻塞讀暫存器 (逾時 5 ms)。會移動 AS5600 的位址指標，呼叫前要先 AS5600_Pause()，之後以 AS5600_Resume() 指回 ANGLE
AS5600_Status AS5600_ReadReg(AS5600_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
// 清除匯流排 (AS5600_Recover)，讀一次 ANGLE (同時把位址指標指到 0x0E)，成功時更新 raw / t_us / seq。
// 呼叫前要先填好 hi2c 與 SCL / SDA 腳位
AS5600_Status AS5600_Init(AS5600_t *dev, uint32_t t_us);
// 停止背景讀取，等進行中的那一筆結束 (最多 2 ms)
void AS5600_Pause(AS5600_t *dev);
// 把位址指標指回 ANGLE，恢復背景讀取
AS5600_Status AS5600_Resume(AS5600_t *dev, uint32_t t_us);
// 匯流排卡住時重新初始化：I2C 周邊重置、以 GPIO 送最多 9 個 SCL 脈衝讓從端放開 SDA、送 STOP。
// 之後要 AS5600_Resume()。MCU 在讀取途中被重置 (例如燒錄) 時，AS5600 可能一直拉住 SDA
void AS5600_Recover(AS5600_t *dev);

// 在 FOC 中斷 (ADC 中斷) 內呼叫：沒有讀取在進行時啟動下一筆 DMA 讀取
static inline void AS5600_Kick(AS5600_t *dev)
{
  I2C_HandleTypeDef *hi2c = dev->hi2c;
  if (dev->paused || hi2c->State != HAL_I2C_STATE_READY) {
    return;
  }
  // HAL 在 BUSY 時會原地等到逾時 (25 ms)，前一筆的 STOP 還沒送完就先跳過，下一次再讀
  if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY)) {
    return;
  }
  HAL_I2C_Master_Receive_DMA(hi2c, AS5600_I2C_ADDR, dev->rx, 2);
}

// HAL_I2C_MasterRxCpltCallback() 內呼叫
static inline void AS5600_RxDone(AS5600_t *dev, uint32_t t_us)
{
  dev->raw = (uint16_t)(((dev->rx[0] & 0x0F) << 8) | dev->rx[1]);
  dev->t_us = t_us;
  dev->seq++;
}

// HAL_I2C_ErrorCallback() 內呼叫
static inline void AS5600_RxError(AS5600_t *dev)
{
  dev->err++;
}

#endif
