#include "AS5600.h"

#define AS5600_TIMEOUT_MS 5

AS5600_Status AS5600_ReadReg(AS5600_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (HAL_I2C_Mem_Read(dev->hi2c, AS5600_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, AS5600_TIMEOUT_MS) != HAL_OK) {
    return AS5600_ERR_I2C;
  }
  return AS5600_OK;
}

static AS5600_Status AS5600_ReadAngle(AS5600_t *dev, uint32_t t_us)
{
  uint8_t buf[2];
  AS5600_Status st = AS5600_ReadReg(dev, AS5600_REG_ANGLE, buf, 2);
  if (st == AS5600_OK) {
    dev->raw = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
    dev->t_us = t_us;
    dev->seq++;
  }
  return st;
}

AS5600_Status AS5600_Init(AS5600_t *dev, uint32_t t_us)
{
  dev->seq = 0;
  dev->err = 0;
  AS5600_Recover(dev);
  return AS5600_Resume(dev, t_us);
}

void AS5600_Pause(AS5600_t *dev)
{
  dev->paused = 1;
  uint32_t t0 = HAL_GetTick();
  while (dev->hi2c->State != HAL_I2C_STATE_READY && HAL_GetTick() - t0 < 2) {
  }
}

AS5600_Status AS5600_Resume(AS5600_t *dev, uint32_t t_us)
{
  AS5600_Status st = AS5600_ReadAngle(dev, t_us);
  if (st == AS5600_OK) {
    dev->paused = 0;
  }
  return st;
}

// 約 5 µs (100 kHz 的半個週期)
static void AS5600_BusDelay(void)
{
  for (volatile int i = 0; i < 40; i++) {
  }
}

void AS5600_Recover(AS5600_t *dev)
{
  dev->paused = 1;
  HAL_I2C_DeInit(dev->hi2c);

  // SCL、SDA 改成開汲極輸出 (放開 = High)
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(dev->scl_port, dev->scl_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dev->sda_port, dev->sda_pin, GPIO_PIN_SET);
  g.Pin = dev->scl_pin;
  HAL_GPIO_Init(dev->scl_port, &g);
  g.Pin = dev->sda_pin;
  HAL_GPIO_Init(dev->sda_port, &g);
  AS5600_BusDelay();
  // 從端拉住 SDA 時，送 SCL 脈衝讓它把那一個 byte 送完
  for (int i = 0; i < 9 && HAL_GPIO_ReadPin(dev->sda_port, dev->sda_pin) == GPIO_PIN_RESET; i++) {
    HAL_GPIO_WritePin(dev->scl_port, dev->scl_pin, GPIO_PIN_RESET);
    AS5600_BusDelay();
    HAL_GPIO_WritePin(dev->scl_port, dev->scl_pin, GPIO_PIN_SET);
    AS5600_BusDelay();
  }
  // STOP：SCL High 時 SDA 由 Low 變 High
  HAL_GPIO_WritePin(dev->sda_port, dev->sda_pin, GPIO_PIN_RESET);
  AS5600_BusDelay();
  HAL_GPIO_WritePin(dev->scl_port, dev->scl_pin, GPIO_PIN_SET);
  AS5600_BusDelay();
  HAL_GPIO_WritePin(dev->sda_port, dev->sda_pin, GPIO_PIN_SET);
  AS5600_BusDelay();

  // HAL_I2C_Init()：重新開時脈、腳位設回 AF，並以 SWRST 重置 I2C 周邊 (清掉 F1 的 BUSY 旗標卡住)
  HAL_I2C_Init(dev->hi2c);
}
