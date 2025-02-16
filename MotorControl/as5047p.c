#include "as5047p.h"

extern SPI_HandleTypeDef hspi1;

static uint16_t transmit_receive_16bit(uint16_t tx_data)
{
  AP_CS(0);
  uint16_t rx_data;
  if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx_data, (uint8_t *)&rx_data, 1, 100) != HAL_OK)
  {
    rx_data = 0;
  }
  AP_CS(1);
  return rx_data;
}

unsigned int even_check(unsigned int v)
{
  if (v == 0)
    return 0;
  v ^= v >> 8;
  v ^= v >> 4;
  v ^= v >> 2;
  v ^= v >> 1;
  return v & 1;
}

/* 带校验的读取 */
unsigned int read_as5047p_checked(unsigned int cmd)
{

  (void)transmit_receive_16bit(cmd);

  uint16_t data = transmit_receive_16bit(cmd);
  /* 偶校验 */
  if (data & (1 << 14))
  {
    (void)transmit_receive_16bit(READ_ERRFL);
    unsigned int spi_error = transmit_receive_16bit(READ_ERRFL);
    spi_error = spi_error & 0x0003;
    return 0;
  }
  else
  {
    if ((data >> 15) == even_check(data & 0x7FFF))
    {
      return (data & 0x3FFF);
    }
    else
    {
      return 0;
    }
  }
}

unsigned int read_as5047p_error(void)
{
  (void)transmit_receive_16bit(READ_ERRFL);
  // 获取错误原因
  unsigned int spi_error = transmit_receive_16bit(READ_NOP);
  spi_error = spi_error & 0x0003;

  return spi_error;
}

unsigned int read_as5047p_uncheck(unsigned int cmd)
{
  static unsigned int data = 0;
  cmd |= 0x4000;
  if (even_check(cmd) == 1)
    cmd |= 0x8000;
  (void)transmit_receive_16bit(cmd);
  data = transmit_receive_16bit(cmd | 0x4000);
  data &= 0x3FFF;
  return data;
}
