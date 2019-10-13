#ifndef _I2S_DRIVER
#define _I2S_DRIVER

#include "stm32f411xe.h"
#define I2S2 ((i2s_t *) SPI2_BASE)
#define I2S3 ((i2s_t *) SPI3_BASE)

#define I2S_STATE_FREE 0
#define I2S_STATE_TX 1
#define I2S_STATE_RX 2
#define I2S_STATE_NULL 3

typedef struct
{
  __IO uint32_t RESERVED0;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t RESERVED1;     /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RESERVED2;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t RESERVED3;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} i2s_t;

typedef struct
{
  uint32_t config;
  uint32_t i2s_std;
  uint32_t pcm_sync;
  uint32_t clock_pol;
  uint32_t data_len;
  uint32_t ch_len;
  uint8_t prescaler;
  uint32_t master_clock_enable;
  uint32_t prescaler_odd;
} i2s_init_struct_t;

typedef struct
{
  i2s_t *instance;
  i2s_init_struct_t init;
  uint32_t *p_tx_buffer;
  uint32_t *p_rx_buffer;
  volatile uint32_t tx_len;
  volatile uint32_t rx_len;
  uint32_t state;
  void (*i2s_tx)(i2s_handle_t *);
  void (*i2s_rx)(i2s_handle_t *);
} i2s_handle_t;

void i2s_init(i2s_handle_t *hi2s);

void i2s_transmit_it(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len);

void i2s_transmit_dma();

void i2s_receive_it(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len);

void i2s_receive_dma();

void i2s_irq_handler(i2s_handle_t *hi2s);

void i2s_deinit(i2s_handle_t *hi2s);
#endif /*_I2S_DRIVER */
