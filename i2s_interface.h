#ifndef _I2S_H
#define _I2S_H



#include "i2s_stm32f4xx_config.h"


#define I2S_STATE_FREE 0
#define I2S_STATE_TX 1
#define I2S_STATE_RX 2
#define I2S_STATE_NULL 3

#ifdef USE_DMA_I2S
#include "dma_driver_stm32f4xx.h"
#endif

typedef struct
{
  i2s_channel_t instance;
  uint32_t *p_tx_buffer;
  uint32_t *p_rx_buffer;
  volatile uint32_t tx_len;
  volatile uint32_t rx_len;
  uint32_t state;
  void (*i2s_tx)(void *);
  void (*i2s_rx)(void *);
} i2s_handle_t;

typedef i2s_handle_t *i2s_handle_array_t;

void i2s_init(i2s_handle_array_t *hi2s_array, const i2s_config_t *config_table);

void i2s_transmit_blocking(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len);

void i2s_receive_blocking(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len);

void i2s_transmit_it(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len);

void i2s_receive_it(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len);

#ifdef USE_DMA_I2S
void i2s_receive_dma(i2s_handle_t *hi2s, dma_handle_t *hdma, uint3_t *data, uint32_t data_len);
void i2s_transmit_dma(i2s_handle_t *hi2s, dma_handle_t *hdma, uint32_t *data, uint32_t data_len);
#else
void i2s_transmit_dma();
void i2s_receive_dma();
#endif


void i2s_irq_handler(i2s_handle_t *hi2s);
void i2s_deinit(i2s_handle_t *hi2s);

void i2s_reg_write(uint32_t i2s_register, uint16_t value);
uint16_t i2s_reg_read(uint32_t i2s_register);

 #endif /*_I2S_DRIVER */
