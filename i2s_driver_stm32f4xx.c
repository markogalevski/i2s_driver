#include "i2s_driver_stm32f4xx.h"
#include <stdlib.h>
#ifdef USE_DMA_I2S


void i2s_transmit_dma(i2s_handle_t *hi2s, dma_handle_t *hdma, uint32_t *data, uint32_t data_len)
{
  hdma->p_memory0 = data;
  hdma->p_periph = hi2s->instance->DR;
  hdma->data_length = data_len;

  if (hi2s->instance == I2S2)
  {
    hdma->stream = DMA1_Stream3;
    hdma->controller = DMA1;
  }
  else if (hi2s->instance == I2S3)
  {
    hdma->stream = DMA1_Stream0;
    hdma->controller = DMA1;
  }
  hdma->init->transfer_dir = DMA_DIR_M2P;
  hdma->init->channel_select = DMA_CH_0;
  hdma->init->peripheral_flow_ctrl = FLOW_DMA;
  hdma->init->circ_mode_en = EN_RESET;
  hdma->init->double_buffer_en = EN_RESET;
  hdma->init->peripheral_data_size = PSIZE_HWORD;
  hdma->init->peripheral_increment_mode = PAR_FIXED;
  hdma->init->memory_data_size = MSIZE_HWORD;
  hdma->init->memory_increment_mode = MAR_INC;
  hdma->init->peripheral_burst_transfer = DMA_BURST_SINGLE;
  hdma->init->memory_burst_transfer = DMA_BURST_SINGLE;

  hdma->fifo_config->direct_mode_disable = FIFO_DIRECT_ENABLE;

  dma_transfer(hdma);
}

void i2s_receive_dma(i2s_handle_t *hi2s, dma_handle_t *hdma, uint3_t *data, uint32_t data_len)
{
    hdma->p_memory0 = data;
    hdma->p_periph = hi2s->instance->DR;
    hdma->data_length = data_len;

    if (hi2s->instance == I2S2)
    {
      hdma->stream = DMA1_Stream4;
      hdma->controller = DMA1;
    }
    else if (hi2s->instance == I2S3)
    {
      hdma->stream = DMA1_Stream7;
      hdma->controller = DMA1;
    }
    hdma->init->transfer_dir = DMA_DIR_P2M;
    hdma->init->channel_select = DMA_CH_0;
    hdma->init->peripheral_flow_ctrl = FLOW_DMA;
    hdma->init->circ_mode_en = EN_RESET;
    hdma->init->double_buffer_en = EN_RESET;
    hdma->init->peripheral_data_size = PSIZE_HWORD;
    hdma->init->peripheral_increment_mode = PAR_FIXED;
    hdma->init->memory_data_size = MSIZE_HWORD;
    hdma->init->memory_increment_mode = MAR_INC;
    hdma->init->peripheral_burst_transfer = DMA_BURST_SINGLE;
    hdma->init->memory_burst_transfer = DMA_BURST_SINGLE;

    hdma->fifo_config->direct_mode_disable = FIFO_DIRECT_ENABLE;

    dma_transfer(hdma);
}
#endif

static void i2s_gpio_init(i2s_t *p_i2s);
static void i2s_tx_phillips(void *v_self);
static void i2s_rx_phillips(void *v_self);
static void (*i2s_tx_msb)(void *) = i2s_tx_phillips;
static void (*i2s_rx_msb)(void *) = i2s_rx_phillips;
static void i2s_tx_lsb(void *v_self);
static void i2s_rx_lsb(void *v_self);
static void (*i2s_tx_pcm)(void *) = i2s_tx_phillips;
static void (*i2s_rx_pcm)(void *) = i2s_rx_phillips;


void i2s_init(i2s_handle_t *hi2s)
{
  // Init function takes the broader handle pointer, and first initialises
  // all of the prerequisites on an STM32F4 chip: RCC and GPIO_AF setup.
  // It ensures the I2S device is off before setting all of the registers to
  // their requested values as per the init structure.
  // Single writes to registers via bit overlayed config_reg values are used,
  // just to be sure.
  // The i2s format determines to which implementation the low level transfer
  // function pointer points.
  // The device is then enabled.

    i2s_t *p_i2s =  (i2s_t *) hi2s->instance;
    i2s_init_t *p_init = (i2s_init_t *) &hi2s->init;

    //RCC enables for the required GPIO ports
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk | RCC_AHB1ENR_GPIOBEN_Msk;

    i2s_gpio_init(p_i2s); // GPIO Alternate Function setup


    uint32_t config_reg = 0; //Selection of which SPI/I2S RCC to enable
    if (p_i2s == I2S2)
    {
        config_reg = RCC_APB1ENR_SPI2EN_Msk;
    }
    else if (p_i2s == I2S3)
    {
        config_reg = RCC_APB1ENR_SPI3EN_Msk;
    }
    RCC->APB1ENR |= config_reg;


    p_i2s->I2SCFGR &= ~(SPI_I2SCFGR_I2SE_Msk); //disable I2S
    p_i2s->I2SCFGR |= (SPI_I2SCFGR_I2SMOD_Msk); //ensure i2s mode is on

    config_reg = 0;
    config_reg |= p_init->config << SPI_I2SCFGR_I2SCFG_Pos
           | p_init->pcm_sync << SPI_I2SCFGR_PCMSYNC_Pos
           | p_init->i2s_std << SPI_I2SCFGR_I2SSTD_Pos
           | p_init->clock_pol << SPI_I2SCFGR_CKPOL_Pos
           | p_init->data_len << SPI_I2SCFGR_DATLEN_Pos
           | p_init->ch_len << SPI_I2SCFGR_CHLEN_Pos;
    p_i2s->I2SCFGR = config_reg;

    config_reg = 0;
    config_reg |= p_init->master_clock_enable << SPI_I2SPR_MCKOE_Pos
           | p_init->prescaler_odd << SPI_I2SPR_ODD_Pos
           | p_init->prescaler;
    p_i2s->I2SPR = config_reg;

    if (p_init->i2s_std == 0)
    {
      hi2s->i2s_tx = i2s_tx_phillips;
      hi2s->i2s_rx = i2s_rx_phillips;
    }
    else if (p_init->i2s_std == 1)
    {
      hi2s->i2s_tx = i2s_tx_msb;
      hi2s->i2s_rx = i2s_rx_msb;
    }
    else if (p_init->i2s_std == 2)
    {
      hi2s->i2s_tx = &i2s_tx_lsb;
      hi2s->i2s_rx = &i2s_rx_lsb;
    }
    else if (p_init->i2s_std == 3)
    {
      hi2s->i2s_tx = &i2s_tx_pcm;
      hi2s->i2s_rx = &i2s_rx_pcm;
    }

    p_i2s->I2SCFGR |= SPI_I2SCFGR_I2SE_Msk; //enable peripheral
}

static void i2s_gpio_init(i2s_t *p_i2s)
{
  // Makes writes to the appropriate GPIOAF registers to enable I2S2 or I2S3,
  // as per their implementation in the stm32f411xe manual.
  if (p_i2s == I2S2)
  {
    //GPIO pins PB12, PB13, PB15 need  to be AF5 (0b0101 == 0x05) to work
    //  in SPI/I2S Mode
    // PB12: I2S2_WS
    // PB13: I2S2_CK
    // PB15: I2S2_SD

    GPIOB->AFR[1] |= (0x05U) << (GPIO_AFRH_AFSEL12_Pos)
                    | (0x05U) << (GPIO_AFRH_AFSEL13_Pos)
                    | (0x05U) << (GPIO_AFRH_AFSEL15_Pos);
  }
    // GPIO pins PA15, PB3, and PB5 need to be AF6 (0b0110 == 0x06) to work
    // PA15: I2S3_WS
    // PB3: I2S3_CK
    // PB5: I2S3_SD
  else if (p_i2s == I2S3)
  {
    GPIOA->AFR[1] |= (0x06U) << (GPIO_AFRH_AFSEL15_Pos);
    GPIOB->AFR[0] |= (0x06U) << (GPIO_AFRL_AFSEL3_Pos)
                    | (0x06U) << (GPIO_AFRL_AFSEL5_Pos);
  }
}

void i2s_transmit_blocking(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len)
{
  if (hi2s->state == I2S_STATE_FREE)
  {
    hi2s->p_tx_buffer = data;
    hi2s->tx_len = data_len;
    hi2s->p_rx_buffer = NULL;
    hi2s->rx_len = 0;
    hi2s->state = I2S_STATE_TX;
    while(hi2s->tx_len != 0)
    {
      while(!(hi2s->instance->SR & SPI_SR_TXE_Msk));
      (*hi2s->i2s_tx)(hi2s);
    }
    hi2s->state = I2S_STATE_FREE;
  }
}

void i2s_receive_blocking(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len)
{
  if (hi2s->state == I2S_STATE_FREE)
  {
    hi2s->p_rx_buffer = data;
    hi2s->rx_len = data_len;
    hi2s->p_tx_buffer = NULL;
    hi2s->tx_len = 0;
    hi2s->state = I2S_STATE_RX;
    while(hi2s->rx_len != 0)
    {
      while(!(hi2s->instance->SR & SPI_SR_RXNE_Msk));
      (*hi2s->i2s_rx)(hi2s);
    }
    hi2s->state = I2S_STATE_FREE;
  }
}



void i2s_transmit_it(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len)
{
  // Begins an interrupt based i2s transmission as master.
  // the tx data pointer and length are copied, while the rx ones are nullified.
  // TX/RX states are SET/RESET, and provided valid data is ready, the transmit
  // buffer empty interrupt is enabled. The ISR handles the rest.
  // NOTE: the TXEIE enables the I2S device to request an interrupt, you must
  // activate the I2S NVIC IRQn to make the MCU responsd to the interrupt req.
  // IMPORTANT: WRAP this function in CRITICAL()

  hi2s->p_tx_buffer = data;
  hi2s->tx_len = data_len;
  hi2s->p_rx_buffer = NULL;
  hi2s->rx_len = 0;
  hi2s->state = I2S_STATE_TX;
  if (hi2s->tx_len != 0 && hi2s->p_tx_buffer != NULL)
  {
    hi2s->instance->CR2 |= SPI_CR2_TXEIE_Msk;
  }
  else
  {
    hi2s->state = I2S_STATE_FREE;
  }
}

void i2s_receive_it(i2s_handle_t *hi2s, uint32_t *data, uint32_t data_len)
{
  // Begins an interrupt based i2s reception as master.
  // the tx data pointer and length are copied, while the rx ones are nullified.
  // TX/RX states are SET/RESET, and provided valid data is ready, the transmit
  // buffer empty interrupt is enabled. The ISR handles the rest.
  // NOTE: the RXNEIE enables the I2S device to request an interrupt, you must
  // activate the I2S NVIC IRQn to make the MCU responsd to the interrupt req.
  // Make sure to wrap this in  CRITICAL()
  hi2s->p_rx_buffer = data;
  hi2s->rx_len = data_len;
  hi2s->p_tx_buffer = NULL;
  hi2s->tx_len = 0;
  hi2s->state = I2S_STATE_RX;

  if (hi2s->rx_len != 0 && hi2s->p_rx_buffer != NULL)
  {
    hi2s->instance->CR2 |= SPI_CR2_RXNEIE_Msk;
  }
  else
  {
    hi2s->state = I2S_STATE_FREE;
  }
}

void i2s_irq_handler(i2s_handle_t *hi2s)
{
  // this IRQ handler should be called within the appropriate SPIx_IRQHandler.
  // Naturally, ensure the hi2s is a global variable.
  uint32_t cmp1 = 0;
  uint32_t cmp2 = 0;
  //check txe for high and also txeie
  cmp1 = hi2s->instance->SR & SPI_SR_TXE_Msk;
  cmp2 = hi2s->instance->CR2 & SPI_CR2_TXEIE_Msk;
  if (cmp1 != 0 && cmp2 != 0)
  {
    if (hi2s->tx_len == 0)
    {
      hi2s->instance->CR2 &= ~(SPI_CR2_TXEIE_Msk);
      hi2s->state = I2S_STATE_FREE;
    }
    else if (hi2s->i2s_tx != 0U)
    {
      hi2s->instance->CR2 &= ~(SPI_CR2_TXEIE_Msk);
      (*(hi2s->i2s_tx))(hi2s);
      hi2s->instance->CR2 |= SPI_CR2_TXEIE_Msk;
    }

  }
  // check for reception rxne high and rxneie
  cmp1 = hi2s->instance->SR & SPI_SR_RXNE_Msk;
  cmp2 = hi2s->instance->CR2 & SPI_CR2_RXNEIE_Msk;
  if (cmp1 != 0 && cmp2 != 0)
  {
    if (hi2s->rx_len == 0)
    {
      hi2s->instance->CR2 &= ~(SPI_CR2_RXNEIE_Msk);
      hi2s->state = I2S_STATE_FREE;
    }
    else if (hi2s->i2s_rx != 0U)
    {
      hi2s->instance->CR2 &= ~(SPI_CR2_RXNEIE_Msk);
      (*(hi2s->i2s_rx))(hi2s);
      hi2s->instance->CR2 |= SPI_CR2_RXNEIE_Msk;
    }
  }

}

static void i2s_tx_phillips(void *v_self)
{
  // Places data in the DR register of the I2S device per the specifications
  // in chapter 20 of the reference manual.
  i2s_handle_t * self = (i2s_handle_t *) v_self;
  if (self->init.data_len == 0)
  {
    //16bit. Channel length doesn't affect anything (see ref manual 20.4.3)
    self->instance->DR = (uint16_t)( 0x0000FFFFUL & (*(self->p_tx_buffer)) );
    self->p_tx_buffer++;
    self->tx_len--;
  }
  else if (self->init.data_len == 1) //24bit data length
  {
    self->instance->DR = (uint16_t)( (0x00FFFF00UL & (*(self->p_tx_buffer))) >> 8U);
    while(self->instance->SR & SPI_SR_TXE_Msk == 0);
    self->instance->DR = (uint16_t)( (0x000000FFUL & (*(self->p_tx_buffer))) << 8U);
    self->p_tx_buffer++;
    self->tx_len--;
  }
  else if (self->init.data_len == 2) //32bit data Length
  {
    self->instance->DR = (uint16_t) ((0xFFFF0000UL & (*(self->p_tx_buffer))) >> 16U);
    while(self->instance->SR & SPI_SR_TXE_Msk == 0);
    self->instance->DR = (uint16_t) (0x0000FFFFUL & (*(self->p_tx_buffer)));
    self->p_tx_buffer++;
    self->tx_len--;

  }
}

static void i2s_rx_phillips(void *v_self)
{
  i2s_handle_t *self = (i2s_handle_t *) v_self;
    // Places data in the DR register of the I2S device per the specifications
    // in chapter 20 of the reference manual.
  if (self->init.data_len == 0)
  {
    //16 bit data length.
    *(self->p_rx_buffer) = self->instance->DR;
    self->p_rx_buffer++;
    self->rx_len--;
  }
  else if (self->init.data_len == 1)
  {
    //24 bit data length
    *(self->p_rx_buffer) = (self->instance->DR) << 8U;
    while((self->instance->SR & SPI_SR_RXNE_Msk) == 0);
    *(self->p_rx_buffer) |= (self->instance->DR) >> 8U;
    self->p_rx_buffer++;
    self->rx_len--;
  }
  else if (self->init.data_len == 2)
  {
    //32 bit data length.
    *(self->p_rx_buffer) = (self->instance->DR) << 16U;
    while((self->instance->SR & SPI_SR_RXNE_Msk) == 0);
    *(self->p_rx_buffer) |= self->instance->DR;
    self->p_rx_buffer++;
    self->rx_len--;
  }
}

static void i2s_tx_lsb(void *v_self)
{
	i2s_handle_t * self = (i2s_handle_t *) v_self;
    // Places data in the DR register of the I2S device per the specifications
    // in chapter 20 of the reference manual.
    if (self->init.data_len == 0)
    {
      //16bit. Channel length doesn't affect anything (see ref manual 20.4.3)
      self->instance->DR = (uint16_t)( 0x0000FFFFUL & (*(self->p_tx_buffer)) );
      self->p_tx_buffer++;
      self->tx_len--;
    }

    else if (self->init.data_len >= 1) //24bit data length
    {
      self->instance->DR = (uint16_t)( 0x00FF0000UL & (*(self->p_tx_buffer)) >> 16U);
      while(self->instance->SR & SPI_SR_TXE_Msk == 0);
      self->instance->DR = (uint16_t)( 0x0000FFFFUL & (*(self->p_tx_buffer)));
      self->p_tx_buffer++;
      self->tx_len--;
    }
    else if (self->init.data_len == 2) //32bit data Length
    {
      self->instance->DR = (uint16_t) (0xFFFF0000UL & (*(self->p_tx_buffer)) >> 16U);
      while(self->instance->SR & SPI_SR_TXE_Msk == 0);
      self->instance->DR = (uint16_t) (0x0000FFFFUL & (*(self->p_tx_buffer)));
      self->p_tx_buffer++;
      self->tx_len--;

    }
}

static void i2s_rx_lsb(void *v_self)
{
	i2s_handle_t *self = (i2s_handle_t *) v_self;
    // Places data in the DR register of the I2S device per the specifications
    // in chapter 20 of the reference manual.
  if (self->init.data_len == 0)
  {
    //16 bit data length - RNXE only triggered when non-null/extended data comes in
    *(self->p_rx_buffer) = self->instance->DR;
    self->p_rx_buffer++;
    self->rx_len--;
  }
  else if (self->init.data_len >= 1)
  {
    //24 bit and 32 bit data length can be processed identically
    *(self->p_rx_buffer) = (self->instance->DR) << 16U;
    while(self->instance->SR & SPI_SR_RXNE_Msk == 0);
    *(self->p_rx_buffer) |= self->instance->DR;
    self->p_rx_buffer++;
    self->rx_len--;
  }
}

void i2s_deinit(i2s_handle_t *hi2s)
{
  uint32_t mode = (hi2s->instance->I2SCFGR & (SPI_I2SCFGR_I2SCFG_Msk)) >> SPI_I2SCFGR_I2SCFG_Pos;
  uint32_t counter = 0;
  if (mode == 0x02)
  {
    //mode == master transmit. Stop sequence as per 20.4.5 in Reference Manual.
   while( ((hi2s->instance->SR & SPI_SR_TXE_Msk) == 0)
          || ((hi2s->instance->SR & SPI_SR_BSY_Msk) != 0 ) );
   hi2s->instance->I2SCFGR &= ~(SPI_I2SCFGR_I2SE_Msk); //disable i2s.
  }
  else if (mode == 0x03)
  {
    //mode == master receive. Stop sequence as per 20.4.5 in Reference Manual.
    uint32_t clock_B_pin;
    if (hi2s->instance == I2S2)
    {
    clock_B_pin = 13;
    }
    else if (hi2s->instance == I2S3)
    {
    clock_B_pin = 3;
    }
    if (hi2s->init.ch_len == 1 && hi2s->init.data_len == 0)
    {
      if (hi2s->init.i2s_std == 2)
      {
        //stop with lsb 16 bit 32 extended
        while(hi2s->rx_len > 1 || hi2s->instance->SR & SPI_SR_RXNE_Msk == 0);
        while(counter < 17) //wait for 17 cycles
        {
          while(GPIOB->ODR & (0x01 << clock_B_pin) != 0);
          counter++;
        }
       hi2s->instance->I2SCFGR &= ~(SPI_I2SCFGR_I2SE_Msk);
      }
      else
      {
        //stop with non-lsb 16 bit 32 extended
        while(hi2s->rx_len != 0 || hi2s->instance->SR & SPI_SR_RXNE_Msk == 0); //wait for LAST rxne
        while(GPIOB->ODR & (0x01 << clock_B_pin) != 0); //wait for one cycle
        hi2s->instance->I2SCFGR &= ~(SPI_I2SCFGR_I2SE_Msk);
      }
    }
    else
    {
      while(hi2s->rx_len > 1 || hi2s->instance->SR & SPI_SR_RXNE_Msk == 0);
      while(GPIOB->ODR & (0x01 << clock_B_pin) != 0); //wait for one cycle
      hi2s->instance->I2SCFGR &= ~(SPI_I2SCFGR_I2SE_Msk);
    }
  }
}
#ifndef USE_DMA_I2S

void i2s_transmit_dma()
{
while(1);
}

void i2s_receive_dma()
{
while(1);
}

#endif
