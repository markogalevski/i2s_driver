#include <i2s_interface.h>
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

typedef struct{
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

static volatile uint16_t *const I2S_CR2[NUM_I2S] =
{(uint16_t *) (I2S2_BASE  + 0x04UL), (uint16_t *) (I2S3_BASE + 0x04UL)};

static volatile uint16_t *const I2S_SR[NUM_I2S] =
{(uint16_t *) (I2S2_BASE + 0x08UL), (uint16_t *) (I2S3_BASE + 0x08UL)};

static volatile uint16_t *const I2S_DR[NUM_I2S] =
{(uint16_t *) (I2S2_BASE + 0x0CUL), (uint16_t *) (I2S3_BASE + 0x0CUL)};

static volatile uint16_t *const I2S_CFGR[NUM_I2S] =
{(uint16_t *) (I2S2_BASE + 0x1CUL), (uint16_t *) (I2S3_BASE + 0x1CUL)};

static volatile uint16_t * I2S_PR[NUM_I2S] =
{(uint16_t *) (I2S2_BASE + 0x20UL), (uint16_t *)(I2S3_BASE + 0x20UL)};

static void i2s_gpio_init(i2s_handle_t *h_i2s);
static void i2s_tx_phillips(void *v_self);
static void i2s_rx_phillips(void *v_self);
static void (*i2s_tx_msb)(void *) = i2s_tx_phillips;
static void (*i2s_rx_msb)(void *) = i2s_rx_phillips;
static void i2s_tx_lsb(void *v_self);
static void i2s_rx_lsb(void *v_self);
static void (*i2s_tx_pcm)(void *) = i2s_tx_phillips;
static void (*i2s_rx_pcm)(void *) = i2s_rx_phillips;


void i2s_init(i2s_handle_array_t *hi2s_array, const i2s_config_t *config_table)
{
  // PRE-CONDITION: relevant GPIO clocks enabled
  // PRE-CONDITION: relevant I2S clock enabled
  // PRE-CONDITION: configuration structure has been obtained

  // It ensures the I2S device is off before setting all of the registers to
  // their requested values as per the init structure.
  // Single writes to registers via bit overlayed config_reg values are used,
  // just to be sure.
  // The i2s format determines to which implementation the low level transfer
  // function pointer points.
  // The device is then enabled.
/*
    //RCC enables for the required GPIO ports
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk | RCC_AHB1ENR_GPIOBEN_Msk;

    i2s_gpio_init(hi2s); // GPIO Alternate Function setup


    uint32_t config_reg = 0; //Selection of which SPI/I2S RCC to enable
    if (hi2s->instance == I2S2)
    {
        config_reg = RCC_APB1ENR_SPI2EN_Msk;
    }
    else if (hi2s->instance == I2S3)
    {
        config_reg = RCC_APB1ENR_SPI3EN_Msk;
    }
    RCC->APB1ENR |= config_reg;

    //^^ Above should be preconditions
*/
	uint32_t config_reg = 0;
    for (uint32_t i2s_channel = 0; i2s_channel < NUM_I2S; i2s_channel++)
    {
      *I2S_CFGR[i2s_channel] &= ~(SPI_I2SCFGR_I2SE_Msk); //disable I2S
      *I2S_CFGR[i2s_channel] |= (SPI_I2SCFGR_I2SMOD_Msk); //ensure i2s mode is on

      config_reg = 0;
      config_reg |= config_table[i2s_channel].mode << SPI_I2SCFGR_I2SCFG_Pos
           | config_table[i2s_channel].pcm_sync << SPI_I2SCFGR_PCMSYNC_Pos
           | config_table[i2s_channel].i2s_std << SPI_I2SCFGR_I2SSTD_Pos
           | config_table[i2s_channel].clock_pol << SPI_I2SCFGR_CKPOL_Pos
           | config_table[i2s_channel].data_len << SPI_I2SCFGR_DATLEN_Pos
           | config_table[i2s_channel].ch_len << SPI_I2SCFGR_CHLEN_Pos;
      *I2S_CFGR[i2s_channel] = config_reg;

      config_reg = 0;
      config_reg |= config_table[i2s_channel].master_clock_enable << SPI_I2SPR_MCKOE_Pos
           | config_table[i2s_channel].prescaler_odd << SPI_I2SPR_ODD_Pos
           | config_table[i2s_channel].prescaler;
      *I2S_PR[i2s_channel] = config_reg;

      if (config_table[i2s_channel].i2s_std == PHILLIPS_STD)
      {
        hi2s_array[i2s_channel]->i2s_tx = i2s_tx_phillips;
        hi2s_array[i2s_channel]->i2s_rx = i2s_rx_phillips;
      }
      else if (config_table[i2s_channel].i2s_std == MSB_STD)
      {
    	hi2s_array[i2s_channel]->i2s_tx = i2s_tx_msb;
    	hi2s_array[i2s_channel]->i2s_rx = i2s_rx_msb;
      }
      else if (config_table[i2s_channel].i2s_std == LSB_STD)
      {
    	hi2s_array[i2s_channel]->i2s_tx = &i2s_tx_lsb;
    	hi2s_array[i2s_channel]->i2s_rx = &i2s_rx_lsb;
      }
      else if (config_table[i2s_channel].i2s_std == PCM_STD)
      {
    	hi2s_array[i2s_channel]->i2s_tx = i2s_tx_pcm;
    	hi2s_array[i2s_channel]->i2s_rx = i2s_rx_pcm;
      }

      *I2S_CFGR[i2s_channel] |= config_table[i2s_channel].enable << SPI_I2SCFGR_I2SE_Pos; //enable peripheral (if relevant)
    }
}

static void i2s_gpio_init(i2s_handle_t *h_i2s)
{
  // Makes writes to the appropriate GPIOAF registers to enable I2S2 or I2S3,
  // as per their implementation in the stm32f411xe manual.
  if (h_i2s->instance == I2S2)
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
  else if (h_i2s->instance == I2S3)
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
      while(!(*I2S_SR[hi2s->instance] & SPI_SR_TXE_Msk));
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
      while(!(*I2S_SR[hi2s->instance] & SPI_SR_RXNE_Msk));
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
    *I2S_CR2[hi2s->instance] |= SPI_CR2_TXEIE_Msk;
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
    *I2S_CR2[hi2s->instance] |= SPI_CR2_RXNEIE_Msk;
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
  cmp1 = *I2S_SR[hi2s->instance] & SPI_SR_TXE_Msk;
  cmp2 = *I2S_CR2[hi2s->instance] & SPI_CR2_TXEIE_Msk;
  if (cmp1 != 0 && cmp2 != 0)
  {
    if (hi2s->tx_len == 0)
    {
      *I2S_CR2[hi2s->instance] &= ~(SPI_CR2_TXEIE_Msk);
      hi2s->state = I2S_STATE_FREE;
    }
    else if (hi2s->i2s_tx != 0U)
    {
      *I2S_CR2[hi2s->instance] &= ~(SPI_CR2_TXEIE_Msk);
      (*(hi2s->i2s_tx))(hi2s);
      *I2S_CR2[hi2s->instance] |= SPI_CR2_TXEIE_Msk;
    }

  }
  // check for reception rxne high and rxneie
  cmp1 = *I2S_SR[hi2s->instance] & SPI_SR_RXNE_Msk;
  cmp2 = *I2S_CR2[hi2s->instance] & SPI_CR2_RXNEIE_Msk;
  if (cmp1 != 0 && cmp2 != 0)
  {
    if (hi2s->rx_len == 0)
    {
      *I2S_CR2[hi2s->instance] &= ~(SPI_CR2_RXNEIE_Msk);
      hi2s->state = I2S_STATE_FREE;
    }
    else if (hi2s->i2s_rx != 0U)
    {
      *I2S_CR2[hi2s->instance] &= ~(SPI_CR2_RXNEIE_Msk);
      (*(hi2s->i2s_rx))(hi2s);
      *I2S_CR2[hi2s->instance] |= SPI_CR2_RXNEIE_Msk;
    }
  }

}

static void i2s_tx_phillips(void *v_self)
{
  // Places data in the DR register of the I2S device per the specifications
  // in chapter 20 of the reference manual.
  i2s_handle_t * self = (i2s_handle_t *) v_self;
  if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_16BIT)
  {
    //16bit. Channel length doesn't affect anything (see ref manual 20.4.3)
	*I2S_DR[self->instance] = (uint16_t)( 0x0000FFFFUL & (*(self->p_tx_buffer)) );
    self->p_tx_buffer++;
    self->tx_len--;
  }
  else if (((*I2S_CFGR[self->instance]) & (SPI_I2SCFGR_I2SCFG_Msk)) == DATA_24BIT) //24bit data length
  {
	*I2S_DR[self->instance] = (uint16_t)( (0x00FFFF00UL & (*(self->p_tx_buffer))) >> 8U);
    while((*I2S_SR[self->instance] & SPI_SR_TXE_Msk) == 0);
    *I2S_DR[self->instance] = (uint16_t)( (0x000000FFUL & (*(self->p_tx_buffer))) << 8U);
    self->p_tx_buffer++;
    self->tx_len--;
  }
  else if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_32BIT) //32bit data Length
  {
    *I2S_DR[self->instance] = (uint16_t) ((0xFFFF0000UL & (*(self->p_tx_buffer))) >> 16U);
    while((*I2S_SR[self->instance] & SPI_SR_TXE_Msk) == 0);
    *I2S_DR[self->instance] = (uint16_t) (0x0000FFFFUL & (*(self->p_tx_buffer)));
    self->p_tx_buffer++;
    self->tx_len--;

  }
}

static void i2s_rx_phillips(void *v_self)
{
  i2s_handle_t *self = (i2s_handle_t *) v_self;
    // Places data in the DR register of the I2S device per the specifications
    // in chapter 20 of the reference manual.
  if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_16BIT)
  {
    //16 bit data length.
    *(self->p_rx_buffer) = *I2S_DR[self->instance];
    self->p_rx_buffer++;
    self->rx_len--;
  }
  else if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_24BIT) //24bit data length
  {
    //24 bit data length
    *(self->p_rx_buffer) = *I2S_DR[self->instance] << 8U;
    while((*I2S_SR[self->instance] & SPI_SR_RXNE_Msk) == 0);
    *(self->p_rx_buffer) |= *I2S_DR[self->instance] >> 8U;
    self->p_rx_buffer++;
    self->rx_len--;
  }
  else if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_32BIT) //32bit data Length
  {
    //32 bit data length.
    *(self->p_rx_buffer) = *I2S_DR[self->instance] << 16U;
    while((*I2S_SR[self->instance] & SPI_SR_RXNE_Msk) == 0);
    *(self->p_rx_buffer) |= *I2S_DR[self->instance];
    self->p_rx_buffer++;
    self->rx_len--;
  }
}

static void i2s_tx_lsb(void *v_self)
{
	i2s_handle_t * self = (i2s_handle_t *) v_self;
    // Places data in the DR register of the I2S device per the specifications
    // in chapter 20 of the reference manual.
	  if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_16BIT)
    {
      //16bit. Channel length doesn't affect anything (see ref manual 20.4.3)
	  *I2S_DR[self->instance] = (uint16_t)( 0x0000FFFFUL & (*(self->p_tx_buffer)) );
      self->p_tx_buffer++;
      self->tx_len--;
    }

	  else if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_24BIT) //24bit data length
    {
      *I2S_DR[self->instance] = (uint16_t)( 0x00FF0000UL & (*(self->p_tx_buffer)) >> 16U);
      while((*I2S_SR[self->instance] & SPI_SR_TXE_Msk) == 0);
      *I2S_DR[self->instance] = (uint16_t)( 0x0000FFFFUL & (*(self->p_tx_buffer)));
      self->p_tx_buffer++;
      self->tx_len--;
    }
	  else if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_32BIT) //32bit data Length
    {
	  *I2S_DR[self->instance] = (uint16_t) (0xFFFF0000UL & (*(self->p_tx_buffer)) >> 16U);
      while((*I2S_SR[self->instance] & SPI_SR_TXE_Msk) == 0);
      *I2S_DR[self->instance] = (uint16_t) (0x0000FFFFUL & (*(self->p_tx_buffer)));
      self->p_tx_buffer++;
      self->tx_len--;

    }
}

static void i2s_rx_lsb(void *v_self)
{
	i2s_handle_t *self = (i2s_handle_t *) v_self;
    // Places data in the DR register of the I2S device per the specifications
    // in chapter 20 of the reference manual.
	  if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) == DATA_16BIT)
  {
    //16 bit data length - RNXE only triggered when non-null/extended data comes in
    *(self->p_rx_buffer) = *I2S_DR[self->instance];
    self->p_rx_buffer++;
    self->rx_len--;
  }
	  else if ((*I2S_CFGR[self->instance] & SPI_I2SCFGR_I2SCFG_Msk) >= DATA_24BIT) //24bit data length
  {
    //24 bit and 32 bit data length can be processed identically
    *(self->p_rx_buffer) = *I2S_DR[self->instance] << 16U;
    while((*I2S_SR[self->instance] & SPI_SR_RXNE_Msk) == 0);
    *(self->p_rx_buffer) |= *I2S_DR[self->instance];
    self->p_rx_buffer++;
    self->rx_len--;
  }
}

void i2s_reg_write(uint32_t i2s_register, uint16_t value)
//PRE-CONDITION : register is within I2S register state
{
	*(uint16_t *)i2s_register = value;
}

uint16_t i2s_reg_read(uint32_t i2s_register)
//PRE-CONDITION : register is within I2S register state
{
	return *(uint16_t *)i2s_register;
}


void i2s_deinit(i2s_handle_t *hi2s)
{
  uint32_t mode = (*I2S_CFGR[hi2s->instance] & (SPI_I2SCFGR_I2SCFG_Msk)) >> SPI_I2SCFGR_I2SCFG_Pos;
  uint32_t counter = 0;
  if (mode == 0x02)
  {
    //mode == master transmit. Stop sequence as per 20.4.5 in Reference Manual.
   while( ((*I2S_SR[hi2s->instance] & SPI_SR_TXE_Msk) == 0)
          || ((*I2S_SR[hi2s->instance] & SPI_SR_BSY_Msk) != 0 ) );
   *I2S_CFGR[hi2s->instance] &= ~(SPI_I2SCFGR_I2SE_Msk); //disable i2s.
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
    if ((*I2S_CFGR[hi2s->instance] & SPI_I2SCFGR_CHLEN_Msk) == CHANNEL_32BIT &&
    		(*I2S_CFGR[hi2s->instance] & SPI_I2SCFGR_DATLEN_Msk) == DATA_16BIT)
    {
      if ((*I2S_CFGR[hi2s->instance] & SPI_I2SCFGR_I2SSTD_Msk) == LSB_STD)
      {
        //stop with lsb 16 bit 32 extended
        while((hi2s->rx_len > 1 || *I2S_SR[hi2s->instance] & SPI_SR_RXNE_Msk) == 0);
        while(counter < 17) //wait for 17 cycles
        {
          while((GPIOB->ODR & (0x01 << clock_B_pin)) != 0);
          counter++;
        }
        *I2S_CFGR[hi2s->instance] &= ~(SPI_I2SCFGR_I2SE_Msk);
      }
      else
      {
        //stop with non-lsb 16 bit 32 extended
        while((hi2s->rx_len != 0 || *I2S_SR[hi2s->instance] & SPI_SR_RXNE_Msk) == 0); //wait for LAST rxne
        while((GPIOB->ODR & (0x01 << clock_B_pin)) != 0); //wait for one cycle
        *I2S_CFGR[hi2s->instance] &= ~(SPI_I2SCFGR_I2SE_Msk);
      }
    }
    else
    {
      while((hi2s->rx_len > 1 || *I2S_SR[hi2s->instance] & SPI_SR_RXNE_Msk) == 0);
      while((GPIOB->ODR & (0x01 << clock_B_pin)) != 0); //wait for one cycle
      *I2S_CFGR[hi2s->instance] &= ~(SPI_I2SCFGR_I2SE_Msk);
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
