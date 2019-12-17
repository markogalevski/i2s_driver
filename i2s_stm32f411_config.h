#ifndef _I2S_STM32F4XX_CFG
#define _I2S_STM32F4XX_CFG

#include <stdint.h>
#include "common.h"
typedef struct
{
  uint32_t i2s_name;
  uint32_t enable;
  uint32_t mode; //I2SCFG bits
  uint32_t i2s_std;
  uint32_t pcm_sync;
  uint32_t clock_pol;
  uint32_t data_len;
  uint32_t ch_len;
  uint8_t prescaler;
  uint32_t master_clock_enable;
  uint32_t prescaler_odd;
} i2s_config_t;

typedef enum
{
  I2S2 = 0U,
  I2S3 = 1U,
  NUM_I2S = 2U

}i2s_channel_t;

typedef enum
{
	SLAVE_TRANSMIT = 0U,
	SLAVE_RECEIVE = 1U,
	MASTER_TRANSMIT = 2U,
	MASTER_RECEIVE = 3U
}i2s_mode_t;

typedef enum
{
	PHILLIPS_STD = 0U,
	MSB_STD = 1U,
	LSB_STD = 2U,
	PCM_STD = 3U
}i2s_std_t;

typedef enum
{
	SHORT_FRAME = 0U,
	LONG_FRAME = 1U
}i2s_pcm_sync_t;

typedef enum
{
	SS_LOW = 0U,
	SS_HIGH = 1U
}i2s_clk_pol;

typedef enum
{
	DATA_16BIT = 0U,
	DATA_24BIT = 1U,
	DATA_32BIT = 2U
}i2s_data_length_t;


typedef enum
{
	CHANNEL_16BIT = 0U,
	CHANNEL_32BIT = 1U
}i2s_channel_length_t;

typedef uint8_t i2s_prescaler_t ;

typedef enable_t i2s_master_clock_output_en_t ;

typedef enum
{
	EVEN_PRESCALER = 0U,
    ODD_PRESCALER = 1U
}i2s_odd_prescaler_t;

const i2s_config_t * i2s_config_get(void);

#endif

