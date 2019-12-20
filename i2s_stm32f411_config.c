#include "i2s_stm32f411_config.h"

static const i2s_config_t i2s_config_table[] = {
		  //I2S		//I2S		//I2S			//I2S		//PCM		//CLOCK			//DATA				//CHANNEL			//MASTER	//PRESCALER
		  //Name    //Enable	//MODE	    	//STD		//SYNC		//POLARITY		//LENGTH			//LENGTH			//CLOCK		//ODD
		{ I2S2,		1,  MASTER_TRANSMIT,	 PCM_STD,	 SHORT_FRAME,	  SS_LOW,		DATA_16BIT,		CHANNEL_32BIT,		0,	EVEN_PRESCALER},
		{ I2S3,		0,	  0,		0,			0,			0,				0,			0,			0,			0},
  };

const i2s_config_t *i2s_config_table_get(void)
{
	return (i2s_config_table);
}

