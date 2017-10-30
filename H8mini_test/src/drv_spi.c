

#include "gd32f1x0.h"
#include "drv_spi.h"
#include "macros.h"
#include "binary.h"
#include "hardware.h"

void spi_init(void)
{
    
	GPIO_InitPara GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;

	GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
	GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_CLK_PIN;
	GPIO_Init(SPI_CLK_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN;
	GPIO_Init(SPI_SS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
	GPIO_Init(SPI_MISO_PORT, &GPIO_InitStructure);
	
	spi_csoff();
    
}


#define gpioset( port , pin) port->BOR = pin
#define gpioreset( port , pin) port->BCR = pin

#define MOSIHIGH gpioset( SPI_MOSI_PORT, SPI_MOSI_PIN)
#define MOSILOW  gpioreset( SPI_MOSI_PORT, SPI_MOSI_PIN)
#define SCKHIGH  gpioset( SPI_CLK_PORT,  SPI_CLK_PIN)
#define SCKLOW   gpioreset( SPI_CLK_PORT,  SPI_CLK_PIN)

#define SPION gpioset( SPI_SS_PORT, SPI_SS_PIN)

#define READMISO ((SPI_MISO_PORT->DIR & SPI_MISO_PIN) != (uint32_t)Bit_RESET)
#ifndef __GNUC__

#pragma push

#pragma Otime
#pragma O2

#endif
__inline void spi_cson()
{
	GPIO_WriteBit(SPI_SS_PORT, SPI_SS_PIN, Bit_RESET);
}

__inline void spi_csoff()
{
	gpioset(SPI_SS_PORT, SPI_SS_PIN);
}



void spi_sendbyte(int data)
{
	for (int i = 7; i >= 0; i--)
	  {
		  if (bitRead(data, i))
		    {
			    MOSIHIGH;
		    }
		  else
		    {
			    MOSILOW;
		    }

		  SCKHIGH;
		  SCKLOW;
	  }
}


int spi_sendrecvbyte2(int data)
{
	int recv = 0;
	for (int i = 7; i >= 0; i--)
	  {
		  if ((data) & (1 << 7))
		    {
			    MOSIHIGH;
		    }
		  else
		    {
			    MOSILOW;
		    }
		  SCKHIGH;
		  data = data << 1;
		  if (READMISO)
			  recv = recv | (1 << 7);

		  recv = recv << 1;
		  SCKLOW;
	  }
	recv = recv >> 8;
	return recv;
}


int spi_sendrecvbyte(int data)
{
	int recv = 0;

	for (int i = 7; i >= 0; i--)
	  {
		  recv = recv << 1;
		  if ((data) & (1 << 7))
		    {
			    MOSIHIGH;
		    }
		  else
		    {
			    MOSILOW;
		    }

		  data = data << 1;

		  SCKHIGH;

		  if (READMISO)
			  recv = recv | 1;

		  SCKLOW;

	  }
	return recv;
}


int spi_sendzerorecvbyte()
{
	int recv = 0;
	MOSILOW;

	for (int i = 7; i >= 0; i--)
	  {
		  recv = recv << 1;

		  SCKHIGH;

		  if (READMISO)
			  recv = recv | 1;

		  SCKLOW;

	  }
	return recv;
}

#ifndef __GNUC__
#pragma pop
#endif
