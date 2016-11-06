

#include "gd32f1x0.h"
#include "config.h"
#include "hardware.h"
#include "drv_time.h"
#include "drv_rgb.h"


#if ( RGB_LED_NUMBER > 0)

#define gpioset( port , pin) 	port->BOR = pin
#define gpioreset( port , pin) port->BCR = pin

#define RGBHIGH gpioset( RGB_PORT, RGB_PIN)
#define RGBLOW gpioreset( RGB_PORT, RGB_PIN);

void rgb_init(void)
{    
	// spi port inits

	if ( (RGB_PIN == GPIO_PIN_13 || RGB_PIN == GPIO_PIN_14) && RGB_PORT == GPIOA ) 
	{
		// programming port used
		
		// wait until 2 seconds from powerup passed
		while ( gettime() < 2e6 ) delay(10);
	}
	
		GPIO_InitPara  GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
  GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;

	GPIO_InitStructure.GPIO_Pin = RGB_PIN;
	GPIO_Init(RGB_PORT, &GPIO_InitStructure);

	for (int i = 0 ; i < RGB_LED_NUMBER ; i++)
	{
		rgb_send( 0x0 );
	}

}


#pragma push

//#pragma Otime
//#pragma O2

#ifdef __GNUC__

// gcc delay
// delay for T0H ( 350uS )
void delay2a()
{
	asm ("nop");
	asm ("nop");
	
	asm ("nop");
	asm ("nop");
	
	asm ("nop");
	asm ("nop");
	
	asm ("nop");
	asm ("nop");
	
	asm ("nop");
	asm ("nop");
}

#else

// Keil delay
// delay for T0H ( 350uS )
void delay2a()
{
	
	__asm{NOP}
	__asm{NOP}
			
	__asm{NOP}
	__asm{NOP}
	
	__asm{NOP}
	__asm{NOP}
	
	__asm{NOP}
	__asm{NOP}
	
	__asm{NOP}
	__asm{NOP}

	__asm{NOP}
// 7 - 14 nops	range

}
#endif


void rgb_send( int data)
{
for ( int i =23 ; i >=0 ; i--)
	{
		RGBHIGH;
		if (  (data>>i)&1  ) 
		{
			delay2a();
			delay2a();
			RGBLOW;
		}
		else 
		{
			delay2a();
			RGBLOW;
		}
		delay2a();
		delay2a();


	}
	
}



#pragma pop

#else
// rgb led not found
// some dummy headers just in case
void rgb_init(void)
{
	
}

void rgb_send( int data)
{
	
}
#endif











