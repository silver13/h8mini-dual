/*
The MIT License (MIT)

Copyright (c) 2015 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// from Mike Morrison
// https://github.com/bikemike/h8mini-testing/blob/fq777-124/H8mini_test/src/led.c

#include "gd32f1x0.h"
#include "drv_time.h"
#include "macros.h"
#include "hardware.h"
#include "led.h"

#define LEDALL 15

void ledon( uint8_t val )
{
#if ( LED_NUMBER > 0 )
	#ifdef LED1_INVERT
	if ( val&1) GPIO_ResetBits( LED1PORT, LED1PIN);
	#else
	if ( val&1)	GPIO_SetBits( LED1PORT, LED1PIN);
	#endif
#endif
#if ( LED_NUMBER > 1 )
	#ifdef LED2_INVERT
  if ( val&2) GPIO_ResetBits( LED2PORT, LED2PIN);
	#else
	if ( val&2)	GPIO_SetBits( LED2PORT, LED2PIN);
	#endif
#endif
#if ( LED_NUMBER > 2 )
	#ifdef LED3_INVERT
	if ( val&4) GPIO_ResetBits( LED3PORT, LED3PIN);
	#else
	if ( val&4)	GPIO_SetBits( LED3PORT, LED3PIN);
	#endif
#endif
#if ( LED_NUMBER > 3 )
	#ifdef LED4_INVERT
	if ( val&8) GPIO_ResetBits( LED4PORT, LED4PIN);
	#else
	if ( val&8)	GPIO_SetBits( LED4PORT, LED4PIN);
	#endif
#endif
			
}

void ledoff( uint8_t val )
{
#if ( LED_NUMBER > 0 )	
	#ifdef LED1_INVERT
	if ( val&1)	GPIO_SetBits( LED1PORT, LED1PIN);
	#else
	if ( val&1) GPIO_ResetBits( LED1PORT, LED1PIN);
	#endif
#endif
#if ( LED_NUMBER > 1 )
	#ifdef LED2_INVERT
	if ( val&2)	GPIO_SetBits( LED2PORT, LED2PIN);
	#else
	if ( val&2) GPIO_ResetBits( LED2PORT, LED2PIN);
	#endif
#endif
#if ( LED_NUMBER > 2 )
	#ifdef LED3_INVERT
	if ( val&4)	GPIO_SetBits( LED3PORT, LED3PIN);
	#else
	if ( val&4) GPIO_ResetBits( LED3PORT, LED3PIN);
	#endif
#endif
#if ( LED_NUMBER > 3 )
	#ifdef LED4_INVERT
	if ( val&8)	GPIO_SetBits( LED4PORT, LED4PIN);
	#else
	if ( val&8) GPIO_ResetBits( LED4PORT, LED4PIN);	
	#endif
#endif
}


void ledflash( uint32_t period , int duty )
{
#if ( LED_NUMBER > 0 )	
	if ( gettime() % period > (period*duty)>>4 )
	{
		ledon(LEDALL);
	}
	else
	{
		ledoff(LEDALL);
	}
#endif	
}
