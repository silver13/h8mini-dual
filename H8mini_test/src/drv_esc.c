


//  ESC DRIVER FOR H8 MINI BOARD (GREEN)





#define ESC_MIN 1200
#define ESC_MAX 1800

#define ESC_THROTTLEOFF 900

// zero = no signal
#define ESC_FAILSAFE 0

// output polarity ( low - motor output with pullup resistor (500 ohms or near) )
// enable for motor output after fets 
#define INVERTED_PWM

// 50 - 500 Hz range
#define ESC_FREQ 500

// enable preload - less noise in esc output but longer latency
#define PRELOAD_ENABLE


//#define ONESHOT_125_ENABLE
















// do not change below

#define TIMER_PRESCALER 16

#ifndef SYS_CLOCK_FREQ_HZ
#define SYS_CLOCK_FREQ_HZ 48000000
#endif


// max pulse width in microseconds (auto calculated)
#define ESC_uS ((float)1000000.0f/(float)ESC_FREQ)

#define PWMTOP ((SYS_CLOCK_FREQ_HZ/(TIMER_PRESCALER) / ESC_FREQ ) - 1)

#define PWMTOP_US ( (float)1000000.0f/((SYS_CLOCK_FREQ_HZ/(TIMER_PRESCALER))/(PWMTOP + 1))  )


#if ( PWMTOP > 65535 )
#error "pwmtop too high"
#endif


// output polarity ( low - motor output with pullup resistor (500 ohms or near) )
// choice of TIMER_OC_POLARITY_LOW / TIMER_OC_POLARITY_HIGH
#ifdef INVERTED_PWM
// for motor output after fets
#define OUT_POLARITY TIMER_OC_POLARITY_LOW
#else
// for output before fets
#define OUT_POLARITY TIMER_OC_POLARITY_HIGH
#endif

#ifdef PRELOAD_ENABLE
// enable preload
#define ESC_PRELOAD TIMER_OC_PRELOAD_ENABLE
#else
// disable preload
#define ESC_PRELOAD TIMER_OC_PRELOAD_DISABLE
#endif


#include <gd32f1x0.h>
#include <math.h>

#include "drv_pwm.h"
#include "drv_time.h"
#include "util.h"
#include "config.h"
#include "hardware.h"

#ifdef USE_ESC_DRIVER

 TIMER_OCInitPara  TIM_OCInitStructure;


void pwm_init(void)
{
	
    GPIO_InitPara GPIO_InitStructure;

	// timer 3 pin PB1 config ( ch4 )
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
		
    GPIO_PinAFConfig(GPIOB,GPIO_PINSOURCE1,GPIO_AF_1);
	
	// GPIO pins PA8 , 9 ,10 setup ( Timer1 ch 1 , 2 , 3)
	  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;

    GPIO_Init(GPIOA,&GPIO_InitStructure);
		
    GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE8,GPIO_AF_2);
	  GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE9,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE10,GPIO_AF_2);
	
    TIMER_BaseInitPara TIM_TimeBaseStructure;

    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_TIMER1,ENABLE);
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_TIMER3,ENABLE);

// TIMER3 for pins A8 A9 A10

    TIM_TimeBaseStructure.TIMER_Prescaler = TIMER_PRESCALER - 1;  //
    TIM_TimeBaseStructure.TIMER_CounterMode = TIMER_COUNTER_UP;
    TIM_TimeBaseStructure.TIMER_Period = PWMTOP;
    TIM_TimeBaseStructure.TIMER_ClockDivision = TIMER_CDIV_DIV1;
    TIMER_BaseInit(TIMER3,&TIM_TimeBaseStructure);
		

 //Ch1 , 2 , 3 
    TIM_OCInitStructure.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIM_OCInitStructure.TIMER_OCPolarity = OUT_POLARITY;
    TIM_OCInitStructure.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.TIMER_OCIdleState = TIMER_OC_IDLE_STATE_RESET;
		
    TIM_OCInitStructure.TIMER_Pulse = 0;
    TIMER_OC4_Init(TIMER3, &TIM_OCInitStructure);

	TIMER_CtrlPWMOutputs(TIMER3,ENABLE);

    TIMER_CARLPreloadConfig(TIMER3,ENABLE);

    TIMER_Enable( TIMER3, ENABLE );
		
    TIMER_BaseInit(TIMER1,&TIM_TimeBaseStructure);

    TIMER_OC1_Init(TIMER1, &TIM_OCInitStructure);

    TIMER_OC2_Init(TIMER1, &TIM_OCInitStructure);

    TIMER_OC3_Init(TIMER1, &TIM_OCInitStructure);
		
	TIMER_CtrlPWMOutputs(TIMER1,ENABLE);	
		
    TIMER_CARLPreloadConfig(TIMER1,ENABLE);

	TIMER_OC1_Preload(TIMER1,ESC_PRELOAD);
	TIMER_OC2_Preload(TIMER1,ESC_PRELOAD);
	TIMER_OC3_Preload(TIMER1,ESC_PRELOAD);
	TIMER_OC4_Preload(TIMER3,ESC_PRELOAD);
	
  TIMER_Enable( TIMER1, ENABLE );
}



extern int onground;
extern int failsafe;
unsigned long pwm_failsafe_time = 1;

void pwm_set( uint8_t number , float pwm)
{

	if ( pwm < 0 ) pwm = 0;
	
	pwm = mapf ( pwm , 0 , 1 , ( (float) PWMTOP/PWMTOP_US)*ESC_MIN , ( (float) PWMTOP/PWMTOP_US)*ESC_MAX ); 

if ( onground ) pwm = ((float)PWMTOP/PWMTOP_US) * ESC_THROTTLEOFF;
	
	if ( failsafe ) 
	{
		if ( !pwm_failsafe_time )
		{
			pwm_failsafe_time = gettime();
		}
		else
		{
			// 100mS after failsafe we turn off the signal (for safety while flashing)
			if ( gettime() - pwm_failsafe_time > 100000 )
			{
				pwm = ((float)PWMTOP/PWMTOP_US) * ESC_FAILSAFE;
			}
		}
		
	}
	else
	{
		pwm_failsafe_time = 0;
	}

	if ( pwm > ((float)PWMTOP/PWMTOP_US)*ESC_MAX ) pwm = ((float)PWMTOP/PWMTOP_US)*ESC_MAX ;

#ifdef ONESHOT_125_ENABLE
	pwm = pwm/8;
#endif
	
	pwm = lroundf(pwm);
	

	

	
    if ( pwm < 0 ) pwm = 0;
  if ( pwm > PWMTOP ) pwm = PWMTOP;
	
	

  switch( number)
	{
		case 0:
		  TIMER1->CHCC1 = (uint32_t) pwm; 	  
		break;
		
		case 1:
		  TIMER3->CHCC4 = (uint32_t) pwm; 
		break;
		
		case 2:
		  TIMER1->CHCC2 = (uint32_t) pwm; 
		break;
		
		case 3:
		  TIMER1->CHCC3 = (uint32_t) pwm; 
		break;
		
		default:
			// handle error;
			//
		break;	
				
	}
	
}

void motorbeep()
{
}

#endif
