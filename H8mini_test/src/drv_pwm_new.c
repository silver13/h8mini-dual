#include <gd32f1x0.h>

#include "drv_pwm.h"
#include "config.h"
#include "hardware.h"
#include "drv_time.h"
#include <stdlib.h>
#include  <math.h>

// pwm driver by Mike Morrison -  "bikemike"
// https://github.com/bikemike from the fq777-124 code

#ifdef USE_PWM_DRIVER_NEW

// CENTER ALIGNED PWM METHOD

// set in config.h 

//#define PWM_490HZ
//#define PWM_8KHZ_OLD
//#define PWM_8KHZ
//#define PWM_16KHZ
//#define PWM_24KHZ

// 490Khz
#ifdef PWM_490HZ
#define PWMTOP 16383
#define TIMER_PRESCALER 3
#endif


// 8Khz - ch div 3
#ifdef PWM_8KHZ_OLD
#define PWMTOP 1023
#define TIMER_PRESCALER 3
#endif

// 8Khz
#ifdef PWM_8KHZ
#define PWMTOP 3072
#define TIMER_PRESCALER 1
#endif

// 16Khz
#ifdef PWM_16KHZ
#define PWMTOP 1535
#define TIMER_PRESCALER 1
#endif

// 24Khz
#ifdef PWM_24KHZ
#define PWMTOP 1023
#define TIMER_PRESCALER 1
#endif


// 32Khz
#ifdef PWM_32KHZ
#define PWMTOP 767
#define TIMER_PRESCALER 1
#endif

typedef void (*PeriphClock_Enable_Func)(uint32_t, TypeState);

struct PWM_SETTINGS
{
	uint16_t pin;
	GPIO_TypeDef* port;
	uint8_t alt_func;
	uint8_t pin_src;
	TIMER_TypeDef* timer;
	uint8_t chan;
	volatile uint32_t* chcc;
	PeriphClock_Enable_Func periphclock_enable_func; 
	uint32_t periph_clock;
};

struct PWM_SETTINGS_2
{
	volatile uint32_t* chcc;
	uint8_t motor_id;
};

#define TIMER_CH1 0
#define TIMER_CH2 1
#define TIMER_CH3 2
#define TIMER_CH4 3

uint8_t pwm_order[4];

#if defined PWM_PA0 && defined PWM_PA5
#error PWM_PA0 and PWM_PA5 use the same timer
#endif

struct PWM_SETTINGS_2 pwm_settings_global[] = 
{
#ifdef PWM_PA0
	{ &TIMER2->CHCC1, PA0_MOTOR_ID},
#endif
#ifdef PWM_PA1
	{ &TIMER2->CHCC2, PA1_MOTOR_ID},
#endif
#ifdef PWM_PA2
	{ &TIMER2->CHCC3, PA2_MOTOR_ID},
#endif
#ifdef PWM_PA3
	{ &TIMER2->CHCC4, PA3_MOTOR_ID},
#endif
#ifdef PWM_PA4
	{ &TIMER14->CHCC1, PA4_MOTOR_ID},
#endif
#ifdef PWM_PA5
	{ &TIMER2->CHCC1, PA5_MOTOR_ID},
#endif
#ifdef PWM_PA6
	{ &TIMER3->CHCC1, PA6_MOTOR_ID},
#endif
#ifdef PWM_PA7
	{ &TIMER3->CHCC3, PA7_MOTOR_ID},
#endif
#ifdef PWM_PA8
	{ &TIMER1->CHCC1, PA8_MOTOR_ID},
#endif
#ifdef PWM_PA9
	{ &TIMER1->CHCC2, PA9_MOTOR_ID},
#endif
#ifdef PWM_PA10
	{  &TIMER1->CHCC3, PA10_MOTOR_ID},
#endif
#ifdef PWM_PA15
	{&TIMER2->CHCC1, PA15_MOTOR_ID},
#endif
#ifdef PWM_PB0
	{ &TIMER3->CHCC3, PB0_MOTOR_ID},
#endif
#ifdef PWM_PB1
	{ &TIMER3->CHCC4, PB1_MOTOR_ID},
#endif
#ifdef PWM_PB4
	{ &TIMER3->CHCC1, PB4_MOTOR_ID},
#endif
#ifdef PWM_PB5
	{ &TIMER3->CHCC2, PB5_MOTOR_ID},
#endif
#ifdef PWM_PB6
	{ &TIMER16->CHCC1, PB6_MOTOR_ID},
#endif
#ifdef PWM_PB7
	{ &TIMER17->CHCC1, PB7_MOTOR_ID},
#endif
};
static void init_timer(TIMER_TypeDef* timer, uint8_t chan_id, uint32_t period)
{
	TIMER_BaseInitPara TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIMER_Prescaler      = TIMER_PRESCALER - 1;	//
	TIM_TimeBaseStructure.TIMER_CounterMode    = TIMER_COUNTER_CENTER_ALIGNED2;
	TIM_TimeBaseStructure.TIMER_Period         = period;
	TIM_TimeBaseStructure.TIMER_ClockDivision  = TIMER_CDIV_DIV1;

	TIMER_BaseInit(timer, &TIM_TimeBaseStructure);

	TIMER_OCInitPara TIM_OCInitStructure;
	TIM_OCInitStructure.TIMER_OCMode      = TIMER_OC_MODE_PWM1;
	TIM_OCInitStructure.TIMER_OCPolarity  = TIMER_OC_POLARITY_HIGH;
	TIM_OCInitStructure.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
	TIM_OCInitStructure.TIMER_OCIdleState = TIMER_OC_IDLE_STATE_RESET;
	TIM_OCInitStructure.TIMER_Pulse       = 0;

	if (TIMER_CH1 == chan_id)
		TIMER_OC1_Init(timer,&TIM_OCInitStructure);
	else if (TIMER_CH2 == chan_id)
		TIMER_OC2_Init(timer,&TIM_OCInitStructure);
	else if (TIMER_CH3 == chan_id)
		TIMER_OC3_Init(timer,&TIM_OCInitStructure);
	else if (TIMER_CH4 == chan_id)
		TIMER_OC4_Init(timer,&TIM_OCInitStructure);
}



void pwm_init(void)
{

    
struct PWM_SETTINGS pwm_settings[] = 
{
#ifdef PWM_PA0
	{GPIO_PIN_0, GPIOA, GPIO_AF_2, GPIO_PINSOURCE0, TIMER2, TIMER_CH1, &TIMER2->CHCC1, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER2},
#endif
#ifdef PWM_PA1
	{GPIO_PIN_1, GPIOA, GPIO_AF_2, GPIO_PINSOURCE1, TIMER2, TIMER_CH2, &TIMER2->CHCC2, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER2},
#endif
#ifdef PWM_PA2
	{GPIO_PIN_2, GPIOA, GPIO_AF_2, GPIO_PINSOURCE2, TIMER2, TIMER_CH3, &TIMER2->CHCC3, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER2},
#endif
#ifdef PWM_PA3
	{GPIO_PIN_3, GPIOA, GPIO_AF_2, GPIO_PINSOURCE3, TIMER2, TIMER_CH4, &TIMER2->CHCC4, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER2},
#endif
#ifdef PWM_PA4
	{GPIO_PIN_4, GPIOA, GPIO_AF_4, GPIO_PINSOURCE4, TIMER14, TIMER_CH1, &TIMER14->CHCC1, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER14},
#endif
#ifdef PWM_PA5
	{GPIO_PIN_5, GPIOA, GPIO_AF_2, GPIO_PINSOURCE5, TIMER2, TIMER_CH1, &TIMER2->CHCC1, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER2},
#endif
#ifdef PWM_PA6
	{GPIO_PIN_6, GPIOA, GPIO_AF_1, GPIO_PINSOURCE6, TIMER3, TIMER_CH1, &TIMER3->CHCC1, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER3},
#endif
#ifdef PWM_PA7
	{GPIO_PIN_7, GPIOA, GPIO_AF_1, GPIO_PINSOURCE7, TIMER3, TIMER_CH2, &TIMER3->CHCC3, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER3},
#endif
#ifdef PWM_PA8
	{GPIO_PIN_8, GPIOA, GPIO_AF_2, GPIO_PINSOURCE8, TIMER1, TIMER_CH1, &TIMER1->CHCC1, &RCC_APB2PeriphClock_Enable, RCC_APB2PERIPH_TIMER1},
#endif
#ifdef PWM_PA9
	{GPIO_PIN_9, GPIOA, GPIO_AF_2, GPIO_PINSOURCE9, TIMER1, TIMER_CH2, &TIMER1->CHCC2, &RCC_APB2PeriphClock_Enable, RCC_APB2PERIPH_TIMER1},
#endif
#ifdef PWM_PA10
	{GPIO_PIN_10, GPIOA, GPIO_AF_2, GPIO_PINSOURCE10, TIMER1, TIMER_CH3, &TIMER1->CHCC3, &RCC_APB2PeriphClock_Enable, RCC_APB2PERIPH_TIMER1},
#endif
#ifdef PWM_PA15
	{GPIO_PIN_15, GPIOA, GPIO_AF_2, GPIO_PINSOURCE15, TIMER2, TIMER_CH1, &TIMER2->CHCC1, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER2},
#endif
#ifdef PWM_PB0
	{GPIO_PIN_0, GPIOB, GPIO_AF_1, GPIO_PINSOURCE0, TIMER3, TIMER_CH3, &TIMER3->CHCC3, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER3},
#endif
#ifdef PWM_PB1
	{GPIO_PIN_1, GPIOB, GPIO_AF_1, GPIO_PINSOURCE1, TIMER3, TIMER_CH4, &TIMER3->CHCC4, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER3},
#endif
#ifdef PWM_PB4
	{GPIO_PIN_4, GPIOB, GPIO_AF_1, GPIO_PINSOURCE4, TIMER3, TIMER_CH1, &TIMER3->CHCC1, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER3},
#endif
#ifdef PWM_PB5
	{GPIO_PIN_5, GPIOB, GPIO_AF_1, GPIO_PINSOURCE5, TIMER3, TIMER_CH2, &TIMER3->CHCC2, &RCC_APB1PeriphClock_Enable, RCC_APB1PERIPH_TIMER3},
#endif
#ifdef PWM_PB6
	{GPIO_PIN_6, GPIOB, GPIO_AF_2, GPIO_PINSOURCE6, TIMER16, TIMER_CH1, &TIMER16->CHCC1, &RCC_APB2PeriphClock_Enable, RCC_APB2PERIPH_TIMER16},
#endif
#ifdef PWM_PB7
	{GPIO_PIN_7, GPIOB, GPIO_AF_2, GPIO_PINSOURCE7, TIMER17, TIMER_CH1, &TIMER17->CHCC1, &RCC_APB2PeriphClock_Enable, RCC_APB2PERIPH_TIMER17},
#endif
};
    
	GPIO_InitPara GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
	GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;

	for (int i = 0; i < sizeof(pwm_settings) / sizeof(pwm_settings[0]); ++i)
	{
		GPIO_InitStructure.GPIO_Pin = pwm_settings[i].pin;
		GPIO_Init(pwm_settings[i].port, &GPIO_InitStructure);
		GPIO_PinAFConfig(pwm_settings[i].port, pwm_settings[i].pin_src, pwm_settings[i].alt_func);

		pwm_settings[i].periphclock_enable_func(pwm_settings[i].periph_clock, ENABLE);

		init_timer(pwm_settings[i].timer, pwm_settings[i].chan, PWMTOP);
	}

	for (int i = 0; i < sizeof(pwm_settings) / sizeof(pwm_settings[0]); ++i)
	{
		TIMER_CARLPreloadConfig(pwm_settings[i].timer, ENABLE);

		if (TIMER_CH1 == pwm_settings[i].chan)
			TIMER_OC1_Preload(pwm_settings[i].timer, TIMER_OC_PRELOAD_DISABLE);
		else if (TIMER_CH2 == pwm_settings[i].chan)
			TIMER_OC2_Preload(pwm_settings[i].timer, TIMER_OC_PRELOAD_DISABLE);
		else if (TIMER_CH3 == pwm_settings[i].chan)
			TIMER_OC3_Preload(pwm_settings[i].timer, TIMER_OC_PRELOAD_DISABLE);
		else if (TIMER_CH4 == pwm_settings[i].chan)
			TIMER_OC4_Preload(pwm_settings[i].timer, TIMER_OC_PRELOAD_DISABLE);

		TIMER_Enable(pwm_settings[i].timer, ENABLE);


		TIMER_CtrlPWMOutputs(pwm_settings[i].timer, ENABLE);
	}

    for ( int i = 0 ; i < 4 ; i++)
    {
      for ( int z = 0 ; z < sizeof(pwm_settings) / sizeof(pwm_settings[0]) ; z++)
        {
          if ( pwm_settings_global[z].motor_id == i )  
          {
            pwm_order[i] = z;  
          }
        }  
    }
    
}



extern int failsafe;
extern float rx[];
unsigned long motorbeeptime = 0;



#ifndef MOTOR_BEEPS_TIMEOUT
// default value if not defined elsewhere
#define MOTOR_BEEPS_TIMEOUT 30e6
#endif

#define MOTOR_BEEPS_PWM_ON 0.2
#define MOTOR_BEEPS_PWM_OFF 0.0


void motorbeep( void)
{
	if (failsafe)
	{
		unsigned long time = gettime();
		if (!motorbeeptime)
				motorbeeptime = time;
		else
			if ( time - motorbeeptime > MOTOR_BEEPS_TIMEOUT)
			{
				if ((time%2000000 < 125000))
				{
					
					for ( int i = 0 ; i <= 3 ; i++)
						{
						pwm_set( i , MOTOR_BEEPS_PWM_ON);
							delay(50);
						pwm_set( i , MOTOR_BEEPS_PWM_OFF);
							delay(50);
						pwm_set( i , MOTOR_BEEPS_PWM_ON);
							delay(50);
						pwm_set( i , MOTOR_BEEPS_PWM_OFF);
							delay(50);
						pwm_set( i , MOTOR_BEEPS_PWM_ON);
							delay(50);
						pwm_set( i , MOTOR_BEEPS_PWM_OFF);
											
						}
				
				}
				else
				{
				for ( int i = 0 ; i <= 3 ; i++)
					{
					pwm_set( i , MOTOR_BEEPS_PWM_OFF);
					}
					
				}
				
			}
	}
	else
		motorbeeptime = 0;
}


void pwm_set(uint8_t number, float pwm)
{
	pwm = pwm * PWMTOP;

	if (pwm < 0)
		pwm = 0;
	if (pwm > PWMTOP)
		pwm = PWMTOP;

	pwm = lroundf(pwm);

    (*pwm_settings_global[pwm_order[number]].chcc) = (uint32_t)pwm;
    
}


#endif

