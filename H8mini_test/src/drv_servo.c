#include "gd32f1x0.h"
#include "drv_servo.h"
#include "drv_time.h"



#define SERVO0_PIN GPIO_PIN_0
#define SERVO0_PORT GPIOB


#define SERVO1_ENABLE

#define SERVO1_PIN GPIO_PIN_2
#define SERVO1_PORT GPIOA

#define SERVO_FREQ 50

// usage
// servo_set( 1500.2 , 1) // set servo 1 to 1500.20 uS


// code start
#define SV_TIMER TIMER14

void servo_init()
{
   	TIMER_BaseInitPara TIM_TimeBaseStructure;

	RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_TIMER15|RCC_APB2PERIPH_TIMER17|RCC_APB2PERIPH_TIMER16, ENABLE);
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_TIMER2|RCC_APB1PERIPH_TIMER14|RCC_APB1PERIPH_TIMER6, ENABLE);
    
    
	TIM_TimeBaseStructure.TIMER_Prescaler = 1;	// divide by 2
	TIM_TimeBaseStructure.TIMER_CounterMode = TIMER_COUNTER_UP;
	TIM_TimeBaseStructure.TIMER_Period = 48000; // 2ms
	TIM_TimeBaseStructure.TIMER_ClockDivision = TIMER_CDIV_DIV1;
	TIMER_BaseInit(SV_TIMER, &TIM_TimeBaseStructure);

    TIMER_INTConfig( SV_TIMER , TIMER_INT_UPDATE, ENABLE );  

   TIMER_CARLPreloadConfig(SV_TIMER, ENABLE);     
    
   TIMER_Enable(SV_TIMER, DISABLE);
    
    
    NVIC_PRIGroup_Enable(NVIC_PRIGROUP_1);

NVIC_InitPara NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQ = TIMER14_IRQn;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	GPIO_InitPara GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
	GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;	// GPIO_PUPD_NOPULL
    
    GPIO_InitStructure.GPIO_Pin = SERVO0_PIN;
	GPIO_Init(SERVO0_PORT, &GPIO_InitStructure);
#ifdef SERVO2_ENABLE    
    GPIO_InitStructure.GPIO_Pin = SERVO1_PIN;
	GPIO_Init(SERVO1_PORT, &GPIO_InitStructure);
#endif
}

volatile int servo_timer_enabled;
uint8_t servo_index = 0;

void TIM14_IRQHandler(void)
{
GPIO_WriteBit(SERVO0_PORT, SERVO0_PIN, Bit_RESET); 
    
#ifdef SERVO1_ENABLE
GPIO_WriteBit(SERVO1_PORT, SERVO1_PIN, Bit_RESET);  
#endif
    
TIMER_ClearIntBitState(  SV_TIMER , TIMER_INT_UPDATE );
TIMER_Enable(SV_TIMER, DISABLE);
servo_timer_enabled = 0;
}

unsigned long last_servo_time[2];
uint16_t servo_pwmtop[2];

void servo_set(float us , int num)
{
num&=1;   
unsigned int pwmtop = 24*us;
if ( pwmtop > 0xFFFF ) pwmtop = 0xFFFF; 
servo_pwmtop[num] = pwmtop;   
    
}

void servo_timer_loop( )
{
if ( servo_timer_enabled) return;  

if (   gettime() - last_servo_time[servo_index] > 1000000/ SERVO_FREQ )
{ 
servo_index++;
servo_index&=1;  
TIMER_SetAutoreload( SV_TIMER ,  servo_pwmtop[servo_index] );
TIMER_Enable(SV_TIMER, ENABLE); 
servo_timer_enabled = 1;      
if ( servo_index == 0 ) GPIO_WriteBit(SERVO0_PORT, SERVO0_PIN, Bit_SET); 
#ifdef SERVO1_ENABLE    
if ( servo_index == 1 ) GPIO_WriteBit(SERVO1_PORT, SERVO1_PIN, Bit_SET); 
#endif
last_servo_time[servo_index] = gettime();  
}
  
  
}
