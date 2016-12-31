// CHANGING THE H-BRIDGE CODE CAN RESULT IN CONNECTING THE FETs ACROSS THE
// BATTERY AND AS SUCH BREAKING THE BOARD. ( h101 only )

// Dshot driver for H101_dual firmware. Written by Markus Gritsch.
// No throttle jitter, no min/max calibration, just pure digital goodness :)

// Dshot150 would be fast enough for up to 8 kHz main loop frequency. But
// since this implementation does simple bit banging, Dshot150 takes a lot of
// our 1 ms main loop time. Dshot300 takes less time for bit banging, which
// leaves more idle time. Implementing the driver using DMA (like Betaflight
// does) is left as an excercise for the reader ;)

// The ESC signal must be taken before the FET, i.e. non-inverted. The
// signal after the FET with a pull-up resistor is not good enough.
// Bit-bang timing tested only with Keil compiler.

// Dshot capable ESCs required. Consider removing the input filter cap,
// especially if you get drop outs. Tested on "Racerstar MS Series 15A ESC
// BLHeLi_S OPTO 2-4S" ESCs (rebranded ZTW Polaris) with A_H_20_REV16_43.HEX
// and removed filter cap.

// USE AT YOUR OWN RISK. ALWAYS REMOVE PROPS WHEN TESTING.


// Enable this for 3D. The 'Motor Direction' setting in BLHeliSuite must
// be set to 'Bidirectional' (or 'Bidirectional Rev.') accordingly:
//#define BIDIRECTIONAL

// Select Dshot150 or Dshot300. Dshot150 consumes quite some main loop time.
// DShot300 may require removing the input filter cap on the ESC:
#define DSHOT150
//#define DSHOT300

// IDLE_OFFSET is added to the throttle. Adjust its value so that the motors
// still spin at minimum throttle.
#define IDLE_OFFSET 40


#include <gd32f1x0.h>

#include "config.h"
#include "defines.h"
#include "drv_pwm.h"
#include "drv_time.h"
#include "hardware.h"

#ifdef USE_DSHOT_DRIVER

#ifdef THREE_D_THROTTLE
#error "Not tested with THREE_D_THROTTLE config option"
#endif

#ifdef __GNUC__
#error "Bit-bang timing not tested with GCC"
#endif

extern int failsafe;
extern int onground;

int pwmdir = 0;
static unsigned long pwm_failsafe_time = 1;
static uint8_t motor_data[ 64 ] = { 0 };

static void make_packet( uint8_t number, uint16_t value );
static void bitbang_data( void );

// PA8 - 9 - 10
// PB1

// FL - PA8
#define DSHOT_PIN_1 GPIO_PIN_8
#define DSHOT_PORT_1 GPIOA
// BL - PA9
#define DSHOT_PIN_2 GPIO_PIN_9
#define DSHOT_PORT_2 GPIOA
//FR - PA10
#define DSHOT_PIN_3 GPIO_PIN_10
#define DSHOT_PORT_3 GPIOA
//BR - PB1
#define DSHOT_PIN_4 GPIO_PIN_1
#define DSHOT_PORT_4 GPIOB

#ifndef FORWARD
#define FORWARD 0
#define REVERSE 1
#endif


void pwm_init()
{
	GPIO_InitPara GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
	GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;


	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_1 ;
	GPIO_Init( DSHOT_PORT_1, &GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_2 ;
	GPIO_Init( DSHOT_PORT_2, &GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_3 ;
	GPIO_Init( DSHOT_PORT_3, &GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_4 ;
	GPIO_Init( DSHOT_PORT_4, &GPIO_InitStructure );


	// set failsafetime so signal is off at start
	pwm_failsafe_time = gettime() - 100000;
	
	pwmdir = FORWARD;
}

void pwm_set( uint8_t number, float pwm )
{
	if ( pwm < 0.0f ) {
		pwm = 0.0;
	}
	if ( pwm > 0.999f ) {
		pwm = 0.999;
	}

	uint16_t value = 0;

#ifdef BIDIRECTIONAL

	if ( pwmdir == FORWARD ) {
		// maps 0.0 .. 0.999 to 48 + IDLE_OFFSET .. 1047
		value = 48 + IDLE_OFFSET + (uint16_t)( pwm * ( 1000 - IDLE_OFFSET ) );
	} else if ( pwmdir == REVERSE ) {
		// maps 0.0 .. 0.999 to 1048 + IDLE_OFFSET .. 2047
		value = 1048 + IDLE_OFFSET + (uint16_t)( pwm * ( 1000 - IDLE_OFFSET ) );
	}

#else

	// maps 0.0 .. 0.999 to 48 + IDLE_OFFSET * 2 .. 2047
	value = 48 + IDLE_OFFSET * 2 + (uint16_t)( pwm * ( 2001 - IDLE_OFFSET * 2 ) );

#endif

	if ( onground ) {
		value = 0; // stop the motors
	}

	if ( failsafe ) {
		if ( ! pwm_failsafe_time ) {
			pwm_failsafe_time = gettime();
		} else {
			// 100ms after failsafe we turn off the signal (for safety while flashing)
			if ( gettime() - pwm_failsafe_time > 100000 ) {
				value = 0;
			}
		}
	} else {
		pwm_failsafe_time = 0;
	}

	make_packet( number, value );

	if ( number == 3 ) {
		bitbang_data();
	}
}

static void make_packet( uint8_t number, uint16_t value )
{
	uint16_t packet = ( value << 1 ) | 0; // Here goes telemetry bit (false for now)
	// compute checksum
	uint16_t csum = 0;
	uint16_t csum_data = packet;
	for ( uint8_t i = 0; i < 3; ++i ) {
		csum ^= csum_data; // xor data by nibbles
		csum_data >>= 4;
	}
	csum &= 0xf;
	// append checksum
	packet = ( packet << 4 ) | csum;

	// generate pulses for whole packet
	for ( uint8_t i = 0; i < 16; ++i ) {
		if ( packet & 0x8000 ) { // MSB first
			motor_data[ i * 4 + 0 ] |= 1 << number;
			motor_data[ i * 4 + 1 ] |= 1 << number;
			motor_data[ i * 4 + 2 ] |= 1 << number;
			motor_data[ i * 4 + 3 ] |= 0 << number;
		} else {
			motor_data[ i * 4 + 0 ] |= 1 << number;
			motor_data[ i * 4 + 1 ] |= 0 << number;
			motor_data[ i * 4 + 2 ] |= 0 << number;
			motor_data[ i * 4 + 3 ] |= 0 << number;
		}
		packet <<= 1;
	}
}

// Do not change anything between #pragma push and #pragma pop
// without redoing thorough timing measurements.
#pragma push
#pragma O2

#define gpioset( port , pin) port->BOR = pin
#define gpioreset( port , pin) port->BCR = pin

static void bitbang_data()
{
	for ( uint8_t i = 0; i < 64; ++i ) {
		const uint8_t data = motor_data[ i ];
		motor_data[ i ] = 0;

		if ( data & 0x01 ) {
			__asm{NOP}
			gpioset( DSHOT_PORT_1, DSHOT_PIN_1 ); // FL
		} else {
			__asm{NOP} __asm{NOP}
			gpioreset( DSHOT_PORT_1, DSHOT_PIN_1 );
		}
        
		if ( data & 0x04 ) {
			__asm{NOP}
			gpioset( DSHOT_PORT_2, DSHOT_PIN_2 );  // BL
		} else {
			__asm{NOP} __asm{NOP}
			gpioreset( DSHOT_PORT_2, DSHOT_PIN_2 );
		}

		if ( data & 0x08 ) {
			__asm{NOP}
			gpioset( DSHOT_PORT_3, DSHOT_PIN_3 ); // FR
		} else {
			__asm{NOP} __asm{NOP}
			gpioreset( DSHOT_PORT_3, DSHOT_PIN_3 );
		}
        
        if ( data & 0x02 ) {
			__asm{NOP}
			gpioset( DSHOT_PORT_4, DSHOT_PIN_4 ); // BR
		} else {
			__asm{NOP} __asm{NOP}
			gpioreset( DSHOT_PORT_4, DSHOT_PIN_4 );


		}

#if defined( DSHOT300 ) && ! defined( DSHOT150 )

		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}

#elif defined( DSHOT150 ) && ! defined( DSHOT300 )

		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}
		__asm{NOP} __asm{NOP} __asm{NOP} __asm{NOP}

#else
#error "Either define DSHOT150 or DSHOT300"
#endif

	}
}

#pragma pop

void motorbeep()
{
}

void pwm_dir( int dir )
{
	pwmdir = dir;
}

#endif
