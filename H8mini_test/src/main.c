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


// Eachine H8mini acro firmware
// files of this project are assumed MIT licence unless otherwise noted

// licence of files binary.h and macros.h LGPL 2.1


#include "gd32f1x0.h"

#include "config.h"
#include "led.h"
#include "util.h"
#include "sixaxis.h"
#include "drv_adc.h"
#include "drv_time.h"
#include "drv_softi2c.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "drv_gpio.h"
#include "drv_rgb.h"
#include "drv_serial.h"
#include "rx_bayang.h"
#include "drv_spi.h"
#include "control.h"
#include "defines.h"
#include "drv_i2c.h"
#include "buzzer.h"
#include "binary.h"
#include <math.h>
#include "hardware.h"
#include "drv_servo.h"

#include <inttypes.h>



#ifdef __GNUC__
// gcc warnings and fixes
#ifdef AUTO_VDROP_FACTOR
	#undef AUTO_VDROP_FACTOR
	#warning #define AUTO_VDROP_FACTOR not working with gcc, using fixed factor
#endif
#endif



// hal
void clk_init(void);

float looptime;

void failloop(int val);

unsigned long maintime;
unsigned long lastlooptime;

int ledcommand = 0;
unsigned long ledcommandtime = 0;


int lowbatt = 0;
float vbatt = 4.2;
float vbattfilt = 4.2;
float vbatt_comp = 4.2;
int random_seed = 0;

extern char aux[AUXNUMBER];

extern void loadcal(void);
extern void imu_init(void);


int main(void)
{

	clk_init();

	gpio_init();

#ifdef SERIAL
	serial_init();
#endif

	i2c_init();

	spi_init();

	pwm_init();
#ifdef SERVO_DRIVER
    servo_init();
#endif    
	for (int i = 0; i <= 3; i++)
	  {
		  pwm_set(i, 0);
	  }

	time_init();


	if (RCC_GetCK_SYSSource() == 8)
	  {
          
	  }
	else
	  {
		  failloop(5);
	  }

	sixaxis_init();

	if (sixaxis_check())
	  {
#ifdef SERIAL
		  printf(" MPU found \n");
#endif
	  }
	else
	  {
#ifdef SERIAL
		  printf("ERROR: MPU NOT FOUND \n");
#endif
		  failloop(4);
	  }

	adc_init();

	rx_init();

	int count = 0;
	vbattfilt = 0.0;

	while (count < 64)
	  {
		  vbattfilt += adc_read(1);
		  count++;
	  }
       // for randomising MAC adddress of ble app - this will make the int = raw float value        
		random_seed =  *(int *)&vbattfilt ; 
		random_seed = random_seed&0xff;
      
	vbattfilt = vbattfilt / 64;

#ifdef SERIAL
	printf("Vbatt %2.2f \n", vbattfilt);
#ifdef NOMOTORS
	printf("NO MOTORS\n");
#warning "NO MOTORS"
#endif
#endif

#ifdef STOP_LOWBATTERY
// infinite loop
	if (vbattfilt < STOP_LOWBATTERY_TRESH)
		failloop(2);
#endif

// loads acc calibration and gyro dafaults
	loadcal();

	gyro_cal();

	rgb_init();
	
	imu_init();
	
	extern unsigned int liberror;
	if (liberror)
	  {
#ifdef SERIAL
		  printf("ERROR: I2C \n");
#endif
		  failloop(7);
	  }


	lastlooptime = gettime();
	extern int rxmode;
	extern int failsafe;

	float thrfilt;

//
//
//              MAIN LOOP
//
//


	checkrx();

	while (1)
	  {
		  // gettime() needs to be called at least once per second 
		  maintime = gettime();
		  looptime = ((uint32_t) (maintime - lastlooptime));
		  if (looptime <= 0)
			  looptime = 1;
		  looptime = looptime * 1e-6f;
		  if (looptime > 0.02f)	// max loop 20ms
		    {
			    failloop(3);
			    //endless loop                  
		    }
		  lastlooptime = maintime;

		  if (liberror > 20)
		    {
			    failloop(8);
			    // endless loop
		    }

		  sixaxis_read();

		  control();

// battery low logic
				
		float hyst;
		float battadc = adc_read(1);
vbatt = battadc;
		// average of all 4 motor thrusts
		// should be proportional with battery current			
		extern float thrsum; // from control.c
		// filter motorpwm so it has the same delay as the filtered voltage
		// ( or they can use a single filter)		
		lpf ( &thrfilt , thrsum , 0.9968f);	// 0.5 sec at 1.6ms loop time	
		
		lpf ( &vbattfilt , battadc , 0.9968f);		

#ifdef AUTO_VDROP_FACTOR

static float lastout[12];
static float lastin[12];
static float vcomp[12];
static float score[12];
static int current_index = 0;

int minindex = 0;
float min = score[0];

{
	int i = current_index;

	vcomp[i] = vbattfilt + (float) i *0.1f * thrfilt;
		
	if ( lastin[i] < 0.1f ) lastin[i] = vcomp[i];
	float temp;
	//	y(n) = x(n) - x(n-1) + R * y(n-1) 
	//  out = in - lastin + coeff*lastout
		// hpf
	 temp = vcomp[i] - lastin[i] + FILTERCALC( 1000*12 , 1000e3) *lastout[i];
		lastin[i] = vcomp[i];
		lastout[i] = temp;
	 lpf ( &score[i] , fabsf(temp) , FILTERCALC( 1000*12 , 10e6 ) );

	}
	current_index++;
	if ( current_index >= 12 ) current_index = 0;

	for ( int i = 0 ; i < 12; i++ )
	{
	 if ( score[i] < min )  
		{
			min = score[i];
			minindex = i;
		}
}

#undef VDROP_FACTOR
#define VDROP_FACTOR  minindex * 0.1f
#endif

		if ( lowbatt ) hyst = HYST;
		else hyst = 0.0f;

		vbatt_comp = vbattfilt + (float) VDROP_FACTOR * thrfilt;

		if ( vbatt_comp <(float) VBATTLOW + hyst ) lowbatt = 1;
		else lowbatt = 0;
		

// led flash logic              

		  if (rxmode != RX_MODE_BIND)
		    {
					// non bind                    
			    if (failsafe)
			      {
				      if (lowbatt)
					      ledflash(500000, 8);
				      else
					      ledflash(500000, 15);
			      }
			    else
			      {
				      if (lowbatt)
					      ledflash(500000, 8);
				      else
					{
						if (ledcommand)
						  {
							  if (!ledcommandtime)
								  ledcommandtime = gettime();
							  if (gettime() - ledcommandtime > 500000)
							    {
								    ledcommand = 0;
								    ledcommandtime = 0;
							    }
							  ledflash(100000, 8);
						  }
						else
						{
							if ( aux[LEDS_ON] )
							ledon( 255);
							else 
							ledoff( 255);
						}
					}
			      }
		    }
		  else
		    {		// bind mode
			    ledflash(100000 + 500000 * (lowbatt), 12);
		    }

// rgb strip logic   
#if (RGB_LED_NUMBER > 0)				
	extern void rgb_led_lvc( void);
	rgb_led_lvc( );
#endif
				
#ifdef BUZZER_ENABLE
	buzzer();
#endif

#ifdef SERVO_DRIVER
servo_timer_loop( );
#endif
            
#ifdef FPV_ON
			static int fpv_init = 0;
			if ( rxmode == RX_MODE_NORMAL && ! fpv_init ) {
				fpv_init = gpio_init_fpv();
			}
			if ( fpv_init ) {
				if ( failsafe ) {
					GPIO_WriteBit( FPV_PIN_PORT, FPV_PIN, Bit_RESET );
				} else {
					GPIO_WriteBit( FPV_PIN_PORT, FPV_PIN, aux[ FPV_ON ] ? Bit_SET : Bit_RESET );
				}
			}
#endif

	checkrx();
				
					
	// loop time 1ms                
	while ((gettime() - maintime) < (1000 - 22) )
			delay(10);



	}			// end loop


}

// 2 - low battery at powerup - if enabled by config
// 3 - loop time issue
// 4 - Gyro not found
// 5 - clock , intterrupts , systick
// 7 - i2c error 
// 8 - i2c error main loop

void failloop(int val)
{
	for (int i = 0; i <= 3; i++)
	  {
		  pwm_set(i, 0);
	  }

	while (1)
	  {
		  for (int i = 0; i < val; i++)
		    {
			    ledon(255);
			    delay(200000);
			    ledoff(255);
			    delay(200000);
		    }
		  delay(800000);
	  }

}


void HardFault_Handler(void)
{
	failloop(5);
}

void MemManage_Handler(void)
{
	failloop(5);
}

void BusFault_Handler(void)
{
	failloop(5);
}

void UsageFault_Handler(void)
{
	failloop(5);
}
