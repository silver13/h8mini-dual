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

#include <inttypes.h>
#include <math.h>

#include "pid.h"
#include "config.h"
#include "util.h"
#include "drv_pwm.h"
#include "control.h"
#include "defines.h"
#include "drv_time.h"

#include "sixaxis.h"

int gestures2( void);
extern int ledcommand;		

extern float rx[4];
extern float gyro[3];
extern int failsafe;
extern float pidoutput[3];

extern float lpffilter( float in,int num );

extern char auxchange[AUXNUMBER];
extern char aux[AUXNUMBER];

extern float looptime;
extern float attitude[3];

int onground = 1;
float pwmsum;
float thrsum;

float rxcopy[4];

float error[PIDNUMBER];
float motormap( float input);
int lastchange;
int pulse;
float yawangle;
float angleerror[3];

/*
// bump test vars
int lastchange;
int pulse;
unsigned long timestart;
*/

extern float apid(int x );
extern void imu_calc( void);
extern void savecal( void);

void motorcontrol(void);
int gestures(void);
void pid_precalc( void);


void control( void)
{

	// hi rates
	float ratemulti;
	float ratemultiyaw;
	float maxangle;
	float anglerate;
	
	if ( aux[RATES] ) 
	{
		ratemulti = HIRATEMULTI;
		ratemultiyaw = HIRATEMULTIYAW;
		maxangle = MAX_ANGLE_HI;
		anglerate = LEVEL_MAX_RATE_HI;
	}
	else 
	{
		ratemulti = 1.0f;
		ratemultiyaw = 1.0f;
		maxangle = MAX_ANGLE_LO;
		anglerate = LEVEL_MAX_RATE_LO;
	}

/*	
int change = ( aux[PULSE] );

if ( change != lastchange )
{
	pulse = 1;
}
lastchange = change;

float motorchange = 0;


if ( pulse )
{
	if ( !timestart) timestart = gettime();
	
	
	if ( gettime() - timestart < 200000 )
	{
		motorchange = 0.2;	
	}
	else
	{
		motorchange = 0.0;
		pulse = 0;
		timestart = 0;
	}
}
*/
	
	for ( int i = 0 ; i < 3; i++)
	{
		rxcopy[i]=   rx[i];
	}
	
	
	yawangle = yawangle + gyro[2]*looptime;

	if ( auxchange[HEADLESSMODE] )
	{
		yawangle = 0;
	}
	
	if ( (aux[HEADLESSMODE] )&&!aux[LEVELMODE] ) 
	{
		float temp = rxcopy[0];
		rxcopy[0] = rxcopy[0] * cosf( yawangle) - rxcopy[1] * sinf(yawangle );
		rxcopy[1] = rxcopy[1] * cosf( yawangle) + temp * sinf(yawangle ) ;
	}
	
// check for acc calibration
		
	int command = gestures2();

	if ( command )
	{	
			if ( command == 3 )
			{
				gyro_cal(); // for flashing lights
				acc_cal();
				savecal();
				// reset loop time 
				extern unsigned lastlooptime;
				lastlooptime = gettime();
			}
			else
			{
				ledcommand = 1;
				if ( command == 2 )
				{
					aux[CH_AUX1]= 1;
					
				}
				if ( command == 1 )
				{
					aux[CH_AUX1]= 0;
				}
			}
	}

imu_calc();

pid_precalc();

	if ( aux[LEVELMODE] ) 
	{// level mode

	angleerror[0] = rxcopy[0] * maxangle - attitude[0];
	angleerror[1] = rxcopy[1] * maxangle - attitude[1];

	error[0] = apid(0) * anglerate * DEGTORAD  - gyro[0];
	error[1] = apid(1) * anglerate * DEGTORAD  - gyro[1];	 

	error[2] = rxcopy[2] * MAX_RATEYAW * DEGTORAD * ratemultiyaw - gyro[2];
		
	}
else
{ // rate mode
	
	error[0] = rxcopy[0] * MAX_RATE * DEGTORAD * ratemulti - gyro[0];
	error[1] = rxcopy[1] * MAX_RATE * DEGTORAD * ratemulti - gyro[1];
	
	// reduce angle Iterm towards zero
	extern float aierror[3];
	for ( int i = 0 ; i <= 3 ; i++) aierror[i] *= 0.8f;

	error[2] = rxcopy[2] * MAX_RATEYAW * DEGTORAD * ratemultiyaw - gyro[2];
}	


	pid(0);
	pid(1);
	pid(2);

// map throttle so under 10% it is zero	
float	throttle = mapf(rx[3], 0 , 1 , -0.1 , 1 );
if ( throttle < 0   ) throttle = 0;

// turn motors off if throttle is off and pitch / roll sticks are centered
	if ( failsafe || (throttle < 0.001f && (!ENABLESTIX||  (fabs(rx[0]) < 0.5f && fabs(rx[1]) < 0.5f ) ) ) ) 

	{ // motors off
		onground = 1;
		pwmsum = 0;
		thrsum = 0;
		for ( int i = 0 ; i <= 3 ; i++)
		{
			pwm_set( i , 0 );
		}	
	}
	else
	{
		onground = 0;
		float mix[4];	
		
//		pidoutput[0] += motorchange;
		
/*
static float offset;
if ( !pulse )
{
  offset = pidoutput[2];
}
else 	pidoutput[2] = motorchange + offset;
*/

		mix[MOTOR_FR] = throttle - pidoutput[0] - pidoutput[1] + pidoutput[2];		// FR
		mix[MOTOR_FL] = throttle + pidoutput[0] - pidoutput[1] - pidoutput[2];		// FL	
		mix[MOTOR_BR] = throttle - pidoutput[0] + pidoutput[1] - pidoutput[2];		// BR
		mix[MOTOR_BL] = throttle + pidoutput[0] + pidoutput[1] + pidoutput[2];		// BL	


		
		for ( int i = 0 ; i <= 3 ; i++)
		{
		float test = motormap( mix[i] );
		#ifndef NOMOTORS
		pwm_set( i , ( test )  );
		#else
		#warning "NO MOTORS"
		#endif
		}	

		for ( int i = 0 ; i <= 3 ; i++)
		{
			if ( mix[i] < 0 ) mix[i] = 0;
			if ( mix[i] > 1 ) mix[i] = 1;
			thrsum+= mix[i];
		}	
		thrsum = thrsum / 4;
		
	}// end motors on
	
}
	
/*
float motormap_old( float input)
{ 
	// this is a thrust to pwm function
	//  float 0 to 1 input and output
	// reverse of a power to thrust graph for 8.5 mm coreless motors + hubsan prop
	// should be ok for other motors without reduction gears.
	// a*x^2 + b*x + c
	// a = 0.75 , b = 0.061 , c = 0.185

if (input > 1.0) input = 1.0;
if (input < 0) input = 0;
	
if ( input < 0.25 ) return input;

input = input*input*0.75  + input*(0.0637);
input += 0.185;

return input;   
}
*/

float motormap( float input)
{ 
	// this is a thrust to pwm function
	//  float 0 to 1 input and output
	// output can go negative slightly
	// measured eachine motors and prop, stock battery
	// a*x^2 + b*x + c
	// a = 0.262 , b = 0.771 , c = -0.0258

if (input > 1) input = 1;
if (input < 0) input = 0;

input = input*input*0.262f  + input*(0.771f);
input += -0.0258f;

return input;   
}




#define STICKMAX 0.7f
#define STICKCENTER 0.2f

#ifdef GESTURES_USE_YAW

	#define GMACRO_LEFT (rx[2] < - STICKMAX)
	#define GMACRO_RIGHT (rx[2] >  STICKMAX)
	#define GMACRO_XCENTER (fabs(rx[2]) < STICKCENTER)

#else

	#define GMACRO_LEFT (rx[0] < - STICKMAX)
	#define GMACRO_RIGHT (rx[0] >  STICKMAX)
	#define GMACRO_XCENTER (fabs(rx[0]) < STICKCENTER)

#endif

#define GMACRO_DOWN (rx[1] < - STICKMAX)
#define GMACRO_UP (rx[1] >  STICKMAX)

#define GMACRO_PITCHCENTER (fabs(rx[1]) < STICKCENTER)


#define GESTURE_CENTER 0 
#define GESTURE_CENTER_IDLE 12 
#define GESTURE_YAWLEFT 1 
#define GESTURE_YAWRIGHT 2
#define GESTURE_PITCHDOWN 3
#define GESTURE_PITCHUP 4
#define GESTURE_OTHER 127
#define GESTURE_LONG 255

#define GESTURETIME_MIN 100e3 
#define GESTURETIME_MAX 500e3
#define GESTURETIME_IDLE 1000e3

int gesture_start;
int lastgesture;
int setgesture;
static unsigned gesturetime;

int gestures2()
{
	if ( onground )
	{
		if ( GMACRO_XCENTER && GMACRO_PITCHCENTER  )
		{				
		gesture_start = GESTURE_CENTER;
		}
		else
		if ( GMACRO_LEFT && GMACRO_PITCHCENTER  )
		{				
		gesture_start = GESTURE_YAWLEFT;
		}
		else
		if ( GMACRO_RIGHT && GMACRO_PITCHCENTER   )
		{				
		gesture_start = GESTURE_YAWRIGHT;
		}
		else
		if ( GMACRO_DOWN && GMACRO_XCENTER  )
		{				
		gesture_start = GESTURE_PITCHDOWN;
		}
		else
		if ( GMACRO_UP && GMACRO_XCENTER  )
		{				
		gesture_start = GESTURE_PITCHUP;
		}
		else
		{
		//	gesture_start = GESTURE_OTHER;	
		}

		unsigned long time = gettime();
		
		if ( gesture_start!=lastgesture ) 
		{
		gesturetime = time;
		}


				if ( time - gesturetime > GESTURETIME_MIN )
				{
					if ( ( gesture_start == GESTURE_CENTER) && (time - gesturetime > GESTURETIME_IDLE) )
					{		
						setgesture = GESTURE_CENTER_IDLE;										
					}
					else
					if ( time - gesturetime > GESTURETIME_MAX)
					{
						if ( (gesture_start != GESTURE_OTHER)  )
								setgesture = GESTURE_LONG;										
					}
					
					else 
						setgesture = gesture_start;
					
				}
		
				
  lastgesture = gesture_start;				
	

	int gesture_sequence( int gesture);			
	return	gesture_sequence(setgesture);	
				
	}
	else
	{
		setgesture = GESTURE_OTHER;
		lastgesture = GESTURE_OTHER;
	}
	
return 0;
}

#define GSIZE 7


#include <inttypes.h>

uint8_t gbuffer[GSIZE];

// L L D
const uint8_t command1[GSIZE] = {
	GESTURE_CENTER_IDLE , GESTURE_YAWLEFT, GESTURE_CENTER , GESTURE_YAWLEFT, GESTURE_CENTER, GESTURE_PITCHDOWN, GESTURE_CENTER 
																			}	;
// R R D
const uint8_t command2[GSIZE] = {
	GESTURE_CENTER_IDLE , GESTURE_YAWRIGHT, GESTURE_CENTER , GESTURE_YAWRIGHT, GESTURE_CENTER, GESTURE_PITCHDOWN, GESTURE_CENTER 
																			}	;
// D D D
const uint8_t command3[GSIZE] = {
	GESTURE_CENTER_IDLE , GESTURE_PITCHDOWN, GESTURE_CENTER , GESTURE_PITCHDOWN, GESTURE_CENTER, GESTURE_PITCHDOWN, GESTURE_CENTER 
																			}	;
																	
																		
int gesture_sequence( int currentgesture)
{

	if (currentgesture!= gbuffer[0])
	{// add to queue
   int ok;
   
		for (int i = GSIZE ; i >= 1 ; i--)
		{
			gbuffer[i] = gbuffer[i-1];
			 
		}
		gbuffer[0] = currentgesture; 


// check commands
	 ok = 1;		
		
		for (int i = 0 ; i <GSIZE ; i++)
		{
			if (gbuffer[i] != command1[GSIZE - i - 1] )
			{
				 ok = 0;		 
			}
		}
		if ( ok )
		{
			// command 1
			
			//change buffer so it does not trigger again
			gbuffer[1] = GESTURE_OTHER;
			return 1;
		}	 

	 ok = 1;		
		
		for (int i = 0 ; i <GSIZE ; i++)
		{
			if (gbuffer[i] != command2[GSIZE - i - 1] )
			{
				 ok = 0;		 
			}	
		}
		if ( ok )
		{
			// command 2
			
			//change buffer so it does not trigger again
			gbuffer[1] = GESTURE_OTHER;
			return 2;
		}	 
		
		ok = 1;		
		
		for (int i = 0 ; i <GSIZE ; i++)
		{
			if (gbuffer[i] != command3[GSIZE - i - 1] )
			{
				 ok = 0; 
			}
		}	
		if ( ok )
		{
			// command 3
			
			//change buffer so it does not trigger again
			gbuffer[1] = GESTURE_OTHER;
			return 3;
		}	
				
	}
	
return 0;	
}

