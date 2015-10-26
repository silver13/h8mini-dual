

#include <stdbool.h>
#include "pid.h"
#include "util.h"
#include "config.h"
#include "defines.h"

// 										ANGLE PIDS				
// Kp											ROLL     PITCH    YAW
float apidkp[PIDNUMBER] = { 2.0e-2 , 2.0e-2  , 0e-1 }; // 2
// angle feedforward
float apidff[PIDNUMBER] = { 0.0e-2 , 0.0e-2  , 0e-1 }; 
// Ki											ROLL     PITCH    YAW
float apidki[PIDNUMBER] = { 1.0e-2  , 1.0e-2 , 0e-1 };	//1
//   											ROLL     PITCH    YAW
// Kd 
float apidkd[PIDNUMBER] = { 3e-2 , 3e-2 , 0e-2 };	 // up to 50e-2 // 0.5e-2
	

// limit of integral term (abs)
#define ITERMLIMIT_FLOAT 1.0	
				
#define OUTLIMIT_FLOAT 1.0
#define OUTLIMIT_FLOATYAW 0.5	

float aierror[PIDNUMBER] = { 0 , 0 , 0};	
float apidoutput[3];

extern int onground;
extern float looptime;
extern float gyro[3];

extern float angleerror[3];
extern float attitude[3];


float apid(int x )
{    

        if (onground ) 
				{
           aierror[x] *= 0.8;
				}
				// anti windup
				// prevent integral increase if output is at max
				int iwindup = 0;
				if (( apidoutput[x] == OUTLIMIT_FLOAT)&& (gyro[x] > 0) )
				{
					iwindup = 1;		
				}
				if (( apidoutput[x] == -OUTLIMIT_FLOAT)&& (gyro[x] < 0) )
				{
					iwindup = 1;				
				}
        if ( !iwindup)
				{
				aierror[x] = aierror[x] + angleerror[x] *  apidki[x] * looptime; 
				}	
         if ( aierror[x]  > ITERMLIMIT_FLOAT) aierror[x] = ITERMLIMIT_FLOAT;
				 if ( aierror[x]  < -ITERMLIMIT_FLOAT) aierror[x] = -ITERMLIMIT_FLOAT;
 
				// P term
          apidoutput[x] = angleerror[x] * apidkp[x] ;
				// FF term
          apidoutput[x] += attitude[x] * apidff[x] ;
			
				// I term	
					apidoutput[x] += aierror[x]  ;
				
					// D term
					//apidoutput[x] = apidoutput[x] - (gyro[x]) * apidkd[x]; 
					apidoutput[x] = apidoutput[x] - (gyro[x]) * apidkd[x]; 
				
				 limitf(  &apidoutput[x] , OUTLIMIT_FLOAT);
						

return apidoutput[x];		 		
}

