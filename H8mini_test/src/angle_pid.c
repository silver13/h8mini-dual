

#include <stdbool.h>
#include "pid.h"
#include "util.h"
#include "config.h"
#include "defines.h"

#define APIDNUMBER 3

//                   						 ANGLE PIDS      
// yaw is done by the rate yaw pid
// Kp                       ROLL     PITCH    YAW
float apidkp[APIDNUMBER] = { 2.2e-2, 2.2e-2, 0e-1 };

// Ki                        ROLL     PITCH    YAW
float apidki[APIDNUMBER] = { 1.0e-2, 1.0e-2, 0e-1 };   



// limit of integral term (abs)
#define ITERMLIMIT_FLOAT 1.0f

#define OUTLIMIT_FLOAT 1.0f

extern float attitude[3];
extern int onground;
extern float looptime;
extern float gyro[3];

float aierror[APIDNUMBER] = { 0, 0, 0 };
float apidoutput[APIDNUMBER];
float angleerror[3];

float apid(int x)
{

	if (onground)
	  {
		  aierror[x] *= 0.8f;
	  }
	// anti windup
	// prevent integral increase if output is at max
	int iwindup = 0;
	if ((apidoutput[x] == OUTLIMIT_FLOAT) && (gyro[x] > 0))
	  {
		  iwindup = 1;
	  }
	if ((apidoutput[x] == -OUTLIMIT_FLOAT) && (gyro[x] < 0))
	  {
		  iwindup = 1;
	  }
	if (!iwindup)
	  {
		  aierror[x] = aierror[x] + angleerror[x] * apidki[x] * looptime;
	  }

	limitf(&aierror[x], ITERMLIMIT_FLOAT);

	// P term
	apidoutput[x] = angleerror[x] * apidkp[x];

	// I term       
	apidoutput[x] += aierror[x];

		
	limitf(&apidoutput[x], OUTLIMIT_FLOAT);


	return apidoutput[x];
}
