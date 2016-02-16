

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
#define GESTURE_LEFT 1
#define GESTURE_RIGHT 2
#define GESTURE_DOWN 3
#define GESTURE_UP 4
#define GESTURE_OTHER 127
#define GESTURE_LONG 255

#define GESTURETIME_MIN 100e3
#define GESTURETIME_MAX 500e3
#define GESTURETIME_IDLE 1000e3

#include <inttypes.h>
#include <math.h>
#include "drv_time.h"
#include "gestures.h"


int gesture_start;
int lastgesture;
int setgesture;
static unsigned gesturetime;

extern int onground;
extern float rx[];


int gestures2()
{
	if (onground)
	  {
		  if (GMACRO_XCENTER && GMACRO_PITCHCENTER)
		    {
			    gesture_start = GESTURE_CENTER;
		    }
		  else if (GMACRO_LEFT && GMACRO_PITCHCENTER)
		    {
			    gesture_start = GESTURE_LEFT;
		    }
		  else if (GMACRO_RIGHT && GMACRO_PITCHCENTER)
		    {
			    gesture_start = GESTURE_RIGHT;
		    }
		  else if (GMACRO_DOWN && GMACRO_XCENTER)
		    {
			    gesture_start = GESTURE_DOWN;
		    }
		  else if (GMACRO_UP && GMACRO_XCENTER)
		    {
			    gesture_start = GESTURE_UP;
		    }
		  else
		    {
			    //      gesture_start = GESTURE_OTHER;  
		    }

		  unsigned long time = gettime();

		  if (gesture_start != lastgesture)
		    {
			    gesturetime = time;
		    }


		  if (time - gesturetime > GESTURETIME_MIN)
		    {
			    if ((gesture_start == GESTURE_CENTER) && (time - gesturetime > GESTURETIME_IDLE))
			      {
				      setgesture = GESTURE_CENTER_IDLE;
			      }
			    else if (time - gesturetime > GESTURETIME_MAX)
			      {
				      if ((gesture_start != GESTURE_OTHER))
					      setgesture = GESTURE_LONG;
			      }

			    else
				    setgesture = gesture_start;

		    }


		  lastgesture = gesture_start;



		  return gesture_sequence(setgesture);

	  }
	else
	  {
		  setgesture = GESTURE_OTHER;
		  lastgesture = GESTURE_OTHER;
	  }

	return 0;
}

#define GSIZE 7


uint8_t gbuffer[GSIZE];

// L L D
const uint8_t command1[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};

// R R D
const uint8_t command2[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};

// D D D
const uint8_t command3[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};


int gesture_sequence(int currentgesture)
{

	if (currentgesture != gbuffer[0])
	  {			// add to queue
		  int ok;

		  for (int i = GSIZE; i >= 1; i--)
		    {
			    gbuffer[i] = gbuffer[i - 1];

		    }
		  gbuffer[0] = currentgesture;


// check commands
		  ok = 1;

		  for (int i = 0; i < GSIZE; i++)
		    {
			    if (gbuffer[i] != command1[GSIZE - i - 1])
			      {
				      ok = 0;
			      }
		    }
		  if (ok)
		    {
			    // command 1

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return 1;
		    }

		  ok = 1;

		  for (int i = 0; i < GSIZE; i++)
		    {
			    if (gbuffer[i] != command2[GSIZE - i - 1])
			      {
				      ok = 0;
			      }
		    }
		  if (ok)
		    {
			    // command 2

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return 2;
		    }

		  ok = 1;

		  for (int i = 0; i < GSIZE; i++)
		    {
			    if (gbuffer[i] != command3[GSIZE - i - 1])
			      {
				      ok = 0;
			      }
		    }
		  if (ok)
		    {
			    // command 3

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return 3;
		    }

	  }

	return 0;
}
