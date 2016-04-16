/*
    IMU file of H8 mini firmware
    Copyright (C) 2015  silverx

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// this file is licenced GPL as it contains a cut and paste rotation matrix routine from a gpl source

// library headers
#include <stdbool.h>
#include <inttypes.h>

//#define _USE_MATH_DEFINES
#include <math.h>
#include "drv_time.h"

#include "util.h"
#include "sixaxis.h"

#include <stdlib.h>

// for arm_sin_f32
//   --"-- cos
//#define ARM_MATH_CM3
//#include <arm_math.h>


#define ACC_1G 2048.0

// disable drift correction ( for testing)
#define DISABLE_ACC 0

// filter time in seconds
// time to correct gyro readings using the accelerometer
// 1-4 are generally good
#define FILTERTIME 2.0

// accel magnitude limits for drift correction
#define ACC_MIN 0.7f
#define ACC_MAX 1.3f

// rotation matrix method
// small angle approx = fast, somewhat more inaccurate
#define SMALL_ANGLE_APPROX

// options for non small angle approx
// rotation matrix with float sinf and cosf
// used when SMALL_ANGLE_APPROX in NOT defined
// best (numeric) performance but a lot of flash
// define one set out of the three below
#define _sinf(val) sinf(val)
#define _cosf(val) cosf(val)

// the lib is not included here for now
//#define _sinf(val) arm_sin_f32(val)
//#define _cosf(val) arm_cos_f32(val)

// rotation matrix with approximations for sin and cos
// smaller size then above versions, and faster
//#define _sinf(val) (val)
//#define _cosf(val)  (1 - ((val)*(val))*0.5) 



// Small angle approximation rotation 
// with simple sin and cos
// do not change
#define ssin(val) (val)
#define scos(val) 1.0f

void limit180(float *);


float GEstG[3] = { 0, 0, ACC_1G };

float attitude[3];

extern float gyro[3];
extern float accel[3];
extern float accelcal[3];


void imu_init(void)
{
	// init the gravity vector with accel values
	for (int xx = 0; xx < 100; xx++)
	  {
		  sixaxis_read();

		  for (int x = 0; x < 3; x++)
		    {
			    lpf(&GEstG[x], accel[x], 0.85);
		    }
		  delay(1000);


	  }
}




void vectorcopy(float *vector1, float *vector2);

static unsigned long gptimer;

float atan2approx(float y, float x);

float calcmagnitude(float vector[3])
{
	float accmag = 0;
	for (uint8_t axis = 0; axis < 3; axis++)
	  {
		  accmag += vector[axis] * vector[axis];
	  }
	accmag = sqrtf(accmag);
	return accmag;
}


void vectorcopy(float *vector1, float *vector2)
{
	for (int axis = 0; axis < 3; axis++)
	  {
		  vector1[axis] = vector2[axis];
	  }
}

float normal = ACC_1G;


void imu_calc(void)
{


	float EstG[3];
	float deltatime;	// time in seconds


	vectorcopy(&EstG[0], &GEstG[0]);


	unsigned long time = gettime();
	deltatime = time - gptimer;
	gptimer = time;
	if (deltatime < 1)
		deltatime = 1;
	if (deltatime > 20000)
		deltatime = 20000;
	deltatime = deltatime * 1e-6;	// uS to seconds



	for (int x = 0; x < 3; ++x)
	  {			// was 3
		  accel[x] = accel[x] - accelcal[x];
	  }

#ifndef SMALL_ANGLE_APPROX
	float gyros[3];
	for (int i = 0; i < 3; i++)
	  {
		  gyros[i] = gyro[i] * deltatime;
	  }

	// This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
	float mat[3][3];
	float cosx, sinx, cosy, siny, cosz, sinz;
	float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;
// the signs are differnt due to different conventions
// for positive/negative angles in various multiwii forks this is based on
	cosx = _cosf(gyros[1]);
	sinx = _sinf(gyros[1]);
	cosy = _cosf(-gyros[0]);
	siny = _sinf(-gyros[0]);
	cosz = _cosf(-gyros[2]);
	sinz = _sinf(-gyros[2]);

	coszcosx = cosz * cosx;
	coszcosy = cosz * cosy;
	sinzcosx = sinz * cosx;
	coszsinx = sinx * cosz;
	sinzsinx = sinx * sinz;

	mat[0][0] = coszcosy;
	mat[0][1] = -cosy * sinz;
	mat[0][2] = siny;
	mat[1][0] = sinzcosx + (coszsinx * siny);
	mat[1][1] = coszcosx - (sinzsinx * siny);
	mat[1][2] = -sinx * cosy;
	mat[2][0] = (sinzsinx) - (coszcosx * siny);
	mat[2][1] = (coszsinx) + (sinzcosx * siny);
	mat[2][2] = cosy * cosx;

	EstG[0] = GEstG[0] * mat[0][0] + GEstG[1] * mat[1][0] + GEstG[2] * mat[2][0];
	EstG[1] = GEstG[0] * mat[0][1] + GEstG[1] * mat[1][1] + GEstG[2] * mat[2][1];
	EstG[2] = GEstG[0] * mat[0][2] + GEstG[1] * mat[1][2] + GEstG[2] * mat[2][2];
//      */
#endif				// end rotation matrix

#ifdef SMALL_ANGLE_APPROX

	// Rotate Estimated vector(s), ROLL
	float deltaGyroAngle = (gyro[0]) * deltatime;
	EstG[2] = scos(deltaGyroAngle) * EstG[2] - ssin(deltaGyroAngle) * EstG[0];
	EstG[0] = ssin(deltaGyroAngle) * EstG[2] + scos(deltaGyroAngle) * EstG[0];

	// Rotate Estimated vector(s), PITCH
	deltaGyroAngle = (gyro[1]) * deltatime;
	EstG[1] = scos(deltaGyroAngle) * EstG[1] + ssin(deltaGyroAngle) * EstG[2];
	EstG[2] = -ssin(deltaGyroAngle) * EstG[1] + scos(deltaGyroAngle) * EstG[2];

	// Rotate Estimated vector(s), YAW
	deltaGyroAngle = (gyro[2]) * deltatime;
	EstG[0] = scos(deltaGyroAngle) * EstG[0] - ssin(deltaGyroAngle) * EstG[1];
	EstG[1] = ssin(deltaGyroAngle) * EstG[0] + scos(deltaGyroAngle) * EstG[1];

#endif				// end small angle approx

#ifdef DEBUG
	attitude[2] += RADTODEG * gyro[2] * deltatime;

	limit180(&attitude[2]);
#endif
// orientation vector magnitude


// calc acc mag
	float accmag = 0;

	accmag = calcmagnitude(&accel[0]);

	// normalize acc
	for (int axis = 0; axis < 3; axis++)
	  {
		  accel[axis] = accel[axis] / (accmag / normal);
	  }


	static unsigned int count = 0;

	if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G) && !DISABLE_ACC)
	  {
		  if (count >= 3 || 1)	//
		    {
			    float filtcoeff = lpfcalc(deltatime, FILTERTIME);
			    for (int x = 0; x < 3; x++)
			      {
				      //lpf( &EstG[x] , accel[x] , GYR_CMPF_FACTOR );
				      lpf(&EstG[x], accel[x], filtcoeff);
			      }
		    }
		  count++;
	  }
	else
	  {			// acc mag out of bounds
		  count = 0;
		  if (rand() % 20 == 5)
		    {
			    float mag = 0;
			    mag = calcmagnitude(&EstG[0]);

			    // normalize orientation vector

			    for (int x = 0; x < 3; x++)
			      {
				      EstG[x] = EstG[x] / (mag / normal);
			      }
		    }
	  }

	vectorcopy(&GEstG[0], &EstG[0]);

	attitude[0] = atan2approx(EstG[0], EstG[2]);

	attitude[1] = atan2approx(EstG[1], EstG[2]);

}



#define M_PI  3.14159265358979323846	/* pi */


#define OCTANTIFY(_x, _y, _o)   do {                            \
    float _t;                                                   \
    _o= 0;                                                \
    if(_y<  0)  {            _x= -_x;   _y= -_y; _o += 4; }     \
    if(_x<= 0)  { _t= _x;    _x=  _y;   _y= -_t; _o += 2; }     \
    if(_x<=_y)  { _t= _y-_x; _x= _x+_y; _y=  _t; _o += 1; }     \
} while(0);

// +-0.09 deg error
float atan2approx(float y, float x)
{

	if (x == 0)
		x = 123e-15;
	float phi = 0;
	float dphi;
	float t;

	OCTANTIFY(x, y, phi);

	t = (y / x);
	// atan function for 0 - 1 interval
	dphi = M_PI / 4 * t - t * ((t) - 1) * (0.2447 + 0.0663 * (t));

	phi *= M_PI / 4;
	dphi = phi + dphi;
	if (dphi > M_PI)
		dphi -= 2 * M_PI;
	return 57.29577951 * dphi;
}

void limit180(float *x)
{
	while (*x < -180)
		*x += 360;
	while (*x > 180)
		*x -= 360;
}
