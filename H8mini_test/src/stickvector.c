#include "config.h"
#include "util.h"

#include <math.h>
#include <string.h>


extern float Q_rsqrt( float number );

void vector_cross(float vout[3], float v1[3],float v2[3])
{
  vout[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vout[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vout[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

extern float rx[];


float stickvector[3];
float errorvect[3];

extern float GEstG[3];

void stick_vector( float maxangle)
{
	
	// start with a down vector
	stickvector[0] = 0;
	stickvector[1] = 0;
	stickvector[2] = 1;

//float yaw = ( rx[1] * MAX_ANGLE_HI * DEGTORAD);
//float roll = ( rx[0] * MAX_ANGLE_HI * DEGTORAD);

//downvect[0] = (sin(pitch)*cos(yaw));
//downvect[1] =  sin(yaw);
//downvect[2] = (cos(pitch)*cos(yaw));

// rotate the down vector to match stick(s) position
// this is slightly distorted in the corners
stickvector[1] = fastsin(rx[1] * maxangle * DEGTORAD);
stickvector[0] = fastsin(rx[0] * maxangle * DEGTORAD);

float mag2 = stickvector[0] * stickvector[0] + stickvector[1] * stickvector[1];

// fixup for the cheap maths used
if ( mag2 > 1)
{
mag2 = Q_rsqrt(mag2);
stickvector[0] *=mag2;
stickvector[1] *=mag2;
stickvector[2] = 0;
}else
{
 mag2 = Q_rsqrt(1 - mag2);
stickvector[2] = 1/mag2; 
}

//memcpy ( &stickvector[0] , &downvect[0] , sizeof(float) * 3 );

// this will be the quads orientation vector
float gestg_normal[3];

//normalize GEstG to 1.0
gestg_normal[0] = GEstG[0] / 2048 ;
gestg_normal[1] = GEstG[1] / 2048 ;
gestg_normal[2] = GEstG[2] / 2048 ;

float tempvect[3];

// find error between vectors
vector_cross( &tempvect[0] , &gestg_normal[0] , &stickvector[0]);

limitf( & tempvect[0] , 1.0);
limitf( & tempvect[1] , 1.0);

// this is the correct way
//errorvect[0] = asin(tempvect[1]);
//errorvect[1] = asin(- tempvect[0]);

// remove asin since the error is high only at high error angles ( > 50 deg )
errorvect[0] = (tempvect[1]);
errorvect[1] = (- tempvect[0]);
}



