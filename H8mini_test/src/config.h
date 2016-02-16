
#include "defines.h"

// rate pids in pid.c
// angle pids in apid.h ( they control the rate pids)
// yaw is the same for both modes

// not including the "f" after float numbers will give a warning
// it will still work



// rate in deg/sec
// for low rates ( acro mode)
#define MAX_RATE 180.0f
#define MAX_RATEYAW 180.0f

// multiplier for high rates
// devo/module uses high rates only
#define HIRATEMULTI 2.0f
#define HIRATEMULTIYAW 2.0f

// max angle for level mode (in degrees)
// low and high rates(angle?)
#define MAX_ANGLE_LO 35.0f
#define MAX_ANGLE_HI 55.0f

// max rate for rate pid in level mode
// this should usually not change unless faster / slower response is desired.
#define LEVEL_MAX_RATE_LO 360.0f
#define LEVEL_MAX_RATE_HI 360.0f



// disable inbuilt expo functions
#define DISABLE_EXPO

// use if your tx has no expo function
// also comment out DISABLE_EXPO to use
// -1 to 1 , 0 = no exp
// positive = less sensitive near center 
#define EXPO_XY 0.3f
#define EXPO_YAW 0.0f




// Hardware gyro LPF filter frequency
// gyro filter 0 = 260hz
// gyro filter 1 = 184hz
// gyro filter 2 = 94hz
// gyro filter 3 = 42hz
// 4 , 5, 6
#define GYRO_LOW_PASS_FILTER 3

// software gyro lpf ( iir )
// set only one below
//#define SOFT_LPF_1ST_023HZ
//#define SOFT_LPF_1ST_043HZ
//#define SOFT_LPF_1ST_100HZ
//#define SOFT_LPF_2ND_043HZ
//#define SOFT_LPF_2ND_088HZ
//#define SOFT_LPF_4TH_088HZ
//#define SOFT_LPF_4TH_160HZ
//#define SOFT_LPF_4TH_250HZ
#define SOFT_LPF_NONE

// this works only on newer boards (non mpu-6050)
// on older boards the hw gyro setting controls the acc as well
#define ACC_LOW_PASS_FILTER 5




// Headless mode
// Only in acro mode
// 0 - flip 
// 1 - expert
// 2 - headfree
// 3 - headingreturn
// 4 - AUX1 ( gestures <<v and >>v)
// 5 - AUX2+ (  none    )
// 6 - Pitch trims
// 7 - Roll trims
// 8 - Throttle trims
// 9 - Yaw trims
// 10 - on always
// 11 - off always
// CH_ON , CH_OFF , CH_FLIP , CH_EXPERT
// CH_HEADFREE , CH_RTH , CH_AUX1 , CH_AUX2 , CH_AUX3 , CH_AUX4
// CH_PIT_TRIM, CH_RLL_TRIM, CH_THR_TRIM, CH_YAW_TRIM
#define HEADLESSMODE CH_OFF


// rates / expert mode
// 0 - flip 
// 1 - expert
// 2 - headfree
// 3 - headingreturn
// 4 - AUX1 ( gestures <<v and >>v)
// 5 - AUX2+ (  none    )
// 6 - Pitch trims
// 7 - Roll trims
// 8 - Throttle trims
// 9 - Yaw trims
// 10 - on always
// 11 - off always
// CH_ON , CH_OFF , CH_FLIP , CH_EXPERT
// CH_HEADFREE , CH_RTH , CH_AUX1 , CH_AUX2 , CH_AUX3 , CH_AUX4
// CH_PIT_TRIM, CH_RLL_TRIM, CH_THR_TRIM, CH_YAW_TRIM
#define RATES 1


// level / acro mode switch
// CH_AUX1 = gestures
// 0 - flip 
// 1 - expert
// 2 - headfree
// 3 - headingreturn
// 4 - AUX1 ( gestures <<v and >>v)
// 5 - AUX2+ (  none    )
// 6 - Pitch trims
// 7 - Roll trims
// 8 - Throttle trims
// 9 - Yaw trims
// 10 - on always
// 11 - off always
// CH_ON , CH_OFF , CH_FLIP , CH_EXPERT
// CH_HEADFREE , CH_RTH , CH_AUX1 , CH_AUX2 , CH_AUX3 , CH_AUX4
// CH_PIT_TRIM, CH_RLL_TRIM, CH_THR_TRIM, CH_YAW_TRIM
#define LEVELMODE CH_AUX1



// aux1 channel starts on if this is defined, otherwise off.
#define AUX1_START_ON

// use yaw/pitch instead of roll/pitch for gestures
//#define GESTURES_USE_YAW




// throttle angle compensation in level mode
// comment out to disable
//#define AUTO_THROTTLE

// enable auto throttle  in acro mode if enabled above
// should be used if no flipping is performed
// 0 / 1 ( off / on )
#define AUTO_THROTTLE_ACRO_MODE 0


// enable auto lower throttle near max throttle to keep control
// comment out to disable
//#define MIX_LOWER_THROTTLE

// options for mix throttle lowering if enabled
// 0 - 100 range ( 100 = full reduction / 0 = no reduction )
#define MIX_THROTTLE_REDUCTION_PERCENT 100
// lpf (exponential) shape if on, othewise linear
//#define MIX_THROTTLE_FILTER_LPF



// battery saver ( only at powerup )
// does not start software if battery is too low
// flashes 2 times repeatedly at startup
#define STOP_LOWBATTERY

// under this voltage the software will not start 
// if STOP_LOWBATTERY is defined above
#define STOP_LOWBATTERY_TRESH 3.3f

// voltage too start warning
// volts
#define VBATTLOW 3.5f

// compensation for battery voltage vs throttle drop
// increase if battery low comes on at max throttle
// decrease if battery low warning goes away at high throttle
// in volts
#define VDROP_FACTOR 0.60f

// voltage hysteresys
// in volts
#define HYST 0.10f




// enable motor filter
// hanning 3 sample fir filter
#define MOTOR_FILTER


// clip feedforward attempts to resolve issues that occur near full throttle
//#define CLIP_FF

// motor transient correction applied	to throttle stick
//#define THROTTLE_TRANSIENT_COMPENSATION



// motor curve to use
// the pwm frequency has to be set independently
#define MOTOR_CURVE_NONE
//#define MOTOR_CURVE_6MM_490HZ
//#define MOTOR_CURVE_85MM_8KHZ
//#define MOTOR_CURVE_85MM_32KHZ

// pwm frequency for motor control
// a higher frequency makes the motors more linear
//#define PWM_490HZ
//#define PWM_8KHZ
#define PWM_16KHZ
//#define PWM_24KHZ
//#define PWM_32KHZ

// failsafe time in uS
#define FAILSAFETIME 1000000  // one second



// ########################################
// things that are experimental / old / etc
// do not change things below

// invert yaw pid for hubsan motors
//#define INVERT_YAW_PID

//some debug stuff
//#define DEBUG

// disable motors for testing
//#define NOMOTORS

// enable serial out on back-left LED
//#define SERIAL


// enable motors if pitch / roll controls off center (at zero throttle)
// possible values: 0 / 1
#define ENABLESTIX 0

// only for compilers other than gcc
#ifndef __GNUC__

#pragma diag_warning 1035 , 177 , 4017

#pragma diag_error 260 

#endif
// --fpmode=fast ON





















