


// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance

#define RGB_LED_NUMBER 0
// pin / port for the RGB led ( programming port ok )
#define RGB_PIN GPIO_PIN_13 // SWDAT
#define RGB_PORT GPIOA


#define FPV_PIN GPIO_PIN_14 // SWCLK
#define FPV_PIN_PORT GPIOA


//#define BUZZER_PIN       GPIO_PIN_13 // SWDAT
#define BUZZER_PIN       GPIO_PIN_14 // SWCLK
#define BUZZER_PIN_PORT  GPIOA
#define BUZZER_DELAY     5e6 // 5 seconds after loss of tx or low bat before buzzer starts

// Analog battery input pin and adc channel

// divider setting for adc uses 2 measurements(readout/value).
// the adc readout can be found in debug mode , debug.adcfilt
// #enable DEBUG should be in config.h
// default for 1/2 divider

// uncomment to enable acd on a pin
// POSSIBLE ADC PINS: PA0-PA7, PB0, PB1
// the associated number is the id passed in to adc_read()

#define ADC_ID_VOLTAGE 5
#define ADC_PA5         ADC_ID_VOLTAGE
#define ADC_PA5_READOUT 2727
#define ADC_PA5_VALUE   3.77f


//*** DO NOT ENABLE ESC DRIVER WITH BRUSHED MOTORS CONNECTED ***
// output driver type , esc settings in drv_esc.c file

#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER
//#define USE_DSHOT_DRIVER

// settings in file drv_servo.c
//#define SERVO_DRIVER

// enable serial out on back-left LED
//#define SERIAL

