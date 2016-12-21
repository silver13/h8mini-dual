


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



//*** DO NOT ENABLE ESC DRIVER WITH BRUSHED MOTORS CONNECTED ***
// output driver type , esc settings in drv_esc.c file

#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER
//#define USE_DSHOT_DRIVER

