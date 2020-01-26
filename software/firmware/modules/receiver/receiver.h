
/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/

/******************************************************************
 * 2. Define declarations (macros then function macros)
******************************************************************/
#define RX_MIN               1000
#define RX_MAX               2000
#define RX_RANGE    RX_MAX-RX_MIN

extern void Rx_setup();
extern void Rx_getThrottle();
extern bool pwmNew;
extern unsigned long pwm_counter;
