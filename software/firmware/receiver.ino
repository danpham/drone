/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/

/******************************************************************
 * 2. Define declarations (macros then function macros)
******************************************************************/
#define THROTTLE_PIN 22

/******************************************************************
 * 3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
 * 4. Variable definitions (static then global)
******************************************************************/
bool pwmNew = false;
unsigned long pwm_counter = 0;

/******************************************************************
 * 5. Functions prototypes (static only)
******************************************************************/

void Rx_setup()
{
  pinMode(THROTTLE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), Rx_getThrottle, CHANGE);
}

void Rx_getThrottle()
{
  static unsigned long low_counter = 0;
  static bool last_was_low = false;
  unsigned long counter = 0;
  
  char val = digitalRead(THROTTLE_PIN);

  if (val == 1) {
    last_was_low = true;
    low_counter = micros();
  }

  if ((val == 0) && (last_was_low == true)){
    last_was_low = false;

    /* Counter overflow */
    counter = micros();
    if (low_counter > counter){
       pwm_counter = 0xFFFFFFFF - low_counter + counter;
    } else {
       pwm_counter = counter - low_counter;
    } 
    pwmNew = true;
  }
}
