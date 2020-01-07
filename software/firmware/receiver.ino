bool pwmNew = false;
unsigned long pwm_counter = 0;

#define THROTTLE_PIN 22


void setup_receiver()
{
  // Met la broche de signal en entrée
  pinMode(12, INPUT);

  pinMode(THROTTLE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), getThrottle, CHANGE);
}


void getThrottle() {
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
