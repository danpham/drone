/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include "motor.h"

/******************************************************************
 * 2. Define declarations (macros then function macros)
******************************************************************/

/******************************************************************
 * 3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
 * 4. Variable definitions (static then global)
******************************************************************/

/******************************************************************
 * 5. Functions prototypes (static only)
******************************************************************/

/**
  @desc Initalize all 4 ESC
  @return void
*/
void setup_esc()
{
  /* Light board's LED when ESC are initialized */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);
  pinMode(MOTOR_C_PIN, OUTPUT);
  pinMode(MOTOR_D_PIN, OUTPUT);

  analogWrite(MOTOR_A_PIN, MOTOR_HI_THROTTLE);
  analogWrite(MOTOR_B_PIN, MOTOR_HI_THROTTLE);
  analogWrite(MOTOR_C_PIN, MOTOR_HI_THROTTLE);
  analogWrite(MOTOR_D_PIN, MOTOR_HI_THROTTLE);

  delay(5000);

  analogWrite(MOTOR_A_PIN, MOTOR_LO_THROTTLE);
  analogWrite(MOTOR_B_PIN, MOTOR_LO_THROTTLE);
  analogWrite(MOTOR_C_PIN, MOTOR_LO_THROTTLE);
  analogWrite(MOTOR_D_PIN, MOTOR_LO_THROTTLE);

  delay(5000);

  digitalWrite(LED_BUILTIN, LOW);
}

/**
  @desc Set motor and check value is in range
  @param motor
  @param value: min value MOTOR_MIN_VALUE, max value MOTOR_MAX_VALUE
  @return void
*/
void setMotorValue(int motor, short value)
{
  if (value < MOTOR_MIN_VALUE){
    analogWrite(motor, MOTOR_MIN_THROTTLE);
  } else if (value > MOTOR_MAX_VALUE){
    analogWrite(motor, MOTOR_MAX_THROTTLE);
  } else {
    analogWrite(motor, MOTOR_MIN_THROTTLE + value);
  }
}
