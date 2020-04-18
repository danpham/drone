/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/


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

  draw("Esc init.\nHi throt.");
  delay(5000);

  analogWrite(MOTOR_A_PIN, MOTOR_LO_THROTTLE);
  analogWrite(MOTOR_B_PIN, MOTOR_LO_THROTTLE);
  analogWrite(MOTOR_C_PIN, MOTOR_LO_THROTTLE);
  analogWrite(MOTOR_D_PIN, MOTOR_LO_THROTTLE);

  draw("Esc init.\nLow throt.");
  delay(5000);

  draw("Esc init.\nOK");
  digitalWrite(LED_BUILTIN, LOW);
}
