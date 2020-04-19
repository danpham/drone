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
static Motor motorConfig[MOTOR_MAX];

/******************************************************************
 * 5. Functions prototypes (static only)
******************************************************************/

/**
  @desc Setup motor configuration
  @return void
*/
void setup_motor(void)
{
    motorConfig[MOTOR_A].motor_pin = MOTOR_A_PIN;
    motorConfig[MOTOR_A].motor_gain = 1.0;
    motorConfig[MOTOR_A].motor_offset = 384;

    motorConfig[MOTOR_B].motor_pin = MOTOR_B_PIN;
    motorConfig[MOTOR_B].motor_gain = 1.0;
    motorConfig[MOTOR_B].motor_offset = 384;

    motorConfig[MOTOR_C].motor_pin = MOTOR_C_PIN;
    motorConfig[MOTOR_C].motor_gain = 1.0;
    motorConfig[MOTOR_C].motor_offset = 384;

    motorConfig[MOTOR_D].motor_pin = MOTOR_D_PIN;
    motorConfig[MOTOR_D].motor_gain = 1.0;
    motorConfig[MOTOR_D].motor_offset = 384;
}

/**
  @desc Set motor value
  @param motorId: identifier of the motor
  @param value: min value MOTOR_MIN_VALUE, max value MOTOR_MAX_VALUE
  @param isArmed: if motor is not armed, it won't turn
  @return void
*/
void setMotorValue(const U8 motorId, const S16 value, const bool isArmed)
{
    S16 valueWithOffset = 0;

    if (motorId < MOTOR_MAX)
    {
        if (isArmed)
        {         
          valueWithOffset = value + (S16)motorConfig[motorId].motor_offset;

          if (valueWithOffset > MOTOR_MAX_VALUE)
          {
              analogWrite(motorConfig[motorId].motor_pin, MOTOR_MAX_THROTTLE);
          }
          else if (valueWithOffset < motorConfig[motorId].motor_offset)
          {
              analogWrite(motorConfig[motorId].motor_pin, (MOTOR_MIN_THROTTLE + motorConfig[motorId].motor_offset));
          }
          else
          {
              analogWrite(motorConfig[motorId].motor_pin, (MOTOR_MIN_THROTTLE + valueWithOffset));
          }
        }
        else
        {
            analogWrite(motorConfig[motorId].motor_pin, MOTOR_MIN_THROTTLE);
        }

    }
}

/**
  @desc Set motor gain
  @param motorId: identifier of the motor
  @param value: gain for selected motor
  @return void
*/
void setMotorGain(const U8 motorId, const float value)
{
    if (motorId < MOTOR_MAX)
    {
       motorConfig[motorId].motor_gain = value;
    }
}

/**
  @desc Get motor gain
  @param motorId: identifier of the motor
  @return float
*/
float getMotorGain(const U8 motorId)
{
    float motorGain = 0;

    if (motorId < MOTOR_MAX)
    {
       motorGain = motorConfig[motorId].motor_gain;
    }

    return motorGain;
}

/**
  @desc Set motor offset
  @param motorId: identifier of the motor
  @param value: offset for selected motor
  @return void
*/
void setMotorOffset(const U8 motorId, const U16 offset)
{
    if (motorId < MOTOR_MAX)
    {
       motorConfig[motorId].motor_offset = offset;
    }
}

/**
  @desc Get motor offset
  @param motorId: identifier of the motor
  @return U16
*/
U16 getMotorOffset(const U8 motorId)
{
    U16 motorOffset = 0;

    if (motorId < MOTOR_MAX)
    {
       motorOffset = motorConfig[motorId].motor_offset;
    }

    return motorOffset;
}

/**
  @desc Display motor data
  @param motorId: identifier of the motor
  @return void
*/
void printMotorData(void)
{
    SerialUSB.print("Motor A\t Gain:\t");
    SerialUSB.print(getMotorGain(MOTOR_A));
    SerialUSB.print("\tOffset:\t");
    SerialUSB.println(getMotorOffset(MOTOR_A));

    SerialUSB.print("Motor B\t Gain:\t");
    SerialUSB.print(getMotorGain(MOTOR_B));
    SerialUSB.print("\tOffset:\t");
    SerialUSB.println(getMotorOffset(MOTOR_B));

    SerialUSB.print("Motor C\t Gain:\t");
    SerialUSB.print(getMotorGain(MOTOR_C));
    SerialUSB.print("\tOffset:\t");
    SerialUSB.println(getMotorOffset(MOTOR_C));

    SerialUSB.print("Motor D\t Gain:\t");
    SerialUSB.print(getMotorGain(MOTOR_D));
    SerialUSB.print("\tOffset:\t");
    SerialUSB.println(getMotorOffset(MOTOR_D));
}
