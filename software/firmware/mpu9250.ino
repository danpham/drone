/******************************************************************
   1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "math.h"
#include "modules/mpu9250/MPU6500.h"
#include "modules/receiver/receiver.h"
#include "modules/console/console.h"

/******************************************************************
   2. Define declarations (macros then function macros)
******************************************************************/
#define NCS_PIN 4
#define TIMER_FREQ 400
#define SQRT2 1.4142135623731
#define GYRO_INIT_SAMPLE_NB 1000
#define ACCEL_VALUES_NB 1000
#define ACCEL_SCALE_FACTOR 4096
#define GYRO_SCALE_FACTOR 65.5
#define MOVING_AVG_SIZE 6

/******************************************************************
   3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
   4. Variable definitions (static then global)
******************************************************************/
static gyro_t gyro_offsets;

bool gyro_initialized = false;
U16 counter = 0;
gyro_t gyro_sum;

/******************************************************************
   5. Functions prototypes (static only)
******************************************************************/

void reinit_gyro()
{
  gyro_initialized = false;
  counter = 0;
  gyro_sum.x = 0;
  gyro_sum.y = 0;
}

static U8 SPI_read_register(U8 reg) {
  /* reg | 0x80 to denote read */
  U8 read_value = 0;
  SPI.transfer(NCS_PIN, reg | 0x80, SPI_CONTINUE);
  /* write 8-bits zero */
  read_value = SPI.transfer(NCS_PIN, 0x00);

  return read_value;
}

static void SPI_write_register(U8 reg, U8 value) {
  SPI.transfer(NCS_PIN, reg, SPI_CONTINUE);
  SPI.transfer(NCS_PIN, value);
}

void setup_driver()
{
  U8 id = 0;

  SerialUSB.begin(115200);
  /* Wait for USB to be available */
  draw("Wait\nSerial");
  while(!SerialUSB);

  SPI.begin(NCS_PIN);
  SPI.setDataMode(NCS_PIN, SPI_MODE0);
  SPI.setBitOrder(NCS_PIN, MSBFIRST);
  
  /* Access configuration registers at 1MHz */
  SPI.setClockDivider(NCS_PIN, 84);

  SerialUSB.println("MPU driver is starting");

  /* reset the device */
  SPI_write_register(MPU6500_RA_PWR_MGMT_1, 0x80);

  /* page 42 - delay 100ms */
  delay(100);

  /* reset gyro, accel, temp */
  SPI_write_register(MPU6500_RA_SIGNAL_PATH_RESET, 0x05);

  /* page 42 - delay 100ms */
  delay(100);

  /* set SPI mode by setting I2C_IF_DIS */
  SPI_write_register(MPU6500_RA_USER_CTRL, 0x10);
  
  delay(1000);

  /* Enable gyroscope filtering FCHOICE_B[1:0]='11' : 41 Hz */
  SPI_write_register(MPU6500_RA_CONFIG, 0x03);

  /* Enable accelerometer filtering FCHOICE_B[3:2]='11' : 41 Hz */
  SPI_write_register(MPU6500_RA_FF_THR, 0x03);

  /* Gyro config: 500dps */
  SPI_write_register(MPU6500_RA_GYRO_CONFIG, 0x08);
  
  /* Accelerometer config: 8G */
  SPI_write_register(MPU6500_RA_ACCEL_CONFIG, 0x10);

  id = SPI_read_register(MPU6500_RA_GYRO_CONFIG);
  SerialUSB.print("GYRO_FS_SEL: ");
  SerialUSB.println(id, HEX);

  id = SPI_read_register(MPU6500_RA_ACCEL_CONFIG);
  SerialUSB.print("Test ACCEL_FS_SEL: ");
  SerialUSB.println(id, HEX);

  id = SPI_read_register(MPU6500_RA_CONFIG);
  SerialUSB.print("Test MPU6500_RA_CONFIG: ");
  SerialUSB.println(id, HEX);

  id = SPI_read_register(MPU6500_RA_FF_THR);
  SerialUSB.print("Test MPU6500_RA_FF_THR: ");
  SerialUSB.println(id, HEX);

  id = SPI_read_register(MPU6500_RA_USER_CTRL);
  SerialUSB.print("Test MPU6500_RA_USER_CTRL: ");
  SerialUSB.println(id, HEX);

  /* Burst reading at 4Mhz (Arduino Due 84Mhz. Divider: 21 */
  SPI.setClockDivider(NCS_PIN, 21);

  startTimer(TC1, 0, TC3_IRQn, TIMER_FREQ);
}

static short read_short_value(U8 register_high, U8 register_low) {
  short value = 0;
  U8 high = 0;
  U8 low = 0;

  high = SPI_read_register(register_high);
  low = SPI_read_register(register_low);
  value = (high << 8) + low;

  return value;
}

/**
  @desc Read acceleration from sensor
  @return accel_t: return acceleration (no unit)
*/
static accel_t read_accel(void)
{
  accel_t accel_results;

  accel_results.x = (F32)read_short_value(MPU6500_RA_ACCEL_XOUT_H, MPU6500_RA_ACCEL_XOUT_L);
  accel_results.y = (F32)read_short_value(MPU6500_RA_ACCEL_YOUT_H, MPU6500_RA_ACCEL_YOUT_L);
  accel_results.z = (F32)read_short_value(MPU6500_RA_ACCEL_ZOUT_H, MPU6500_RA_ACCEL_ZOUT_L);

  return accel_results;
}

/**
  @desc Read acceleration from sensor
  @return gyro_t: return the angle difference on 3 axis in degrees
*/
static gyro_t read_gyro(void)
{
  gyro_t gyro_results;
  short gyroX = 0;
  short gyroY = 0;
  short gyroZ = 0;

  gyroX = read_short_value(MPU6500_RA_GYRO_XOUT_H, MPU6500_RA_GYRO_XOUT_L);
  gyroY = read_short_value(MPU6500_RA_GYRO_YOUT_H, MPU6500_RA_GYRO_YOUT_L);
  gyroZ = read_short_value(MPU6500_RA_GYRO_ZOUT_H, MPU6500_RA_GYRO_ZOUT_L);

  gyro_results.x = (F32)((F32)gyroX / ((F32)GYRO_SCALE_FACTOR * (F32)TIMER_FREQ));
  gyro_results.y = (F32)((F32)gyroY / ((F32)GYRO_SCALE_FACTOR * (F32)TIMER_FREQ));
  gyro_results.z = (F32)((F32)gyroZ / ((F32)GYRO_SCALE_FACTOR * (F32)TIMER_FREQ));

  return gyro_results;
}

/**
  @desc Compute the offset mean of gyroscope
  @return void
*/
static void init_gyro(void)
{
  gyro_t gyro_results;

  counter++;

  if (1 == counter)
  {
    draw("Gyro\ncalib.");      
  }

  gyro_results = read_gyro();
  gyro_offsets.x += gyro_results.x;
  gyro_offsets.y += gyro_results.y;
  gyro_offsets.z += gyro_results.z;

  if (GYRO_INIT_SAMPLE_NB == counter)
  {
    gyro_offsets.x /= GYRO_INIT_SAMPLE_NB;
    gyro_offsets.y /= GYRO_INIT_SAMPLE_NB;
    gyro_offsets.z /= GYRO_INIT_SAMPLE_NB;

    gyro_initialized = true;
    SerialUSB.println("gyro_offsets.x");
    SerialUSB.println(gyro_offsets.x, 5);
    SerialUSB.println("gyro_offsets.y");
    SerialUSB.println(gyro_offsets.y, 5);
    SerialUSB.println("gyro_offsets.z");
    SerialUSB.println(gyro_offsets.z, 5);
    draw("Drone\nready!");
  }
}

void TC3_Handler()
{
  static U8 outValue = 1;
  gyro_t gyro_results;
  TC_GetStatus(TC1, 0);
  static S16 throttle = 0;
  static angle_errors angleErrors;
  static angle_errors accelAngles;
  static angle_errors accelAnglesFiltered;
  static angle_errors accelAnglesTmp[MOVING_AVG_SIZE];
  static angle_errors gyroAnglesFiltered;
  static angle_errors gyroAnglesTmp[MOVING_AVG_SIZE];
  
  accel_t accel_results;
  short int motor_value_a = 0;
  short int motor_value_b = 0;
  short int motor_value_c = 0;
  short int motor_value_d = 0;

  if (!gyro_initialized)
  {
    init_gyro();
  }
  else
  {
    if (true == pwmNew)
    {
      /* Wrong PWM appears because micros() is unstable */
      if ((pwm_counter > RX_MAX) || (pwm_counter < RX_MIN))
      {
        pwm_counter = RX_MIN;
      }
      throttle = ((pwm_counter >> 5) - 31) * 48;

      pwmNew = false;
    }

    gyro_results = read_gyro();
    gyro_sum.x = (gyro_results.x - gyro_offsets.x);
    gyro_sum.y = (gyro_results.y - gyro_offsets.y);

    accel_results = read_accel();
    accel_results.x /= ACCEL_SCALE_FACTOR;
    accel_results.y /= ACCEL_SCALE_FACTOR;
    accel_results.z /= ACCEL_SCALE_FACTOR;
    
    float module_x = sqrt(pow(accel_results.x, 2) + pow(accel_results.z, 2));

    if (module_x != 0.0)
    {
        accelAngles.x = atan(accel_results.y / module_x) * RAD_TO_DEG;
    }
    else
    {
        if (accel_results.x < 0)
        {
            accelAngles.x = -90.0;  
        }
    }
    
    F32 module_y = sqrt(pow(accel_results.y, 2) + pow(accel_results.z, 2));

    if (module_y != 0.0)
    {
        accelAngles.y = atan(accel_results.x / module_y) * RAD_TO_DEG;
    }
    else
    {
        accelAngles.y = 90.0;
          
        if (accel_results.y < 0)
        {
            accelAngles.y = -90.0;  
        }
    }

    // Shift data
    for (int i = 0; i < (MOVING_AVG_SIZE - 1); i++)
    {
        accelAnglesTmp[i].x = accelAnglesTmp[i+1].x;
        accelAnglesTmp[i].y = accelAnglesTmp[i+1].y;
        gyroAnglesTmp[i].x = gyroAnglesTmp[i+1].x;
        gyroAnglesTmp[i].y = gyroAnglesTmp[i+1].y;
    }
    
    // Add latest data
    accelAnglesTmp[MOVING_AVG_SIZE - 1].x = accelAngles.x;
    accelAnglesTmp[MOVING_AVG_SIZE - 1].y = accelAngles.y;
    gyroAnglesTmp[MOVING_AVG_SIZE - 1].x = gyro_sum.x;
    gyroAnglesTmp[MOVING_AVG_SIZE - 1].y = gyro_sum.y;

    //SerialUSB.print(accelAnglesTmp[MOVING_AVG_SIZE - 1].y, 5);
    //SerialUSB.print("\t");

    // Mean
    accelAnglesFiltered.x = 0;
    accelAnglesFiltered.y = 0; 
    gyroAnglesFiltered.x = 0;
    gyroAnglesFiltered.y = 0;
    for (int i = 0; i < MOVING_AVG_SIZE; i++)
    {
        accelAnglesFiltered.x += accelAnglesTmp[i].x;
        accelAnglesFiltered.y += accelAnglesTmp[i].y;
        gyroAnglesFiltered.x += gyroAnglesTmp[i].x;
        gyroAnglesFiltered.y += gyroAnglesTmp[i].y;
    }

    accelAnglesFiltered.x = accelAnglesFiltered.x / (F32)MOVING_AVG_SIZE;
    accelAnglesFiltered.y = accelAnglesFiltered.y / (F32)MOVING_AVG_SIZE;
    gyroAnglesFiltered.x = gyroAnglesFiltered.x / (F32)MOVING_AVG_SIZE;
    gyroAnglesFiltered.y = gyroAnglesFiltered.y / (F32)MOVING_AVG_SIZE;

    
    /* Apply complementary filter */
#define ALPHA    0.98
    angleErrors.x = ALPHA * (angleErrors.x + gyro_sum.x) + (1 - ALPHA) * accelAngles.x;
    angleErrors.y = ALPHA * (angleErrors.y + gyro_sum.y) + (1 - ALPHA) * accelAngles.y;

    /* Compute new values for motors */
    regulation_loop(angleErrors);

    motor_value_a = quadcopter.motor_1_value + throttle;
    motor_value_b = quadcopter.motor_2_value + throttle;
    motor_value_c = quadcopter.motor_3_value + throttle;
    motor_value_d = quadcopter.motor_4_value + throttle;

    SerialUSB.println(angleErrors.y);
    //setMotorValue(MOTOR_A, motor_value_a, console_getDebugArmedStatus());
    //setMotorValue(MOTOR_C, motor_value_c, console_getDebugArmedStatus());
    setMotorValue(MOTOR_B, motor_value_b, console_getDebugArmedStatus());
    setMotorValue(MOTOR_D, motor_value_d, console_getDebugArmedStatus());
  }
}


static void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  /* Enable or disable write protect of PMC registers */
  pmc_set_writeprotect(false);

  /* Enable the specified peripheral clock */
  pmc_enable_periph_clk((uint32_t)irq);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency;

  TC_SetRA(tc, channel, rc / 2);
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}
