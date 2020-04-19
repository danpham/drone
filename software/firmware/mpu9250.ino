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
#define TIMER_FREQ 150

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
  /* 1MHz */
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

  /* set SPI mode by setting I2C_IF_DIS
    reset DMP, FIFO, SIG */
  SPI_write_register(MPU6500_RA_USER_CTRL, 0x10 | 0x8 | 0x4 | 0x1);

  delay(1000);

  id = SPI_read_register(MPU6500_RA_WHO_AM_I);
  switch (id) {
    case 0x68:
      SerialUSB.println("Sensor is MPU6050");
      break;
    case 0x70:
      SerialUSB.println("Sensor is MPU6500");
      break;
    case 0x71:
      SerialUSB.println("Sensor is MPU9250");
      break;
    case 0x73:
      SerialUSB.println("Sensor is MPU9255");
      break;
    case 0x74:
      SerialUSB.println("Sensor is MPU6515");
      break;
    default:
      SerialUSB.print("Unknown sensor: ");
      SerialUSB.println(id, HEX);
  }

  /* Gyro range selection: 0x00 250 degrees per second */
  id = SPI_read_register(MPU6500_RA_GYRO_CONFIG);
  SerialUSB.print("GYRO_FS_SEL: ");
  SerialUSB.println(id, HEX);

  id = SPI_read_register(MPU6500_RA_ACCEL_CONFIG);
  SerialUSB.print("Test ACCEL_FS_SEL: ");
  SerialUSB.println(id, HEX);

  /* Gyroscope filter
    Frequency = 1KHz, bandwith = 92Hz (0x02) */
  SPI_write_register(MPU6500_RA_CONFIG, 0x02);

  id = SPI_read_register(MPU6500_RA_CONFIG);
  SerialUSB.print("Test MPU6500_RA_CONFIG: ");
  SerialUSB.println(id, HEX);

  /* Accelerometer filter
    Frequency = 1KHz, bandwith = 99Hz (0x02) */
  SPI_write_register(MPU6500_RA_FF_THR, 0x02);
  id = SPI_read_register(MPU6500_RA_FF_THR);
  SerialUSB.print("Test MPU6500_RA_FF_THR: ");
  SerialUSB.println(id, HEX);

  /* set SPI mode by setting I2C_IF_DIS */
  SPI_write_register(MPU6500_RA_USER_CTRL, 0x10);

  /* speed up to take data
    10.5MHz; 7 sometimes works */
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

  accel_results.x = (float)read_short_value(MPU6500_RA_ACCEL_XOUT_H, MPU6500_RA_ACCEL_XOUT_L);
  accel_results.y = (float)read_short_value(MPU6500_RA_ACCEL_YOUT_H, MPU6500_RA_ACCEL_YOUT_L);
  accel_results.z = (float)read_short_value(MPU6500_RA_ACCEL_ZOUT_H, MPU6500_RA_ACCEL_ZOUT_L);

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

  gyro_results.x = (float)((float)gyroX / (131 * TIMER_FREQ));
  gyro_results.y = (float)((float)gyroY / (131 * TIMER_FREQ));
  gyro_results.z = (float)((float)gyroZ / (131 * TIMER_FREQ));

  return gyro_results;
}

/**
  @desc Compute the offset mean of gyroscopes and accelerometers
  @return void
*/
static void init_gyro(void)
{
  gyro_t gyro_results;

  if (0 == counter)
  {
    draw("Gyro offs");      
  }

  gyro_results = read_gyro();
  gyro_offsets.x += gyro_results.x;
  gyro_offsets.y += gyro_results.y;
  gyro_offsets.z += gyro_results.z;

  if (999 == counter)
  {
    gyro_offsets.x /= 1000;
    gyro_offsets.y /= 1000;
    gyro_offsets.z /= 1000;

    gyro_initialized = true;
    SerialUSB.println("gyro_offsets.x");
    SerialUSB.println(gyro_offsets.x,5);
    SerialUSB.println("gyro_offsets.y");
    SerialUSB.println(gyro_offsets.y,5);
    SerialUSB.println("gyro_offsets.z");
    SerialUSB.println(gyro_offsets.z,5);
    draw("Drone\nready!");
  }
  counter++;
}

void TC3_Handler() {
  gyro_t gyro_results;
  TC_GetStatus(TC1, 0);
  static unsigned long pwm_value = 0;
  static angle_errors angleErrors;
  static angle_errors accelAngles;

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
      pwm_value = ((pwm_counter >> 5) - 31) * 3;

      pwmNew = false;
    }

    gyro_results = read_gyro();
    gyro_sum.x += (gyro_results.x - gyro_offsets.x);
    gyro_sum.y += (gyro_results.y - gyro_offsets.y);

    accel_results = read_accel();
    accel_results.x /= 16384;
    accel_results.y /= 16384;
    accel_results.z /= 16384;
    
    float module_x = sqrt(accel_results.x * accel_results.x + accel_results.z * accel_results.z);

    if (module_x != 0.0)
    {
        accelAngles.x = atan(accel_results.y / module_x) * RAD_TO_DEG;
    }
    else
    {
        if (accel_results.x > 0)
        {
            accelAngles.x = 90.0;  
        }
        else
        {
            accelAngles.x = -90.0;  
        }
        
    }
    
    float module_y = sqrt(accel_results.y * accel_results.y + accel_results.z * accel_results.z);

    if (module_y != 0.0)
    {
        accelAngles.y = atan(accel_results.x / module_y) * RAD_TO_DEG;
    }
    else
    {
        if (accel_results.y > 0)
        {
            accelAngles.y = 90.0;  
        }
        else
        {
            accelAngles.y = -90.0;  
        }
        
    }
  
    gyro_sum.x = 0.98 * gyro_sum.x + 0.02 * accelAngles.x;
    gyro_sum.y = 0.98 * gyro_sum.y + 0.02 * accelAngles.y;

    angleErrors.x = gyro_sum.x;
    angleErrors.y = gyro_sum.y;

    /* Compute new values for motors */
    regulation_loop(angleErrors);

    motor_value_a = quadcopter.motor_1_value + pwm_value * 2;
    motor_value_b = quadcopter.motor_2_value + pwm_value * 2;
    motor_value_c = quadcopter.motor_3_value + pwm_value * 2;
    motor_value_d = quadcopter.motor_4_value + pwm_value * 2;
          
    if (console_getDebugInfoStatus())
    {        
        SerialUSB.print("x;\t");
        SerialUSB.print(angleErrors.x);
        SerialUSB.print(";\ty;\t");
        SerialUSB.print(angleErrors.y);
        SerialUSB.print(";\ta;\t");
        SerialUSB.print(motor_value_a);
        SerialUSB.print(";\tb;\t");
        SerialUSB.print(motor_value_b);
        SerialUSB.print(";\tc;\t");
        SerialUSB.print(motor_value_c);
        SerialUSB.print(";\td;\t");
        SerialUSB.println(motor_value_d);
    }

    setMotorValue(MOTOR_A, motor_value_a, console_getDebugArmedStatus());
    setMotorValue(MOTOR_C, motor_value_c, console_getDebugArmedStatus());
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
