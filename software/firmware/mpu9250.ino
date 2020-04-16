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
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define NCS_PIN 4
#define TIMER_FREQ 250

/******************************************************************
   3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
   4. Variable definitions (static then global)
******************************************************************/
gyro_t gyro_offsets;
accel_t accel_offsets;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static bool gyro_initialized = false;

/******************************************************************
   5. Functions prototypes (static only)
******************************************************************/

static char SPI_read_register(char reg) {
  /* reg | 0x80 to denote read */
  char read_value = 0;
  SPI.transfer(NCS_PIN, reg | 0x80, SPI_CONTINUE);
  /* write 8-bits zero */
  read_value = SPI.transfer(NCS_PIN, 0x00);

  return read_value;
}

static void SPI_write_register(char reg, char value) {
  SPI.transfer(NCS_PIN, reg, SPI_CONTINUE);
  SPI.transfer(NCS_PIN, value);
}

void setup_driver() {
  char id = 0;

  /* SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    Address 0x3C for 128x64 */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  SerialUSB.begin(115200);
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

static short read_short_value(char register_high, char register_low) {
  short value = 0;
  char high = 0;
  char low = 0;

  high = SPI_read_register(register_high);
  low = SPI_read_register(register_low);
  value = (high << 8) + low;

  return value;
}

static accel_t read_accel(void) {
  short accelX = 0;
  short accelY = 0;
  short accelZ = 0;
  float real_accelX = 0;
  float real_accelY = 0;
  float real_accelZ = 0;
  accel_t accel_results_degrees;
  accel_t accel_results;

  accelX = read_short_value(MPU6500_RA_ACCEL_XOUT_H, MPU6500_RA_ACCEL_XOUT_L);
  accelY = read_short_value(MPU6500_RA_ACCEL_YOUT_H, MPU6500_RA_ACCEL_YOUT_L);
  accelZ = read_short_value(MPU6500_RA_ACCEL_ZOUT_H, MPU6500_RA_ACCEL_ZOUT_L);

  /* -1 to reverse axis from drawing xyz in PCB */
  real_accelX = (float)accelX;
  real_accelY = (float)accelY;
  real_accelZ = (float)accelZ;

  accel_results.x = atan2f(real_accelY, real_accelZ);
  accel_results.y = atan2f(real_accelX, real_accelZ);
  accel_results.z = atan2f(real_accelY, real_accelX);

  /* Convert in degrees */
  accel_results_degrees.x = -accel_results.x * RAD_TO_DEG;
  accel_results_degrees.y = accel_results.y * RAD_TO_DEG;
  accel_results_degrees.z = -accel_results.z * RAD_TO_DEG;

  return accel_results_degrees;
}

static gyro_t read_gyro(void) {
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

static void init_gyro_accel(void) {
  static int counter = 0;
  gyro_t gyro_results;
  accel_t accel_results_degrees;

  counter++;
  gyro_results = read_gyro();
  gyro_offsets.x += gyro_results.x;
  gyro_offsets.y += gyro_results.y;
  gyro_offsets.z += gyro_results.z;

  accel_results_degrees = read_accel();
  accel_offsets.x += accel_results_degrees.x;
  accel_offsets.y += accel_results_degrees.y;
  accel_offsets.z += accel_results_degrees.z;

  if (counter == 100) {
    gyro_offsets.x /= 100;
    gyro_offsets.y /= 100;
    gyro_offsets.z /= 100;
    accel_offsets.x /= 100;
    accel_offsets.y /= 100;
    accel_offsets.z /= 100;

    gyro_initialized = true;
  }
}

void TC3_Handler() {
  gyro_t gyro_results;
  TC_GetStatus(TC1, 0);
  static unsigned long pwm_value = 0;
  static angle_errors angleErrors;
  static gyro_t gyro_sum;
  accel_t accel_results_degrees;
  short int motor_value_a = 0;
  short int motor_value_b = 0;
  short int motor_value_c = 0;
  short int motor_value_d = 0;

  if (!gyro_initialized)
  {
    init_gyro_accel();
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
    gyro_sum.x += gyro_results.x - gyro_offsets.x;
    gyro_sum.y -= gyro_results.y - gyro_offsets.y;
    gyro_sum.z += gyro_results.z - gyro_offsets.z;

    accel_results_degrees = read_accel();

    accel_results_degrees.x -= accel_offsets.x;
    accel_results_degrees.y -= accel_offsets.y;
    accel_results_degrees.z -= accel_offsets.z;

    static float x_gyro_cos = 0;
    x_gyro_cos = cos(gyro_sum.x * PI / 180.0);
    static float x_gyro_sin = 0;
    x_gyro_sin = sin(gyro_sum.x * PI / 180.0);

    static float x_accel_cos = 0;
    x_accel_cos = cos(accel_results_degrees.x * PI / 180.0);
    static float x_accel_sin = 0;
    x_accel_sin = sin(accel_results_degrees.x * PI / 180.0);

    static float y_gyro_cos = 0;
    y_gyro_cos = cos(gyro_sum.y * PI / 180.0);
    static float y_gyro_sin = 0;
    y_gyro_sin = sin(gyro_sum.y * PI / 180.0);

    static float y_accel_cos = 0;
    y_accel_cos = cos(accel_results_degrees.y * PI / 180.0);
    static float y_accel_sin = 0;
    y_accel_sin = sin(accel_results_degrees.y * PI / 180.0);

    static float filtered_cos_x = 0;
    filtered_cos_x = 0.98 * x_gyro_cos + 0.02 * x_accel_cos;
    static float filtered_sin_x = 0;
    filtered_sin_x = 0.98 * x_gyro_sin + 0.02 * x_accel_sin;

    gyro_sum.x = atan2f(filtered_sin_x, filtered_cos_x) * RAD_TO_DEG;

    static float filtered_cos_y = 0;
    filtered_cos_y = 0.98 * y_gyro_cos + 0.02 * y_accel_cos;
    static float filtered_sin_y = 0;
    filtered_sin_y = 0.98 * y_gyro_sin + 0.02 * y_accel_sin;

    gyro_sum.y = atan2f(filtered_sin_y, filtered_cos_y) * RAD_TO_DEG;

    angleErrors.angle_error_x = -gyro_sum.x;
    angleErrors.angle_error_y = -gyro_sum.y;
    angleErrors.angle_error_z = 0;

    /* Compute new values for motors */
    regulation_loop(angleErrors);

    motor_value_a = quadcopter.motor_1_value + pwm_value * 2;
    motor_value_b = quadcopter.motor_2_value + pwm_value * 2;
    motor_value_c = quadcopter.motor_3_value + pwm_value * 2;
    motor_value_d = quadcopter.motor_4_value + pwm_value * 2;
          
    if (console_getDebugInfoStatus())
    {
        SerialUSB.print("ex;\t");
        SerialUSB.print(angleErrors.angle_error_x);
        SerialUSB.print(";\tey;\t");
        SerialUSB.print(angleErrors.angle_error_y);
        SerialUSB.print(";\tpwm;\t");
        SerialUSB.print(pwm_value);
        SerialUSB.print(";\ta;\t");
        SerialUSB.print(motor_value_a);
        SerialUSB.print(";\tb;\t");
        SerialUSB.print(motor_value_b);
        SerialUSB.print(";\tc;\t");
        SerialUSB.print(motor_value_c);
        SerialUSB.print(";\td;\t");
        SerialUSB.println(motor_value_d);
    }
    if (false == console_getDebugArmedStatus())
    {
        motor_value_a = 0;
        motor_value_b = 0;
        motor_value_c = 0;
        motor_value_d = 0;
    }
    
    setMotorValue(MOTOR_A, motor_value_a);
    setMotorValue(MOTOR_C, motor_value_c);
    setMotorValue(MOTOR_B, motor_value_b);
    setMotorValue(MOTOR_D, motor_value_d);   
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
