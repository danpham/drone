/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "math.h"
#include "modules/mpu9250/MPU6500.h"
#include "modules/receiver/receiver.h"

/******************************************************************
 * 2. Define declarations (macros then function macros)
******************************************************************/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define NCS_PIN 4

/******************************************************************
 * 3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
 * 4. Variable definitions (static then global)
******************************************************************/
int count_second = 0;
short acc[3] = {0, 0, 0};
short gyro[3] = {0, 0, 0};
unsigned short n_samples = 0;
unsigned short fifo_count = 0;
unsigned short fifo_err = 0;
gyro_t gyro_offsets;
gyro_t gyro_sum;
char gyro_initialized = 0;
int FREQ_timer = 250;
accel_t accel_results;
accel_t accel_results_degrees;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/******************************************************************
 * 5. Functions prototypes (static only)
******************************************************************/

byte SPI_read_register( byte reg )
{
  SPI.transfer(NCS_PIN, reg | 0x80, SPI_CONTINUE ); // reg | 0x80 to denote read
  byte read_value = SPI.transfer(NCS_PIN, 0x00); // write 8-bits zero

  return read_value;
}

void SPI_write_register(byte reg, byte value)
{
  SPI.transfer(NCS_PIN, reg, SPI_CONTINUE);
  SPI.transfer(NCS_PIN, value);
}

void draw(float x, float y, float z) {
  display.clearDisplay();

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  /*for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
    }*/
  display.print(F("x: "));
  display.print(x);
  display.print(F("\n"));
  display.print(F("y: "));
  display.print(y);
  display.print(F("\n"));
  display.print(F("pwm: "));
  display.print(z);
  display.print(F("\n"));
  display.display();
}

void setup_driver()
{
  int i = 0;
  byte id = 0;
  
  /* SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally */
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
  }
  display.clearDisplay();
  display.display();

  for ( i = 0; i < 3; i++ )
  {
    acc[i] = 0;
    gyro[i] = 0;
  }

  SerialUSB.begin(115200);
  SPI.begin(NCS_PIN);
  SPI.setDataMode(NCS_PIN, SPI_MODE0);
  SPI.setBitOrder(NCS_PIN, MSBFIRST);
  SPI.setClockDivider(NCS_PIN, 84); // 1MHz

  /*delay( 1000 );
    while (!SerialUSB) {
    ; // wait for serial port to connect
    }*/
  SerialUSB.println("MPU driver is starting");

  /*
    the I2C interface should be disabled by setting the
    I2C_IF_DIS configuration bit.
    Setting this bit should be performed immediately after waiting
    for the time specified by the â€œStart Up Time for Register Read/Writeâ€
    in Section 6.3.
    For further information regarding the I2C_IF_DIS
    bit, please refer to the MPU-6500 Register Map and
    Register Descriptions document.
  */

  // reset the device
  SPI_write_register( MPU6500_RA_PWR_MGMT_1, 0x80 );

  delay(100); // page 42 - delay 100ms

  // reset gyro, accel, temp
  SPI_write_register( MPU6500_RA_SIGNAL_PATH_RESET, 0x05 );

  delay(100); // page 42 - delay 100ms

  // set SPI mode by setting I2C_IF_DIS
  // reset DMP, FIFO, SIG
  SPI_write_register( MPU6500_RA_USER_CTRL, 0x10 | 0x8 | 0x4 | 0x1 );

  delay(1000);

  id = SPI_read_register( MPU6500_RA_WHO_AM_I );
  switch (id)
  {
     case 0x68:
        SerialUSB.print("Sensor is MPU6050");
        break;
     case 0x70:
        SerialUSB.print("Sensor is MPU6500");
        break;
     case 0x71:
        SerialUSB.print("Sensor is MPU9250");
        break;
     case 0x73:
        SerialUSB.print("Sensor is MPU9255");
        break;
     case 0x74:
        SerialUSB.print("Sensor is MPU6515");
        break;
     default:
        SerialUSB.print("Unknown sensor: ");
        SerialUSB.println(id, HEX);
  }
  /* Gyro range selection: 0x00 250 degrees per second */
  id = SPI_read_register(MPU6500_RA_GYRO_CONFIG);
  SerialUSB.print("GYRO_FS_SEL: ");
  SerialUSB.println(id, HEX);


  id = SPI_read_register( MPU6500_RA_ACCEL_CONFIG );
  SerialUSB.print("Test ACCEL_FS_SEL: ");
  SerialUSB.println(id, HEX);

  // Set DLPF_CFG to 1: 1kHz Gyro sampling, 41Hz bandwidth
  SPI_write_register(MPU6500_RA_CONFIG, 0x02);

  id = SPI_read_register(MPU6500_RA_CONFIG);
  SerialUSB.print("Test MPU6500_RA_CONFIG: ");
  SerialUSB.println(id, HEX);

  // accel filter
  SPI_write_register(MPU6500_RA_FF_THR, 0x02);
  id = SPI_read_register(MPU6500_RA_FF_THR);
  SerialUSB.print("Test MPU6500_RA_FF_THR: ");
  SerialUSB.println(id, HEX);
  
  // Default: 1kHz Accel sampling, 480Hz cutoff

  // enable temperature, gyro, and accelerometer output
  // with temp
  //SPI_write_register( MPU6500_RA_FIFO_EN, MPU6500_FIFO_EN_ACC | MPU6500_FIFO_EN_TEMP | MPU6500_FIFO_EN_GYRO);
  //SPI_write_register( MPU6500_RA_FIFO_EN, MPU6500_FIFO_EN_ACC | MPU6500_FIFO_EN_GYRO);

  // disable / enable FIFO
  // set SPI mode by setting I2C_IF_DIS
  SPI_write_register( MPU6500_RA_USER_CTRL, 0x10 );

  // speed up to take data
  SPI.setClockDivider(NCS_PIN, 21); // 10.5MHz; 7 sometimes works

  // Enable fifo interrupt
  //pinMode(10, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(10), fifo_overflow, CHANGE);
  // Enable fifo interrupt
  //SPI_write_register(MPU6500_RA_INT_ENABLE, 0x10);

  accel_results.z = 0;
  accel_results.x = 0;
  accel_results.y = 0;

  startTimer(TC1, 0, TC3_IRQn, FREQ_timer);
}



short read_short_value(char register_high, char register_low) {
  short value = 0;
  char high = 0;
  char low = 0;

  high = SPI_read_register(register_high);
  low = SPI_read_register(register_low);
  value = (high << 8) + low;

  return value;
}


accel_t read_accel(void)
{
  short accelX = 0;
  accelX = read_short_value(MPU6500_RA_ACCEL_XOUT_H, MPU6500_RA_ACCEL_XOUT_L);

  short accelY = 0;
  accelY = read_short_value(MPU6500_RA_ACCEL_YOUT_H, MPU6500_RA_ACCEL_YOUT_L);

  short accelZ = 0;
  accelZ = read_short_value(MPU6500_RA_ACCEL_ZOUT_H, MPU6500_RA_ACCEL_ZOUT_L);

  // Sensitivity Scale Factor = 16384 in 2G mode
  // -1 to reverse axis from drawing xyz in PCB
  float real_accelX = (float) - accelX / 16384;
  float real_accelY = (float)accelY / 16384;
  float real_accelZ = (float)accelZ / 16384;

  accel_results.x = atan2f(real_accelZ, real_accelY);
  accel_results.y = atan2f(real_accelX, real_accelZ);
  accel_results.z = atan2f(real_accelY, real_accelX);

  float degrees_z = accel_results.z * RAD_TO_DEG;
  float degrees_x = accel_results.x * RAD_TO_DEG;
  float degrees_y = accel_results.y * RAD_TO_DEG;

  accel_results_degrees.x = degrees_x;
  accel_results_degrees.y = degrees_y;
  accel_results_degrees.z = degrees_z;

  return accel_results_degrees;
}

gyro_t read_gyro(void)
{
  gyro_t gyro_results;

  short gyroX = 0;
  gyroX = read_short_value(MPU6500_RA_GYRO_XOUT_H, MPU6500_RA_GYRO_XOUT_L);

  short gyroY = 0;
  gyroY = read_short_value(MPU6500_RA_GYRO_YOUT_H, MPU6500_RA_GYRO_YOUT_L);

  short gyroZ = 0;
  gyroZ = read_short_value(MPU6500_RA_GYRO_ZOUT_H, MPU6500_RA_GYRO_ZOUT_L);

  gyro_results.x = (float)((float)gyroX / (131 * FREQ_timer));
  gyro_results.y = (float)((float)gyroY / (131 * FREQ_timer));
  gyro_results.z = (float)((float)gyroZ / (131 * FREQ_timer));

  return gyro_results;
}



static void init_gyro(void) {
  gyro_t gyro_results;
  static int counter = 0;

  counter++;

  gyro_results = read_gyro();
  gyro_offsets.x += gyro_results.x;
  gyro_offsets.y += gyro_results.y;
  gyro_offsets.z += gyro_results.z;

  if (counter == 100) {
    gyro_offsets.x = gyro_offsets.x / 100;
    gyro_offsets.y = gyro_offsets.y / 100;
    gyro_offsets.z = gyro_offsets.z / 100;
    gyro_initialized = 1;
  }
}

void TC3_Handler() {
  gyro_t gyro_results;
  TC_GetStatus(TC1, 0);
  static unsigned long pwm_value = 40;
  
  if (!gyro_initialized) {
    init_gyro();
  }
  else {
    if (pwmNew == true){
        /* Wrong PWM appears because micros() is unstable */
        if ((pwm_counter > RX_MAX) || (pwm_counter < RX_MIN)){
          pwm_counter = RX_MIN;
        }
        pwm_value = ((pwm_counter >> 5) - 31)*3;

        pwmNew = false; 
    }
    
    gyro_results = read_gyro();
    gyro_sum.x += gyro_results.x - gyro_offsets.x;
    gyro_sum.y += gyro_results.y - gyro_offsets.y;
    gyro_sum.z += gyro_results.z - gyro_offsets.z;

    accel_results_degrees = read_accel();

    gyro_sum.x = (float)0.98 * gyro_sum.x + (float)0.02 * (90-accel_results_degrees.x - gyro_offsets.x);
    gyro_sum.y = (float)0.98 * gyro_sum.y + (float)0.02 * (accel_results_degrees.y - gyro_offsets.y);

    static angle_errors angleErrors;
    angleErrors.angle_error_x = gyro_sum.x;
    angleErrors.angle_error_y = gyro_sum.y;
    angleErrors.angle_error_z = 0;

    /* Compute new values for motors */
    regulation_loop(angleErrors);

    setMotorValue(MOTOR_A_PIN, quadcopter.motor_1_value + pwm_value);
    //setMotorValue(MOTOR_B_PIN, pwm_value);
    setMotorValue(MOTOR_C_PIN, quadcopter.motor_3_value + pwm_value);
    //setMotorValue(MOTOR_D_PIN, pwm_value);
  }
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {

  //Enable or disable write protect of PMC registers.
  pmc_set_writeprotect(false);
  //Enable the specified peripheral clock.
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
