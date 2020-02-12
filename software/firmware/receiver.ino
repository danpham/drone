/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include <Arduino.h>

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
static void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq);

void Rx_setup()
{
  pinMode(THROTTLE_PIN, INPUT);
  startTimer(TC2, 0, TC6_IRQn); //TC2 channel 0, the IRQ for that channel
}

static void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq)
{

  /* Enable TC configuring see 36.7.20 */
  REG_TC2_WPMR=0x54494D00;

  //enable configuring the io registers. see 32.7.42
  REG_PIOC_WPMR=0x50494F00;

  //we need to configure the pin to be controlled by the right peripheral.
  //pin 5 is port C. PIOC_PDR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //and PIO_PDR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h
  //this disables the pio from controlling the pin. see 32.7.2

  REG_PIOC_PDR |= PIO_PDR_P25;

  //next thing is to assign the io line to the peripheral. See 32.7.24.
  //we need to know which peripheral we should use. Read table 37-4 in section 37.5.1.
  //TIOA6 is peripheral B, so we want to set that bit to 1.
  //REG_PIOC_ABSR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //PIO_ABSR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h
  REG_PIOC_ABSR |= PIO_ABSR_P25;

  //allow configuring the clock.
  pmc_set_writeprotect(false);

  /*
    Every peripheral in the SAM3X is off by default (to save power) and
    should be turned on.
    */
  pmc_enable_periph_clk(ID_TC6);

  /*
    configure the timer. All this is about setting TC_CMRx, see 37.7.10 in atmel pdf.
    We use CLOCK1 at 42 MHz to get the best possible resolution.
    We want input capture on TIOA6 (pin 5). Nothing else should be necessary, BUT there is a caveat:
    As mentioned in 37.6.8, we only get the value loaded in RA if not loaded since the last trigger,
    or RB has been loaded. Since I do not want to trigger as that sets the timer value to 0, I
    instead let register B be loaded when the pulse is going low.
    */
  TC_Configure(tc, channel,  TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_LDRA_RISING | TC_CMR_LDRB_FALLING);

  //set the interrupt flags. We want interrupt on overflow and TIOA6 (pin 5) going high.
  const uint32_t flags=TC_IER_COVFS  | TC_IER_LDRBS;
  tc->TC_CHANNEL[channel].TC_IER=flags;
  tc->TC_CHANNEL[channel].TC_IDR=~flags;//assume IER and IDR are equally defined.

  NVIC_EnableIRQ(irq);

  //start the timer
  TC_Start(tc,channel);
}

void TC6_Handler()
{
  /* reads the interrupt. necessary to clear the interrupt flag */
  const uint32_t status=TC_GetStatus(TC2, 0);
  uint32_t low_counter = 0;
  uint32_t counter = 0;
  uint32_t res = 0;

  const bool overflowed=status & TC_SR_COVFS;
  const bool inputcaptureA=status & TC_SR_LDRAS;
  const bool inputcaptureB=status & TC_SR_LDRBS;

  if (inputcaptureA){
    low_counter = TC2->TC_CHANNEL[0].TC_RA;
  }
 
  if (inputcaptureB){
    counter = TC2->TC_CHANNEL[0].TC_RB;
  }  

  /* Counter overflow */
  if (overflowed){
    res = 0xFFFFFFFF - low_counter + counter;
  } else {
    res = counter - low_counter;
  }

  if (inputcaptureA && inputcaptureA){
     pwm_counter = res / 42; /* Divide by 42 as clock is 42MHz */
     pwmNew = true;
  }
}
