/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include "modules/mpu9250/mpu9250.h"
#include "modules/regulation/regulation.h"
#include "modules/esc/esc.h"

void setup()
{
  setup_esc();

  setup_driver();

  Rx_setup();
  
  regulation_init();
}

void loop()
{
}
