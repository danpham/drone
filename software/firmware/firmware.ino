
#include "modules/mpu9250/mpu9250.h"
#include "modules/regulation/regulation.h"
#include "modules/esc/esc.h"

void setup()
{
  setup_esc();

  setup_driver();

  setup_receiver();
  
  regulation_init();
}

void loop()
{
}
