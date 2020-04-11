/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include "modules/esc/esc.h"
#include "modules/mpu9250/mpu9250.h"
#include "modules/regulation/regulation.h"
#include "modules/receiver/receiver.h"

void setup()
{
  setup_esc();

  setup_driver();

  Rx_setup();
  
  regulation_init();
}

void loop()
{
    String serialData;
    String command;
    float value = 0;

    if (SerialUSB.available() > 0)
    {
        serialData = SerialUSB.readString(); // read the incoming byte:

        command = serialData[0];
        serialData.remove(0, 1);
        value = atof(serialData.c_str());
        switch(command[0]){
            case 'P':
            case 'p':
                SerialUSB.println("P coeff changed.");
                setPidx_P(value);
                break;
            case 'I':
            case 'i':
                SerialUSB.println("I coeff changed.");
                setPidx_I(value);
                break;
            case 'D':
            case 'd':
                SerialUSB.println("D coeff changed.");
                setPidx_D(value);
                break;
            default:
                SerialUSB.println("Unknown command.");
                break;
        }
        print_Pidx();
    }

}
