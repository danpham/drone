/******************************************************************
   1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include <Arduino.h>

/******************************************************************
   2. Define declarations (macros then function macros)
******************************************************************/

/******************************************************************
   3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
   4. Variable definitions (static then global)
******************************************************************/
static bool debugInfoStatus = false;
static bool debugArmedStatus = true;

/******************************************************************
   5. Functions prototypes (static only)
******************************************************************/

/*
 * Enable debug messages on serial console
 */
bool console_getDebugInfoStatus(){
    return debugInfoStatus;
}


/*
 * Get motor status
 */
bool console_getDebugArmedStatus(){
    return debugArmedStatus;
}


void console()
{
    String serialData;
    String command;
    float value = 0;

    if (SerialUSB.available() > 0)
    {
        serialData = SerialUSB.readString();

        command = serialData[0];
        serialData.remove(0, 1);
        value = atof(serialData.c_str());
        switch(command[0])
        {
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
            case 'A':
            case 'a':
                if (true == debugArmedStatus)
                {
                    SerialUSB.println("Motors are disarmed");
                    debugArmedStatus = false;
                }
                else
                {
                    SerialUSB.println("Motors are armed");
                    debugArmedStatus = true;
                }
                break;
            case 'S':
            case 's':
                if (false == debugInfoStatus)
                {
                    SerialUSB.println("Show debug");
                    debugInfoStatus = true;
                }
                else
                {
                    SerialUSB.println("Hide debug");
                    debugInfoStatus = false;
                }
                break;
            default:
                SerialUSB.println("Unknown command.");
                break;
        }
        print_Pidx();
    }
}