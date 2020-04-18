/******************************************************************
   1. Included files (microcontroller ones then user defined ones)
******************************************************************/
#include <Arduino.h>

/******************************************************************
   2. Define declarations (macros then function macros)
******************************************************************/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

/******************************************************************
   3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
   4. Variable definitions (static then global)
******************************************************************/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/******************************************************************
   5. Functions prototypes (static only)
******************************************************************/

/**
  @desc Setup screen configuration
  @return void
*/
void setup_screen(void){
  /* SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally */
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { 
    /* Address 0x3C for 128x64 */
    display.clearDisplay();
    display.display();
    draw("Screen init.\nOK");
  }
}

/**
  @desc Draw text on screen
  @param text: String to display
  @return void
*/
void draw(String text) {
  display.clearDisplay();

  display.setTextSize(2);      /* Normal 1:1 pixel scale */
  display.setTextColor(WHITE); /* Draw white text */
  display.setCursor(0, 0);     /* Start at top-left corner */
  display.cp437(true);         /* Use full 256 char 'Code Page 437' font */

  display.print(text.c_str());
  display.display();
}