#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <stdint.h>
/* Pinout Definition */
#define tftCsPin  7

#define tftDcPin    A5
#define tftRstPin   A4 // Or set to -1 and connect to Arduino RESET pin

uint16_t        Display_Text_Color         = 0x0000;
uint16_t        Display_Backround_Color    = 0xFFFF;
Adafruit_ST7735 tft = Adafruit_ST7735(tftCsPin, tftDcPin, tftRstPin);

void setup() {
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");

  // Init ST7735S chip, black tab
  tft.initR(INITR_BLACKTAB);      
  // initialise the display
  tft.setFont();
  tft.fillScreen(Display_Backround_Color);
  tft.setTextColor(Display_Text_Color);
  tft.setTextSize(2);
  tft.setCursor(5,5); 
  tft.print("Hello");
  tft.setCursor(5,25); 
  tft.print("World!");
  
}

void loop() {

}
