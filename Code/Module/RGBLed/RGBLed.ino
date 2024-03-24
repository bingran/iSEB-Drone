#include <WS2812FX.h>

/* Pinout Definition */
#define rgbledPin 4

WS2812FX ws2812fx = WS2812FX(1, rgbledPin, NEO_GRB + NEO_KHZ800);

void setup() {
  // put your setup code here, to run once:
  ws2812fx.init();
  ws2812fx.setBrightness(1);
  ws2812fx.setSegment(0, 0,1, FX_MODE_SINGLE_DYNAMIC,  RED, 200, false);
  ws2812fx.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  ws2812fx.service();
}
