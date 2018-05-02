#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, 6, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.show();
}

int cycle = 1;
int red = 255;
int green = 0;
int blue = 0;
boolean increasing = true;
int start_pixel = 0;
int count = 0;

void loop() {

  if (count % 20000 == 0) {
  // put your main code here, to run repeatedly:
    for (int i = start_pixel; i < start_pixel + 1; ++i) {
      int n = i % 30;

      if (cycle == 0) {
        if (increasing) {
          red += 51;
          strip.setPixelColor(n, red, blue, green);
          if (red >= 255) {
            cycle = 2;
            increasing = false;
          }
        } else if (!increasing) {
          red -= 51;
          strip.setPixelColor(n, red, blue, green);
          if (red <= 0) {
            cycle = 2;
            increasing = true;
          }
        }
      }
    
      else if (cycle == 1) {
        if (increasing) {
          green += 51;
          strip.setPixelColor(n, red, blue, green);
          if (green >= 255) {
            cycle = 0;
            increasing = false;
          }
        } else if (!increasing) {
          green -= 51;
          strip.setPixelColor(n, red, blue, green);
          if (green <= 0) {
            cycle = 0;
            increasing = true;
          }
        }
      }

      else if (cycle == 2) {
        if (increasing) {
          blue += 51;
          strip.setPixelColor(n, red, blue, green);
          if (blue >= 255) {
            cycle = 1;
            increasing = false;
          }
        } else if (!increasing) {
          blue -= 51;
          strip.setPixelColor(n, red, blue, green);
          if (blue <= 0) {
            cycle = 1;
            increasing = true;
          }
        }
      }
    }
    strip.show();

  }
  ++count;
  if (count%20 == 0) {
//    ++start_pixel;
  }
}
