#include <SoftwareSerial.h>
#include <Adafruit_Pixie.h>

#include <FastLED.h>

#define PIXIEPIN 5 // Pin number for SoftwareSerial output

SoftwareSerial pixieSerial(-1, PIXIEPIN);
Adafruit_Pixie strip = Adafruit_Pixie(1, &pixieSerial);

#define NUM_LEDS 300
#define NUM_LEDS_PER_STRIP 300
#define NUM_STRIPS 1
#define MAX_BRIGHTNESS 255

// PINS
// 1x Potentiometer
// 1x Mic
// 1x Button
// 2x Accel - SDA/SCL - 38/37 or 3/2 or 18/19
// 4x per Wing, 3x Body, 1x rest
// OLED eyes SI->MOSI (0,11), CL->SCLK (32,13) +3 pins

int led = 13;

CRGB leds[NUM_LEDS];

// the setup routine runs once when you press reset:
void setup()
{
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);

  pixieSerial.begin(115200);
  strip.setBrightness(80);
  strip.setPixelColor(0, 0, 0, 255);
  strip.show();

  delay(2000);

  FastLED.addLeds<WS2811_PORTD, NUM_STRIPS>(leds, NUM_LEDS_PER_STRIP);
  // // FastLED.addLeds<WS2811_PORTDC, 8>(leds, NUM_LEDS_PER_STRIP);
  // //.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  // FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  Serial.begin(115200);
  delay(1000); // if we fucked it up - great idea by fastled :D
}

void colorWheel()
{
  static uint8_t hue = 0;
  hue++;
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
    {
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV((32 * i) + hue + j, 192, 255);
    }
  }
}

void clear()
{
  memset(leds, 0, sizeof(leds));
}

void testled()
{
  static uint8_t lvl = HIGH;
  digitalWrite(led, lvl);
  if (lvl == HIGH)
    lvl = LOW;
  else
    lvl = HIGH;
}

void runner()
{
  static int pos = 0;
  // if (pos >= NUM_LEDS_PER_STRIP)
  // {
  //   pos = 0;
  // }
  clear();
  // leds[0] = CRGB(0, 255, 0);
  // leds[1] = CRGB(255, 0, 0);
  // leds[2] = CRGB(0, 0, 255);

  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < 300; j++)
    {
      char r, g, b;
      if (j % 3 == 0)
      {
        r = 255;
      }
      else if (j % 3 == 1)
      {
        g = 255;
      }
      else if (j % 3 == 2)
      {
        b = 255;
      }

      leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(r, g, b);
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 255, 0) ;
    }
    // leds[(i * NUM_LEDS_PER_STRIP) + pos] = CRGB(255, 0, 0);
    // leds[(i * NUM_LEDS_PER_STRIP) + ((pos + 1) % NUM_LEDS_PER_STRIP)] = CRGB(0, 255, 0);
    // leds[(i * NUM_LEDS_PER_STRIP) + ((pos + 2) % NUM_LEDS_PER_STRIP)] = CRGB(0, 0, 255);
    // leds[(i * NUM_LEDS_PER_STRIP) + ((pos + 3) % NUM_LEDS_PER_STRIP)] = CRGB(0, 255, 0);
  }
  // pos++;
}

uint16_t fps = 0;

void showFps()
{
  Serial.println(fps);
  fps = 0;
}

// the loop routine runs over and over again forever:
void loop()
{

  strip.setPixelColor(0, 0, 0, 255);
  strip.show();

  // Turn the LED on, then pause
  // leds[0] = CRGB(20, 20, 255);
  fps++;
  runner();
  FastLED.show();

  EVERY_N_MILLISECONDS(1000) { showFps(); }

  // delay(20);
  // Now turn the LED off, then pause
  // leds[0] = CRGB::Black;
  // clear();
  // FastLED.show();
  // delay(500);

  EVERY_N_MILLISECONDS(500) { testled(); }
  // colorWheel();
  // runner();
  // leds[0] = CRGB(20, 20, 255);
  // leds[1] = CRGB(20, 255, 20);
  // leds[2] = CRGB(255, 20, 20);

  // FastLED.show();
  // FastLED.delay(1000);
}
