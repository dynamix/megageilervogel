// for the pixie
#include <SoftwareSerial.h>
#include <Adafruit_Pixie.h>

#include <FastLED.h>

// #include <Audio.h>

// #include <SPI.h>
// #include <SerialFlash.h>

#define POTENTIOMETER_PIN 16
#define MIC_PIN 17

#define PIXIEPIN 33 // Pin number for SoftwareSerial output

SoftwareSerial pixieSerial(-1, 5);
Adafruit_Pixie strip = Adafruit_Pixie(1, &pixieSerial);

#define NUM_LEDS 2400
#define NUM_LEDS_PER_STRIP 300
#define NUM_STRIPS 4
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

// AudioInputAnalog adc1(17); //xy=164,95
// AudioAnalyzePeak peak1;    //xy=317,123
// AudioConnection patchCord1(adc1, peak1);

void checkPotentiometer()
{
  static uint16_t potentiometer = 0;
  potentiometer = analogRead(POTENTIOMETER_PIN);
  uint8_t brightness = potentiometer / 4;
  // if (usePotentiometer == 1)
  // {
  LEDS.setBrightness(brightness);
  strip.setBrightness(brightness);
  // }
}

// audio code
#define DC_OFFSET 0 // DC offset in mic signal
#define NOISE 100
#define SAMPLES 60 // samples for the mic buffer

#define MIN_DIST_AUDIO_LEVELS 10

int vol[SAMPLES];
int lvl = 10; // audio level dampend
int minLvlAvg = 0;
int maxLvlAvg = 512;
byte volumeSampleIndex = 0;

void audioUpdate()
{
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n = analogRead(MIC_PIN);            // Raw reading from mic
  n = abs(n - 512 - DC_OFFSET);       // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;         // "Dampened" reading (else looks twitchy)

  vol[volumeSampleIndex] = n; // Save sample for dynamic leveling
  if (++volumeSampleIndex >= SAMPLES)
    volumeSampleIndex = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)
      minLvl = vol[i];
    else if (vol[i] > maxLvl)
      maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < MIN_DIST_AUDIO_LEVELS)
    maxLvl = minLvl + MIN_DIST_AUDIO_LEVELS;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

#define TOP 30

void simpleAudio()
{
  clear();
  audioUpdate();
  int height;
  static uint8_t hue = 0;
  hue++;
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  Serial.print(lvl);
  Serial.print(" ");
  Serial.print(minLvlAvg);
  Serial.print(" ");
  Serial.println(maxLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;

  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < height; j++)
    {
      // char r, g, b;
      // if (j % 3 == 0)
      // {
      //   r = 255;
      // }
      // else if (j % 3 == 1)
      // {
      //   g = 255;
      // }
      // else if (j % 3 == 2)
      // {
      //   b = 255;
      // }

      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 255, 0);
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(hue + j, 255, 255);
    }
  }
}

// the setup routine runs once when you press reset:
void setup()
{

  // AudioMemory(4);
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);

  pixieSerial.begin(115200);
  strip.setBrightness(80);
  strip.setPixelColor(0, 0, 0, 255);
  strip.show();
  delay(1000);
  // strip.setBrightness(80);
  // strip.setPixelColor(0, 0, 255, 0);
  // strip.show();

  // delay(2000);
  // strip.setBrightness(255);
  // strip.setPixelColor(0, 255, 0, 0);
  // strip.show();

  // delay(2000);

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
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(255, 255, 255);
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV((32 * i) + hue + j, 192, 255);
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

void randomBluePixelsOnSphere()
{
  clear();
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < 20; j++)
    {
      int x = random(0, 250);
      leds[x + (i * NUM_LEDS_PER_STRIP)] = CRGB::Blue;
    }
  }
}

void ftest()
{
  clear();
  for (int i = 0; i < 4; i++)
  {
    CRGB c;
    if (i == 0)
    {
      c = CRGB::Green;
    }
    if (i == 1)
    {
      c = CRGB::Blue;
    }
    if (i == 2)
    {
      c = CRGB::Red;
    }
    if (i == 3)
    {
      c = CRGB::Yellow;
    }
    // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    for (int j = 0; j < 300; j++)
    {

      leds[j + (i * NUM_LEDS_PER_STRIP)] = c;
    }
  }
}

void runner()
{
  static int pos = 0;
  static uint8_t hue = 0;

  //   -  strip.setPixelColor(0, 0, 0, 255);
  // +  static uint8_t hue = 0;
  // +  hue++;
  // +  CRGB rgb = CRGB(0, 0, 0);
  // +  rgb.setHSV(hue, 192, 255);
  // +
  // +  strip.setPixelColor(0, rgb.r, rgb.g, rgb.b);
  // if (pos >= NUM_LEDS_PER_STRIP)
  // {
  //   pos = 0;
  // }
  clear();
  // leds[0] = CRGB(0, 255, 0);
  // leds[1] = CRGB(255, 0, 0);
  // leds[2] = CRGB(0, 0, 255);

  hue++;
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < 300; j++)
    {
      // char r, g, b;
      // if (j % 3 == 0)
      // {
      //   r = 255;
      // }
      // else if (j % 3 == 1)
      // {
      //   g = 255;
      // }
      // else if (j % 3 == 2)
      // {
      //   b = 255;
      // }

      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 255, 0);
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(hue + j, 255, 255);
      //leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(255, 255, 255);
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(r, g, b);
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

// elapsedMillis fpsx;

// void looptestforpeak()
// {
//   if (fpsx > 24)
//   {
//     if (peak1.available())
//     {
//       fpsx = 0;
//       int monoPeak = peak1.read() * 30.0;
//       Serial.print("|");
//       for (int cnt = 0; cnt < monoPeak; cnt++)
//       {
//         Serial.print(">");
//       }
//       Serial.println();
//     }
//   }
// }

// the loop routine runs over and over again forever:
void loop()
{

  CRGB c(0, 0, 0);
  static uint8_t hue = 0;
  static uint8_t t = 0;

  EVERY_N_MILLISECONDS(20) { hue++; }
  c.setHSV(hue, 255, 255);
  strip.setPixelColor(0, c.r, c.g, c.b);
  strip.setPixelColor(1, c.r, c.g, c.b);
  strip.setPixelColor(2, c.r, c.g, c.b);
  strip.setPixelColor(3, c.r, c.g, c.b);
  strip.show();

  // Turn the LED on, then pause
  // leds[0] = CRGB(20, 20, 255);
  fps++;
  // if (t > 20)
  // {
  // runner();
  // simpleAudio();
  ftest();
  // ftest();
  FastLED.show();
  // }
  // delay(10);

  EVERY_N_MILLISECONDS(1000) { showFps(); }

  EVERY_N_MILLISECONDS(1000) { t++; }

  EVERY_N_MILLISECONDS(100) { checkPotentiometer(); }

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
