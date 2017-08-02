#include <FastLED.h>
#include <SoftwareSerial.h>
#include <Adafruit_Pixie.h>
#include "Adafruit_Trellis.h"

// #include <Audio.h>

// #include <SPI.h>
// #include <SerialFlash.h>
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define POTENTIOMETER_PIN 16
#define MIC_PIN 17

// eyes
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet pad = Adafruit_TrellisSet(&matrix0);

SoftwareSerial pixieSerial(-1, 5);
SoftwareSerial pixieSerial2(-1, 20);
SoftwareSerial pixieSerial3(-1, 21);
Adafruit_Pixie eyes = Adafruit_Pixie(1, &pixieSerial);
Adafruit_Pixie fiberHead = Adafruit_Pixie(1, &pixieSerial3);
Adafruit_Pixie fiberTail = Adafruit_Pixie(1, &pixieSerial2);

#define NUM_LEDS 600
#define NUM_LEDS_PER_STRIP 120
#define NUM_STRIPS 5
#define MAX_BRIGHTNESS 255
#define TEENSY_LED 13

CRGB leds[NUM_LEDS];

typedef void (*Mode[2])(void);

uint16_t perStrip[] = {68, 68, 100, 95};

#define EYE_STATE 12
#define FIBER_HEAD_STATE 13
#define FIBER_TAIL_STATE 14
#define FIBER_WING_STATE 15

uint8_t buttonState[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t usePotentiometer = 1;
int8_t currentMode = 0;
int8_t previousMode = 0;
uint8_t currentBrightness = MAX_BRIGHTNESS;
uint16_t currentDelay = 0;
uint8_t button = 0;
uint8_t shouldClear = 1;
uint8_t usePixies = 0;

void none() {}
void testMode() {}

Mode modes[] = {
    // {juggle, juggleSetup},
    {flash2, flash2Setup},
    {bpm, bpmSetup},
    {randomBluePixels, randomBluePixelsSetup},
    {fireWithPotentiometer, fireWithPotentiometerSetup},
    {fire, fireSetup},
    {fiberPulse, fiberPulseSetup},
    {fiberBlink, fiberBlinkSetup},

    // {ftest, none},
    {none, none},
    {sinelon, sinelonSetup},
    {fiberMode, fiberModeSetup},
    {rainbowSparks, rainbowSparksSetup},
    {sparks, sparksSetup},
    {simpleAudio, simpleAudioSetup},
    {runner, none},
    {colorWheelPulsing, colorWheelPulsingSetup},
    {iceSparks, iceSparksSetup},
};

// AudioInputAnalog adc1(17); //xy=164,95
// AudioAnalyzePeak peak1;    //xy=317,123
// AudioConnection patchCord1(adc1, peak1);

// DEBUG STUFF
// static int globalP = 0;

// the setup routine runs once when you press reset:
void setup()
{
  pad.begin(0x70);
  for (uint8_t i = 0; i < 16; i++)
  {
    pad.setLED(i);
    pad.writeDisplay();
    delay(50);
  }
  // then turn them off
  for (uint8_t i = 0; i < 16; i++)
  {
    pad.clrLED(i);
    pad.writeDisplay();
    delay(50);
  }

  // AudioMemory(4);
  // initialize the digital pin as an output.
  pinMode(TEENSY_LED, OUTPUT);

  pixieSerial.begin(115200);
  pixieSerial2.begin(115200);
  pixieSerial3.begin(115200);

  FastLED.addLeds<WS2811_PORTD, NUM_STRIPS>(leds, NUM_LEDS_PER_STRIP);
  // //.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  // FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  Serial.begin(115200);
  delay(10); // if we fucked it up - great idea by fastled :D

  modes[0][1]();
}

void nextMode(int8_t dir)
{
  int8_t newMode;
  newMode = currentMode + dir;

  if (newMode < 0)
  {
    newMode = ARRAY_SIZE(modes) - 1;
  }
  else if (newMode >= ARRAY_SIZE(modes))
  {
    newMode = 0;
  }
  Serial.print("NEXT MODE");
  Serial.println(newMode);
  setMode(newMode);
}

void setMode(uint8_t mode)
{

  previousMode = currentMode;
  currentMode = mode;
  if (currentMode < 0)
    currentMode = ARRAY_SIZE(modes) - 1;
  // setBrightness(MAX_BRIGHTNESS);
  usePotentiometer = 1;
  currentDelay = 0;
  shouldClear = 1;
  usePixies = 0;
  modes[currentMode][1]();
}

void checkSerial()
{
  if (Serial.available() > 0)
  {
    Serial.read();
    //nextMode(1);
    Serial.println(currentMode);
    // globalP++;
    // Serial.println(globalP);
  }
}

static uint16_t potentiometer = 0;

void checkPotentiometer()
{
  potentiometer = analogRead(POTENTIOMETER_PIN);
  uint8_t brightness = potentiometer / 4;
  currentBrightness = brightness;
  if (usePotentiometer == 1)
  {
    LEDS.setBrightness(brightness);
    // fiberHead.setBrightness(brightness);
    // fiberTail.setBrightness(brightness);
  }
}

void fiberModeSetup()
{
  currentDelay = 25;
}

void fiberMode()
{
  static int8_t hue = 0;
  hue++;
  CRGB c;
  c.setHSV(hue, 240, 255);
  fiberHead.setPixelColor(0, c.r, c.g, c.b);
  fiberTail.setPixelColor(0, c.r, c.g, c.b);
  fiberHead.setBrightness(currentBrightness);
  fiberTail.setBrightness(currentBrightness);
  // fibertest();
  fiberHead.show();
  fiberTail.show();
  leds[fiberleft(0)] = CHSV(hue * 2, 240, 255);
  leds[fiberleft(1)] = CHSV(hue, 240, 255);
  leds[fiberleft(2)] = CHSV(hue * 3, 200, 255);
  leds[fiberleft(3)] = CHSV(hue, 140, 255);
  leds[fiberright(0)] = CHSV(hue * 2, 240, 255);
  leds[fiberright(1)] = CHSV(hue, 240, 255);
  leds[fiberright(2)] = CHSV(hue * 3, 200, 255);
  leds[fiberright(3)] = CHSV(hue, 140, 255);
}

void checkButtons()
{
  if (pad.readSwitches())
  {
    // buttons with a direct mode mapping
    for (uint8_t i = 0; i < 12; i++)
    {

      if (pad.justPressed(i))
      {
        Serial.print("BUTTON ");
        Serial.println(i);
        pad.setLED(i);
        // if (i == 8)
        // {
        //   nextMode(1);
        // }
        // else if (i == 9)
        // {
        //   nextMode(-1);
        // }
        // else
        if (i == 10)
        {
          flash();
        }
        else if (i == 11)
        {
          strobo();
        }
        else if (i < 9)
        {
          setMode(i);
        }
      }

      if (pad.justReleased(i))
      {
        pad.clrLED(i);
      }
    }
  }

  for (uint8_t i = 12; i < 16; i++)
  {
    if (pad.justPressed(i))
    {
      if (pad.isLED(i))
      {
        pad.clrLED(i);
        buttonState[i] = 0;
      }
      else
      {
        pad.setLED(i);
        buttonState[i] = 1;
      }
    }
  }
  pad.writeDisplay();
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

#define TOP 60

void simpleAudioSetup()
{
  currentDelay = 10;
}

void simpleAudio()
{
  // clear();
  audioUpdate();
  int height;
  static uint8_t hue = 0;
  hue++;
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  // Serial.print("Audio: ");
  // Serial.print(lvl);
  // Serial.print(" ");
  // Serial.print(minLvlAvg);
  // Serial.print(" ");
  // Serial.println(maxLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;

  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < height; j++)
    {
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 255, 0);
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(hue + j, 255, 255);
    }
  }
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
  digitalWrite(TEENSY_LED, lvl);
  if (lvl == HIGH)
    lvl = LOW;
  else
    lvl = HIGH;
}

void randomBluePixelsSetup()
{
  currentDelay = 2;
}

void randomBluePixels()
{
  clear();
  for (int j = 0; j < 10; j++)
  {
    int x = random(0, NUM_LEDS);
    leds[x] = CRGB::Blue;
  }
}

uint16_t stripoffset(uint8_t n)
{
  return NUM_LEDS_PER_STRIP * n;
}

void ftest()
{
  clear();
  for (int i = 0; i < 4; i++)
  {
    CRGB c;
    // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
    // for (int j = 0; j < globalP; j++)
    {
      if (i % 4 == 0)
      {
        c = CRGB::Green;
      }
      if (i % 4 == 1)
      {
        c = CRGB::Blue;
      }
      if (i % 4 == 2)
      {
        c = CRGB::Red;
      }
      if (i % 4 == 3)
      {
        c = CRGB::Yellow;
      }

      leds[j + stripoffset(i)] = c;
    }
  }
}

uint16_t xy60x6(uint8_t x, uint8_t y)
{
  if (y > 29)
  {
    x = x + 6;
    y = 59 - y;
  }
  return (x * 30) + y;
}

uint16_t xy(uint8_t x, uint8_t y)
{
  if (x % 2 != 0)
    return (x % 12) * 30 + (y % 30);
  else
    return (x % 12) * 30 + (29 - (y % 30));
}

uint16_t fiberleft(int8_t n)
{
  return NUM_LEDS_PER_STRIP * 2 + 100 + n;
}

uint16_t fiberright(uint8_t n)
{
  return 95 + n;
}

void fibertest()
{

  // leds[fiberright(-5)] = CRGB::Green;
  // leds[fiberright(-3)] = CRGB::Green;
  // leds[fiberright(-3)] = CRGB::Green;
  // leds[fiberright(-2)] = CRGB::Green;
  // leds[fiberright(-1)] = CRGB::Green;
  leds[fiberright(0)] = CRGB::Green;
  leds[fiberright(1)] = CRGB::Blue;
  leds[fiberright(2)] = CRGB::Red;
  leds[fiberright(3)] = CRGB::Yellow;
  leds[fiberright(4)] = CRGB::Yellow;
  leds[fiberright(5)] = CRGB::Yellow;
  leds[fiberright(6)] = CRGB::Yellow;
  leds[fiberright(7)] = CRGB::Yellow;

  // leds[fiberleft(0)] = CRGB::Green;
  // leds[fiberleft(1)] = CRGB::Blue;
  // leds[fiberleft(2)] = CRGB::Red;
  // leds[fiberleft(3)] = CRGB::Yellow;

  // leds[fiberright(0)] = CRGB::Green;
  // leds[fiberright(1)] = CRGB::Blue;
  // leds[fiberright(2)] = CRGB::Red;
  // leds[fiberright(3)] = CRGB::Yellow;
}

void runner()
{
  static int pos = 0;
  static uint8_t hue = 0;

  hue++;
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
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

void colorWheelPulsingSetup()
{
  currentDelay = 30;
}
void colorWheelPulsing()
{
  static uint8_t hue = 0;
  static uint8_t pulse = 255;
  static int8_t dir = -1;

  pulse += dir;

  if (pulse < 100)
  {
    dir = 1;
  }
  if (pulse > 253)
  {
    dir = -1;
  }

  hue++;
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
    {
      leds[stripoffset(i) + j] = CHSV(16 * j + hue, 240, pulse);
    }
  }
}

void iceSparksSetup()
{
  usePotentiometer = 0;
  FastLED.setBrightness(255);
  currentDelay = 50;
  shouldClear = false;
}

void iceSparks()
{
  fadeToBlackBy(leds, NUM_LEDS, 128);

  static uint8_t sat = 0;
  sat++;
  for (int j = 0; j < 35; j++)
  {
    leds[random(0, NUM_LEDS)] = CHSV(170 + (random(20) - 10), random(255), 150);
  }
}

void basicPowerLedMode()
{
  // CRGB c(0, 0, 0);
  // static uint8_t hue = 0;
  static uint8_t eyeFlicker = 0;

  int16_t b = 0;
  CRGB c(255, 0, 0);

  static int8_t d = 1;
  static int16_t v = 10;

  v += d;
  if (v > 50)
    d = -1;
  if (v < -50)
    d = 1;

  // EVERY_N_MILLISECONDS(20) { hue++; }
  // c.setHSV(hue, 255, 255);
  if (buttonState[EYE_STATE] == 1)
  {
    b = currentBrightness + v;
    if (b < 0)
    {
      b = 0;
    }
    if (b > 255)
      b = 255;
    eyes.setBrightness(b);
  }
  else
  {
    eyes.setBrightness(0);
  }

  eyes.setPixelColor(0, c.r, c.g, c.b);
  eyes.show();
}

void sparksSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 10;
  usePotentiometer = 0;
}

void sparks()
{
  for (int j = 0; j < 30; j++)
  {
    leds[random(0, NUM_LEDS)] = CRGB::White;
  }
}

void fiberBlinkSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 5;
  usePotentiometer = 0;
  shouldClear = true;
  fiberHead.setBrightness(0);
  fiberTail.setBrightness(0);
}
void fiberBlink()
{
  int fiber = random(0, 4 + 4 + 2);
  fiberHead.setBrightness(0);
  fiberTail.setBrightness(0);
  if (fiber < 4)
  {
    leds[fiberleft(fiber)] = CRGB(255, 255, 255);
    // leds[fiberleft(fiber)] = CRGB(0, 0, 255);
  }
  else if (fiber >= 4 && fiber < 8)
  {
    // leds[fiberright(fiber - 4)] = CRGB(255, 0, 0);
    leds[fiberright(fiber - 4)] = CRGB(255, 255, 255);
  }
  else if (fiber == 8)
  {
    fiberTail.setPixelColor(0, 255, 255, 255);
    fiberTail.setBrightness(200);
  }
  else if (fiber == 9)
  {
    fiberHead.setPixelColor(0, 255, 255, 255);
    fiberHead.setBrightness(100);
  }

  fiberHead.show();
  fiberTail.show();
}

void fiberPulseSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 10;
  usePotentiometer = 0;
  shouldClear = true;
  fiberHead.setBrightness(0);
  fiberTail.setBrightness(0);
}
void fiberPulse()
{
  static uint8_t hue = 0;
  static uint8_t pulse = 255;
  static int8_t dir = -1;
  static int dirs[10] = {-1, 1, -1, 1, -1, 1, -1, 1, -1, 1};
  static byte pulses[10] = {random8(), random8(), random8(), random8(), random8(), random8(), random8(), random8(), random8(), random8()};

  hue++;
  for (int i = 0; i < 10; i++)
  {
    pulses[i] += dirs[i];

    if (pulses[i] < 10)
    {
      dirs[i] = 1;
    }
    if (pulses[i] > 253)
    {
      dirs[i] = -1;
    }

    CRGB c = CHSV(16 * i + hue, 240, pulses[i]);

    if (i < 4)
    {
      leds[fiberleft(i)] = c;
      // leds[fiberleft(fiber)] = CRGB(0, 0, 255);
    }
    else if (i >= 4 && i < 8)
    {
      // leds[fiberright(fiber - 4)] = CRGB(255, 0, 0);
      leds[fiberright(i - 4)] = c;
    }
    else if (i == 8)
    {
      fiberTail.setPixelColor(0, c.r, c.g, c.b);
      fiberTail.setBrightness(255);
    }
    else if (i == 9)
    {
      fiberHead.setPixelColor(0, c.r, c.g, c.b);
      fiberHead.setBrightness(255);
    }
  }
  fiberHead.show();
  fiberTail.show();
}

void rainbowSparksSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 15;
  usePotentiometer = 0;
}
void rainbowSparks()
{
  static uint8_t hue = 0;
  hue++;
  for (int j = 0; j < 40; j++)
  {
    leds[random(0, NUM_LEDS)] = CHSV(hue, 210, 255);
  }
}

void bpmSetup()
{
  currentDelay = 10;
}

void bpm()
{
  static uint8_t gHue = 0;
  uint8_t BeatsPerMinute = 100;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 32, 255);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette(palette, gHue + (i * 4), beat - gHue + (i * 10));
  }
  EVERY_N_MILLISECONDS(20) { gHue++; }
}

void juggleSetup()
{
  currentDelay = 50;
}

void juggle()
{
  static uint8_t gHue = 0;
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for (int i = 0; i < 8; i++)
  {
    leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void sinelon()
{
  static uint8_t gHue = 0;
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS);
  leds[pos] += CHSV(gHue, 255, 192);
  EVERY_N_MILLISECONDS(20) { gHue++; }
}

void sinelonSetup()
{
  shouldClear = false;
  FastLED.setBrightness(128);
}

void flash()
{
  eyes.setBrightness(255);
  eyes.setPixelColor(0, 255, 255, 255);
  eyes.show();
  delay(100);
  eyes.setBrightness(0);
  eyes.show();
}

CRGB stroboC[5] = {CRGB(0, 0, 255), CRGB(255, 0, 0), CRGB(0, 255, 255), CRGB(255, 0, 255), CRGB(255, 255, 255)};

void strobo()
{

  for (int i = 0; i < 5; i++)
  {
    delay(50);
    eyes.setBrightness(255);
    eyes.setPixelColor(0, 255, 20, 20);
    // eyes.setPixelColor(0, 126, 6, 229);
    // eyes.setPixelColor(0, stroboC[i].r, stroboC[i].g, stroboC[i].b);
    eyes.show();
    delay(50);
    eyes.setBrightness(0);
    eyes.show();
  }
}

CRGBPalette16 firePal;

void fireSetup()
{
  currentDelay = 40;
  usePixies = 1;
  // firePal = CRGBPalette16(CRGB::Black, CRGB::Red, CRGB::Yellow, CRGB::Grey);
  // firePal = CRGBPalette16(CRGB::Black, CRGB::Red, CRGB::Yellow, CRGB::White);
  // firePal = CRGBPalette16(CRGB::Black, CRGB::Red, CRGB::Yellow, CRGB::White);

  // firePal = CRGBPalette16(CRGB::Black,
  //                         CRGB::Black,
  //                         CRGB::Black,
  //                         CRGB::DarkBlue,
  //                         CRGB::MidnightBlue,
  //                         CRGB::Black,
  //                         CRGB::MediumBlue,
  //                         CRGB::Teal,
  //                         CRGB::CadetBlue,
  //                         CRGB::Blue,
  //                         CRGB::DarkCyan,
  //                         CRGB::CornflowerBlue,
  //                         CRGB::Aquamarine,
  //                         CRGB::SeaGreen,
  //                         CRGB::Aqua,
  //                         CRGB::LightSkyBlue);
  // firePal = LavaColors_p;
  firePal = HeatColors_p;
  // firePal = ForestColors_p;
  // firePal = OceanColors_p;
}

#define COOLING 4
#define SPARKING 80

void fire()
{
  static byte heat[NUM_LEDS];
  random16_add_entropy(random());
  // cool down
  for (int x = 0; x < NUM_LEDS; x++)
  {
    heat[x] = qsub8(heat[x], random8(0, COOLING));
  }
  for (int x = 0; x < NUM_LEDS; x++)
  {
    if (x == 0)
    {
      heat[0] = (heat[0] + heat[1]) / 2;
    }
    if (x == NUM_LEDS - 1)
    {
      heat[NUM_LEDS - 1] = (heat[NUM_LEDS - 1] + heat[NUM_LEDS - 2]) / 2;
    }
    if (x > 0 && x < NUM_LEDS - 1)
    {
      heat[x] = (heat[x - 1] + heat[x] + heat[x + 1]) / 3;
    }
  }

  // // drift up and difuse
  // for (int x = 0; x < X; x++)
  // {
  //   for (int k = Y - 1; k >= 2; k--)
  //   {
  //     int kk = k;
  //     int dir = 1;
  //     if (x % 2 != 0)
  //     {
  //       kk = 29 - k;
  //       dir = -1;
  //     }
  //     heat[(kk + x * Y)] = (heat[(kk + x * Y) - dir] + heat[(kk + x * Y) - (2 * dir)] + heat[(kk + x * Y) - (2 * dir)]) / 3;
  //   }
  // }

  // ignite wings
  for (int i = 0; i < 5; i++)
  {
    if (random8() < SPARKING)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4);
      heat[x] = qadd8(heat[x], random8(50, 150));
    }
  }
  if (random8() < SPARKING)
  {
    int x = random16(NUM_LEDS_PER_STRIP * 4);
    heat[x] = qadd8(heat[x], random8(80, 190));
  }

  // corpus
  for (int i = 0; i < 6; i++)
  {
    if (random8() < SPARKING)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4, NUM_LEDS_PER_STRIP * 5);
      heat[x] = qadd8(heat[x], random8(50, 150));
    }
  }

  // tail
  byte colorindex = scale8(heat[NUM_LEDS_PER_STRIP], 240);
  CRGB c = ColorFromPalette(firePal, colorindex);
  fiberTail.setPixelColor(0, c.r, c.g, c.b);
  fiberTail.setBrightness(255);
  fiberTail.show();

  // eyes
  // if (buttonState[EYE_STATE] == 1)
  // {
  byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  CRGB cc = ColorFromPalette(firePal, ci);

  // eyes.setBrightness(currentBrightness / 3);
  eyes.setBrightness(100);
  eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // }
  // else
  // {
  //   eyes.setBrightness(0);
  // }

  eyes.show();

  // map to pixels
  for (int j = 0; j < NUM_LEDS; j++)
  {
    byte colorindex = scale8(heat[j], 240);
    CRGB color = ColorFromPalette(firePal, colorindex);
    leds[j] = color;
  }
}

void fireWithPotentiometerSetup()
{
  fireSetup();
  usePotentiometer = 0;
}

void fireWithPotentiometer()
{

  int sparking = potentiometer / 4;
  int bodyCooling = 5;
  int wingCooling = 7;

  static byte heat[NUM_LEDS];
  random16_add_entropy(random());
  // cool down
  for (int x = 0; x < NUM_LEDS; x++)
  {
    heat[x] = qsub8(heat[x], random8(0, wingCooling));
  }
  for (int x = NUM_LEDS * 4; x < NUM_LEDS_PER_STRIP * 5; x++)
  {
    heat[x] = qsub8(heat[x], random8(0, bodyCooling));
  }
  for (int x = 0; x < NUM_LEDS; x++)
  {
    if (x == 0)
    {
      heat[0] = (heat[0] + heat[1]) / 2;
    }
    if (x == NUM_LEDS - 1)
    {
      heat[NUM_LEDS - 1] = (heat[NUM_LEDS - 1] + heat[NUM_LEDS - 2]) / 2;
    }
    if (x > 0 && x < NUM_LEDS - 1)
    {
      heat[x] = (heat[x - 1] + heat[x] + heat[x + 1]) / 3;
    }
    // if (x > 1 && x < NUM_LEDS - 2)
    // {
    //   heat[x] = (heat[x - 2] + heat[x - 1] + heat[x] + heat[x + 1] + heat[x + 2]) / 5;
    // }
  }

  // // drift up and difuse
  // for (int x = 0; x < X; x++)
  // {
  //   for (int k = Y - 1; k >= 2; k--)
  //   {
  //     int kk = k;
  //     int dir = 1;
  //     if (x % 2 != 0)
  //     {
  //       kk = 29 - k;
  //       dir = -1;
  //     }
  //     heat[(kk + x * Y)] = (heat[(kk + x * Y) - dir] + heat[(kk + x * Y) - (2 * dir)] + heat[(kk + x * Y) - (2 * dir)]) / 3;
  //   }
  // }

  // ignite wings
  for (int i = 0; i < 7; i++)
  {
    if (random8() < sparking)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4);
      heat[x] = qadd8(heat[x], random8(50, 190));
    }
  }

  // corpus
  for (int i = 0; i < 6; i++)
  {
    if (random8() < sparking)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4, NUM_LEDS_PER_STRIP * 5);
      heat[x] = qadd8(heat[x], random8(50, 150));
    }
  }

  // tail
  byte colorindex = scale8(heat[NUM_LEDS_PER_STRIP], 240);
  CRGB c = ColorFromPalette(firePal, colorindex);
  fiberTail.setPixelColor(0, c.r, c.g, c.b);
  fiberTail.setBrightness(255);
  fiberTail.show();

  // eyes
  // if (buttonState[EYE_STATE] == 1)
  // {
  byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  CRGB cc = ColorFromPalette(firePal, ci);

  // eyes.setBrightness(currentBrightness / 3);
  eyes.setBrightness(100);
  eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // }
  // else
  // {
  //   eyes.setBrightness(0);
  // }

  eyes.show();

  // map to pixels
  for (int j = 0; j < NUM_LEDS; j++)
  {
    byte colorindex = scale8(heat[j], 240);
    CRGB color = ColorFromPalette(firePal, colorindex);
    leds[j] = color;
  }
}

uint8_t frequency = 20;
uint8_t flashes = 8;
unsigned int dimmer = 1;

uint16_t ledstart;
uint8_t ledlen;

void flash2Setup()
{
  currentDelay = 0;
}
void flash2()
{

  ledstart = random16(NUM_LEDS);
  ledlen = random16(NUM_LEDS - ledstart);

  for (int flashCounter = 0; flashCounter < random8(3, flashes); flashCounter++)
  {
    if (flashCounter == 0)
      dimmer = 5;
    else
      dimmer = random8(1, 3);

    fill_solid(leds + ledstart, ledlen, CHSV(255, 0, 255 / dimmer));
    FastLED.show();
    delay(random8(4, 10));
    fill_solid(leds + ledstart, ledlen, CHSV(255, 0, 0));
    FastLED.show();

    if (flashCounter == 0)
      delay(150);

    delay(50 + random8(100));
  }

  delay(random8(frequency) * 100);
}

// the loop routine runs over and over again forever:
void loop()
{

  if (shouldClear)
    clear();
  if (!usePixies)
  {
    basicPowerLedMode();
  }
  modes[currentMode][0]();
  FastLED.show();

  delay(currentDelay);

  // CRGB c(0, 0, 0);
  // static uint8_t hue = 0;
  // static uint8_t t = 0;

  // EVERY_N_MILLISECONDS(20) { hue++; }
  // c.setHSV(hue, 255, 255);
  // eyes.setPixelColor(0, c.r, c.g, c.b);
  // fiberHead.setPixelColor(0, c.r, c.g, c.b);
  // fiberTail.setPixelColor(0, c.r, c.g, c.b);
  // eyes.show();
  // fiberHead.show();
  // fiberTail.show();

  // runner();

  EVERY_N_MILLISECONDS(100) { checkPotentiometer(); }
  EVERY_N_MILLISECONDS(30) { checkButtons(); }
  EVERY_N_MILLISECONDS(500) { testled(); }
  EVERY_N_MILLISECONDS(500) { checkSerial(); }
  // EVERY_N_MILLISECONDS(100) { simpleAudio(); }
  // colorWheel();
  // runner();
  // leds[0] = CRGB(20, 20, 255);
  // leds[1] = CRGB(20, 255, 20);
  // leds[2] = CRGB(255, 20, 20);

  // FastLED.show();
  // FastLED.delay(1000);
}
