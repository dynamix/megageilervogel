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
Adafruit_Pixie eyes = Adafruit_Pixie(1, &pixieSerial);
SoftwareSerial pixieSerial2(-1, 20);
Adafruit_Pixie fiberHead = Adafruit_Pixie(1, &pixieSerial2);
SoftwareSerial pixieSerial3(-1, 21);
Adafruit_Pixie fiberTail = Adafruit_Pixie(1, &pixieSerial3);

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
    {runner, none},
    {colorWheelPulsing, colorWheelPulsingSetup},
    {iceSparks, iceSparksSetup},
};

// // all the main modes we support
// Modes modes =         {
//   testMode,
//   // stars,
//   //spiral,
//   ambientAllRainbow,
//   ambientRedCycle,
//   gyro,
//   fire,
//   fireSensor,
//   simpleSensor, // move down later
//   ringAudio,
//   simpleAudio,
//   twoRingAudio,
//   colorWheel,
//   fastColorWheel,
//   colorWheelPulsing,
//   colorWheelUpDown,
//   segmentTurning,
//   randomBluePixelsOnSphere,
//   rainbowSparks,
//   randomSparks,
//   sparks,
//   sparksAndRainbow,
//   threeSnakes,
//   lightning

// };

// Modes setupForModes = {
//   none,
//   // starsSetup,
//   //spiralSetup,
//   ambientAllRainbowSetup,
//   ambientRedCycleSetup,
//   gyroSetup,
//   fireSetup,
//   fireSensorSetup,
//   simpleSensorSetup,
//   ringAudioSetup,
//   none,
//   twoRingAudioSetup,
//   none,
//   none,
//   none,
//   colorWheelUpDownSetup,
//   segmentTurningSetup,
//   none,
//   rainbowSparksSetup,
//   randomSparksSetup,
//   sparksSetup,
//   sparksAndRainbowSetup,
//   threeSnakesSetup,
//   lightningSetup
// };

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
  delay(1000); // if we fucked it up - great idea by fastled :D

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

void checkPotentiometer()
{
  static uint16_t potentiometer = 0;
  potentiometer = analogRead(POTENTIOMETER_PIN);
  uint8_t brightness = potentiometer / 4;
  if (usePotentiometer == 1)
  {
    LEDS.setBrightness(brightness);
    // eyes.setBrightness(brightness);
    // fiberHead.setBrightness(brightness);
    // fiberTail.setBrightness(brightness);
  }
}

void checkButtons()
{
  if (pad.readSwitches())
  {
    for (uint8_t i = 0; i < 12; i++)
    {

      if (pad.justPressed(i))
      {
        Serial.print("BUTTON ");
        Serial.println(i);
        pad.setLED(i);
        if (i == 3)
        {
          nextMode(1);
        }
        if (i == 7)
        {
          nextMode(-1);
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

#define TOP 30

void simpleAudio()
{
  // clear();
  audioUpdate();
  int height;
  static uint8_t hue = 0;
  hue++;
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  Serial.print("Audio: ");
  Serial.print(lvl);
  Serial.print(" ");
  Serial.print(minLvlAvg);
  Serial.print(" ");
  Serial.println(maxLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;

  // for (int i = 0; i < NUM_STRIPS; i++)
  // {
  //   for (int j = 0; j < height; j++)
  //   {
  //     // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 255, 0);
  //     leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(hue + j, 255, 255);
  //   }
  // }
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
      if (j % 4 == 0)
      {
        c = CRGB::Green;
      }
      if (j % 4 == 1)
      {
        c = CRGB::Blue;
      }
      if (j % 4 == 2)
      {
        c = CRGB::Red;
      }
      if (j % 4 == 3)
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

uint16_t fiberleft(uint8_t n)
{
  return NUM_LEDS_PER_STRIP * 2 + 100 + n;
}

uint16_t fiberright(uint8_t n)
{
  return NUM_LEDS_PER_STRIP + 95 + n;
}

void fibertest()
{

  leds[fiberleft(0)] = CRGB::Green;
  leds[fiberleft(1)] = CRGB::Blue;
  leds[fiberleft(2)] = CRGB::Red;
  leds[fiberleft(3)] = CRGB::Yellow;

  leds[fiberright(0)] = CRGB::Green;
  leds[fiberright(1)] = CRGB::Blue;
  leds[fiberright(2)] = CRGB::Red;
  leds[fiberright(3)] = CRGB::Yellow;
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

void colorWheelPulsingSetup() {}
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

  // EVERY_N_MILLISECONDS(20) { hue++; }
  // c.setHSV(hue, 255, 255);
  if (buttonState[EYE_STATE] == 1)
  {
    eyes.setBrightness(80);
  }
  else
  {
    eyes.setBrightness(0);
  }

  CRGB c(255, 0, 0);
  // static int8_t d = 1;
  // static int8_t v = 10;
  // EVERY_N_MILLISECONDS(50)
  // {
  //   v += d;
  //   if (v > 20)
  //     d = -1;
  //   if (v < 1)
  //     d = 1;
  // }
  // c.setHSV(30 + v, 70, 250);
  eyes.setPixelColor(0, c.r, c.g, c.b);
  eyes.show();
  // fiberHead.setPixelColor(0, c.r, c.g, c.b);
  // fiberTail.setPixelColor(0, c.r, c.g, c.b);
  // fiberHead.show();
  // fiberTail.show();
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
