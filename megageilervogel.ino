#include <FastLED.h>
#include <SoftwareSerial.h>
#include <Adafruit_Pixie.h>
#include "Adafruit_Trellis.h"

// sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include <i2c_t3.h>

// #include <Audio.h>

// #include <SPI.h>
// #include <SerialFlash.h>
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define POTENTIOMETER_PIN 16
#define MIC_PIN 17

// motion sensor
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(&Wire2, 1000);

// button board
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet pad = Adafruit_TrellisSet(&matrix0);

// 3W pixels
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

// leds per strip
uint8_t ledsPerStrip[] = {68, 68, 100, 95};

// buttons
#define EYE_STATE 15
// #define FIBER_HEAD_STATE 13
// #define FIBER_TAIL_STATE 14
// #define FIBER_WING_STATE 15

// active or not active
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
    {none, none},
    {fiberPulse, fiberPulseSetup},
    {fiberBlink, fiberBlinkSetup},
    {simpleAudio, simpleAudioSetup}, // y

    {colorWheelPulsing, colorWheelPulsingSetup},
    // {ftest, none},
    // {fireNoise, fireNoiseSetup},
    // {betterAudio, betterAudioSetup},
    {pride, prideSetup},         // y
    {lerpTest, lerpTestSetup},   // y
    {accelLerp, accelLerpSetup}, // y

    // {juggle, juggleSetup},
    // {flash2, flash2Setup},
    // {fire, fireSetup},
    // {none, none},
    {bpm, bpmSetup}, // y
    // {randomBluePixels, randomBluePixelsSetup}, //
    {fireWithPotentiometer, fireWithPotentiometerSetup}, // y
    {sinelon, sinelonSetup},                             // ?
    // {fiberMode, fiberModeSetup},
    // {rainbowSparks, rainbowSparksSetup},
    // {sparks, sparksSetup},
    {runner, none},
    {iceSparks, iceSparksSetup},
};

// AudioInputAnalog adc1(17);
// AudioAnalyzePeak peak1;
// AudioConnection patchCord1(adc1, peak1);

// Debug stuff to count LEDS properly
static int globalP = 0;

// the setup routine runs once when you press reset:
void setup()
{
  Serial.begin(115200);

  // init button pads and flash them once
  pad.begin(0x70);
  for (uint8_t i = 0; i < 16; i++)
  {
    pad.setLED(i);
    pad.writeDisplay();
    delay(25);
  }
  for (uint8_t i = 0; i < 16; i++)
  {
    pad.clrLED(i);
    pad.writeDisplay();
    delay(25);
  }
  // init motion sensor
  if (lsm.begin())
  {
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  }
  else
  {
    Serial.println("LSM SENSOR ERROR!");
  }

  // AudioMemory(4);

  // initialize the digital pin as an output.
  pinMode(TEENSY_LED, OUTPUT);

  // init 3w leds serial connection
  pixieSerial.begin(115200);
  pixieSerial2.begin(115200);
  pixieSerial3.begin(115200);

  // init all other leds
  FastLED.addLeds<WS2811_PORTD, NUM_STRIPS>(leds, NUM_LEDS_PER_STRIP);
  // //.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  // FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  delay(10); // if we fucked it up - great idea by fastled :D

  // start with mode number 0
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
    // Serial.println(currentMode);
    globalP++;
    Serial.print("CURRENT POS");
    Serial.println(globalP);
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
    // if (pad.justPressed(0))
    // {
    //   nextMode(1);
    //   return;
    // }

    //  debug
    // if (pad.justPressed(0))
    // {
    //   globalP = 0;
    // }
    // if (pad.justPressed(1))
    // {
    //   globalP++;
    // }
    // if (pad.justPressed(2))
    // {
    //   globalP--;
    //   if (globalP < 0)
    //   {
    //     globalP = 0;
    //   }
    // }
    // Serial.println(globalP);
    // return;
    // buttons with a direct mode mapping
    for (uint8_t i = 0; i < 14; i++)
    {

      if (pad.justPressed(i))
      {
        // Serial.print("BUTTON ");
        // Serial.println(i);
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
        if (i == 12)
        {
          // globalP++;
          flash();
        }
        else if (i == 13)
        {
          // globalP = 0;
          strobo();
        }
        else if (i < 12)
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

  for (uint8_t i = 14; i < 16; i++)
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
  shouldClear = false;
}

void simpleAudio()
{
  fadeToBlackBy(leds, NUM_LEDS, 70);
  // clear();
  audioUpdate();
  int height;
  static uint8_t hue = 0;
  hue++;
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)
    height = 0; // Clip output
  else if (height > TOP)
    height = TOP;

  int f2 = height * 5;

  leftWingLinear(f2, CHSV(hue, 255, 255));
  rightWingLinear(f2, CHSV(hue, 255, 255));
  bodyFront(f2, CHSV(hue, 255, 255));
  bodyBack(f2, CHSV(hue, 255, 255));

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

// looking from behind
// left:
//   strip outside - 0
//     front: 1-45 (first pixel is dead), (running outwards)
//     up-down  46-71 (running outwards)
//     down-up: 72-94 (running inwards)
//     fiber: 95-98
//   strip  inside - 1 (+120)
//     down-up: 0-31 (running outwards)
//     top-down 31-62 (running inwards)
// right:
//   strip outside - 2 (+240)
//     front: 0-44 (running outwards)
//     up-down:  45-71 (going outwards)
//     down-up: 72-99 (running inwards)
//     fiber: 100-103
//   strip inside -  3 (+360)
//     up-down: 0-33 (going outwards)
//     down->up: 34-67 (running inwards)
// body:
//   strip 4 (+480)
//     back 0-35
//     front 36-67

#define LW_FRONT_START 1
#define LW_FRONT_END 45
#define LW_S1A_START 46
#define LW_S1A_END 71
#define LW_S1B_START 72
#define LW_S1B_END 94
#define LW_S2A_START 0 + NUM_LEDS_PER_STRIP
#define LW_S2A_END 31 + NUM_LEDS_PER_STRIP
#define LW_S2B_START 32 + NUM_LEDS_PER_STRIP
#define LW_S2B_END 62 + NUM_LEDS_PER_STRIP

#define RW_FRONT_START (0 + (NUM_LEDS_PER_STRIP * 2))
#define RW_FRONT_END (44 + (NUM_LEDS_PER_STRIP * 2))
#define RW_S1A_START (45 + (NUM_LEDS_PER_STRIP * 2))
#define RW_S1A_END (71 + (NUM_LEDS_PER_STRIP * 2))
#define RW_S1B_START (72 + (NUM_LEDS_PER_STRIP * 2))
#define RW_S1B_END (99 + (NUM_LEDS_PER_STRIP * 2))
#define RW_S2A_START (0 + (NUM_LEDS_PER_STRIP * 3))
#define RW_S2A_END (33 + (NUM_LEDS_PER_STRIP * 3))
#define RW_S2B_START (34 + (NUM_LEDS_PER_STRIP * 3))
#define RW_S2B_END (67 + (NUM_LEDS_PER_STRIP * 3))

#define BODY_FRONT_START (36 + NUM_LEDS_PER_STRIP * 4)
#define BODY_FRONT_END (67 + NUM_LEDS_PER_STRIP * 4)
#define BODY_BACK_START (0 + NUM_LEDS_PER_STRIP * 4)
#define BODY_BACK_END (35 + NUM_LEDS_PER_STRIP * 4)

#define LW_LERP_OFFSET 100.0
#define RW_LERP_OFFSET 100.0
// 5 leds per wing at the same position (0-255)
void leftWingLinear(uint8_t x, CRGB c)
{
  leds[lerp8by8(LW_FRONT_START, LW_FRONT_END - 15, x)] += c;
  if (float(x) > LW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
    leds[lerp8by8(LW_S1A_START, LW_S1A_END, xx)] += c;
    leds[lerp8by8(LW_S1B_END, LW_S1B_START, xx)] += c; // starts later ... skip the first 100
  }
  leds[lerp8by8(LW_S2A_START, LW_S2A_END, x)] += c;
  leds[lerp8by8(LW_S2B_END, LW_S2B_START, x)] += c;
}
void rightWingLinear(uint8_t x, CRGB c)
{
  leds[lerp16by8(RW_FRONT_START, RW_FRONT_END - 15, x)] += c;
  if (float(x) > RW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - RW_LERP_OFFSET) * (255.0 / (255.0 - RW_LERP_OFFSET)));
    leds[lerp16by8(RW_S1A_START, RW_S1A_END, xx)] += c;
    leds[lerp16by8(RW_S1B_END, RW_S1B_START, xx)] += c; // starts later ... skip the first 100
  }
  leds[lerp16by8(RW_S2A_START, RW_S2A_END, x)] += c;
  leds[lerp16by8(RW_S2B_END, RW_S2B_START, x)] += c;
}

void bodyFront(uint8_t x, CRGB c)
{
  leds[lerp16by8(BODY_FRONT_START, BODY_FRONT_END, x)] += c;
}
void bodyBack(uint8_t x, CRGB c)
{
  leds[lerp16by8(BODY_BACK_END, BODY_BACK_START, x)] += c;
}

void accelLerpSetup()
{
  currentDelay = 5;
}
#define LERP_ACCEL_START 40

void accelLerp()
{

  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float mv = accel.acceleration.x + (9.81 + 0.9);
  // Serial.println(mv);
  static uint8_t hue1 = 0;
  hue1++;
  uint8_t hue2 = hue1;

  uint8_t f2 = uint8_t(LERP_ACCEL_START + (mv * 8.0));

  for (int i = 0; i < f2; i += 5)
  {
    leftWingLinear(i, CHSV(hue2 + 100, 255, 255));
    rightWingLinear(i, CHSV(hue2 + 100, 255, 255));
    bodyFront(i, CHSV(hue2 + 100, 255, 255));
    bodyBack(i, CHSV(hue2 + 100, 255, 255));
  }

  if (mv > 10)
  {
    strobo();
  }
}

void lerpTestSetup()
{
  currentDelay = 10;
  shouldClear = false;
}
void lerpTest()
{
  static uint8_t hue1 = 0;
  hue1++;
  uint8_t hue2 = hue1;

  fadeToBlackBy(leds, NUM_LEDS, 25);

  // for (int i = 0; i < 2; i++)
  // {

  // uint8_t f = ease8InOutCubic(p + i);
  uint8_t f = beatsin8(100, 0, 255);
  leftWingLinear(f, CHSV(hue1, 255, 255));
  rightWingLinear(f, CHSV(hue1, 255, 255));
  uint8_t f2 = beatsin8(50, 20, 220);
  leftWingLinear(f2, CHSV(hue2 + 100, 255, 255));
  rightWingLinear(f2, CHSV(hue2 + 100, 255, 255));
  bodyFront(f2, CHSV(hue2 + 100, 255, 255));
  bodyBack(f2, CHSV(hue2 + 100, 255, 255));

  // }
}

void ftest()
{
  clear();
  CRGB stripColor[5] = {CRGB::Green, CRGB::Red, CRGB::Blue, CRGB::Yellow, CRGB::Orange};
  for (int i = 0; i < 5; i++)
  {
    leds[globalP + stripoffset(i)] = stripColor[i];

    // CRGB c;
    // // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    // // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    // for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
    // // for (int j = 0; j < globalP; j++)
    // {
    //   if (i % 4 == 0)
    //   {
    //     c = CRGB::Green;
    //   }
    //   if (i % 4 == 1)
    //   {
    //     c = CRGB::Blue;
    //   }
    //   if (i % 4 == 2)
    //   {
    //     c = CRGB::Red;
    //   }
    //   if (i % 4 == 3)
    //   {
    //     c = CRGB::Yellow;
    //   }

    //   leds[j + stripoffset(i)] = c;
    // }
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

uint16_t fiberright(int8_t n)
{
  return NUM_LEDS_PER_STRIP * 2 + 100 + n;
}

uint16_t fiberleft(uint8_t n)
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
  currentDelay = 40;
}
void colorWheelPulsing()
{
  static uint8_t hue = 0;
  uint8_t pulse = beatsin8(60, 40, 255);
  hue++;

  hue++;
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(5 * i + hue, 240, pulse);
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
    leds[random(0, NUM_LEDS)] = CHSV(170 + (random(20) - 10), random(255), 250);
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
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  // float mv = accel.acceleration.x + accel.acceleration.y + accel.acceleration.z + 9.81;
  float mv = accel.acceleration.x + (9.81 + 0.9);
  // Serial.println(mv);

  if (mv > 10)
  {
    strobo();
  }

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
    eyes.setPixelColor(0, 255, 255, 255);
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

void reportSensor()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  // print out accelleration data
  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" ");
  Serial.print("  \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" ");
  Serial.print("  \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println("  \tm/s^2");

  // print out gyroscopic data
  Serial.print("Gyro  X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" ");
  Serial.print("  \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" ");
  Serial.print("  \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println("  \tdps");

  // print out temperature data
  Serial.print("Temp: ");
  Serial.print(temp.temperature);
  Serial.println(" *C");

  Serial.println("**********************\n");
}

void fireNoiseSetup()
{
  currentDelay = 20;
  shouldClear = false;
}

CRGBPalette16 fireNoisePal = CRGBPalette16(
    CRGB::Black, CRGB::Black, CRGB::Black, CHSV(0, 255, 4),
    CHSV(0, 255, 8), CRGB::Red, CRGB::Red, CRGB::Red,
    CRGB::DarkOrange, CRGB::Orange, CRGB::Orange, CRGB::Orange,
    CRGB::Yellow, CRGB::Yellow, CRGB::Gray, CRGB::Gray);

uint32_t xscale = 20; // How far apart they are
uint32_t yscale = 3;  // How fast they move

void fireNoise()
{

  uint8_t index = 0;

  for (int i = 0; i < NUM_LEDS; i++)
  {
    index = inoise8(i * xscale, millis() * yscale * NUM_LEDS / 255);                                       // X location is constant, but we move along the Y at the rate of millis()
    leds[i] = ColorFromPalette(fireNoisePal, min(i * (index) >> 6, 255), i * 255 / NUM_LEDS, LINEARBLEND); // With that value, look up the 8 bit colour palette value and assign it to the current LED.
  }

  // // tail
  // byte colorindex = scale8(heat[NUM_LEDS_PER_STRIP], 240);
  // CRGB c = ColorFromPalette(firePal, colorindex);
  // fiberTail.setPixelColor(0, c.r, c.g, c.b);
  // fiberTail.setBrightness(255);
  // fiberTail.show();

  // // eyes
  // // if (buttonState[EYE_STATE] == 1)
  // // {
  // byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  // CRGB cc = ColorFromPalette(firePal, ci);

  // // eyes.setBrightness(currentBrightness / 3);
  // eyes.setBrightness(100);
  // eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // // }
  // // else
  // // {
  // //   eyes.setBrightness(0);
  // // }

  // eyes.show();

  // // map to pixels
  // for (int j = 0; j < NUM_LEDS; j++)
  // {
  //   byte colorindex = scale8(heat[j], 240);
  //   CRGB color = ColorFromPalette(firePal, colorindex);
  //   leds[j] = color;
  // }

  // void 	fill_noise8 (CRGB *leds, int num_leds, uint8_t octaves, uint16_t x, int scale, uint8_t hue_octaves, uint16_t hue_x, int hue_scale, uint16_t time)
}

void prideSetup()
{
  currentDelay = 20;
}

void pride()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  uint8_t sat8 = beatsin88(87, 220, 250);
  uint8_t brightdepth = beatsin88(341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88(203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16; //gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis;
  sLastMillis = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88(400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16 += brightnessthetainc16;
    uint16_t b16 = sin16(brightnesstheta16) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV(hue8, sat8, qadd8(bri8, 100));

    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS - 1) - pixelnumber;

    nblend(leds[pixelnumber], newcolor, 64);
  }
}

// void betterAudioSetup()
// {
//   currentDelay = 10;
//   shouldClear = false;
// }

// elapsedMillis audioFps = 0;

// void betterAudio()
// {
//   static uint8_t hue = 0;
//   fadeToBlackBy(leds, NUM_LEDS, 25);
//   hue++;
//   if (audioFps > 25)
//   {
//     if (peak1.available())
//     {
//       audioFps = 0;
//       int monoPeak = peak1.read();
//       Serial.println(monoPeak);
//       for (int cnt = 0; cnt < monoPeak; cnt++)
//       {
//         leftWingLinear(cnt, CHSV(hue, 255, 255));
//       }
//     }
//   }
// }

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
  // EVERY_N_MILLISECONDS(250) { reportSensor(); }
  // EVERY_N_MILLISECONDS(100) { simpleAudio(); }
  // colorWheel();
  // runner();
  // leds[0] = CRGB(20, 20, 255);
  // leds[1] = CRGB(20, 255, 20);
  // leds[2] = CRGB(255, 20, 20);

  // FastLED.show();
  // FastLED.delay(1000);
}
