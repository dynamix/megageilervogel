// animation ideas todo
// - heartbeat
// - thunder mode
// #define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include <ADC.h>
#include <Audio.h>
#include <SoftwareSerial.h>
#include <Adafruit_Pixie.h>
#include "Adafruit_Trellis.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <math.h>

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define POTENTIOMETER_PIN 37
#define MIC_PIN 17

// analog - digital
ADC *adc = new ADC();

// motion sensor
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(&Wire2, 1000);

// button board
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet pad = Adafruit_TrellisSet(&matrix0);
#define PAD_CONNECTED false

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
#define MAX_BRIGHTNESS 55
#define TEENSY_LED 13

CRGB leds[NUM_LEDS];

class Segment
{
  int start, end;

public:
  Segment(int s, int e)
  {
    start = s;
    end = e;
  }

  uint16_t lerp(uint8_t x)
  {
    return lerp16by8(start, end, x);
  }
  void lerpTo(uint8_t x, CRGB c)
  {
    for (int i = start; i < lerp(x); i++)
    {
      leds[i] = c;
    }
  }
};

typedef void (*Mode[2])(void);

// leds per strip
uint8_t ledsPerStrip[] = {68, 68, 100, 95};

// buttons
#define EYE_STATE 15

// active or not active
uint8_t buttonState[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// global state
int8_t currentMode = 0;
int8_t previousMode = 0;
uint8_t currentBrightness = MAX_BRIGHTNESS;
uint16_t currentDelay = 0; // delay between frames
uint8_t shouldClear = 1;   // clear all leds between frames
uint8_t shouldShow = 1;
uint8_t usePixies = 1;        // pixies on
uint8_t usePotentiometer = 1; // use potentiometer to get brightness value for next frame

// Debug stuff to count LEDS properly
static int globalP = 0;

void none() {}

Mode modes[] = {
    {wingBeat, wingBeatSetup},
    {slowBeat, slowBeatSetup},
    {sameBeat, sameBeatSetup},
    {fibertest, none},
    {wingTest, none},
    {newAudio, newAudioSetup},       // y
    {heartbeat, heartbeatSetup},     // y
    {betterAudio, betterAudioSetup}, // y

    {lerpTest, lerpTestSetup}, // y
    // {ftest, none},

    // {wingTest, none},

    // {simpleAudio, simpleAudioSetup}, // y

    // {wingTest, none},
    // {linearTest, none},
    {pixieTest, pixieTestSetup},
    {ftest, none},
    // {fiberPulse, fiberPulseSetup},
    // {none, none},
    {pride, prideSetup}, // y
    // {sparks, sparksSetup},
    {flashTest, flashTestSetup},
    {rainbowSparks, rainbowSparksSetup},
    {randomBluePixels, randomBluePixelsSetup},
    // {none, none},
    {fiberBlink, fiberBlinkSetup},
    // {simpleAudio, simpleAudioSetump}, // y

    {colorWheelPulsing, colorWheelPulsingSetup},
    // {fireNoise, fireNoiseSetup},
    // {betterAudio, betterAudioSetup},
    {pride, prideSetup},         // y
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

#include "libs/util.cpp"

// the setup routine runs once when you press reset:
void setup()
{
  while (!Serial && (millis() <= 2000))
    ; // Wait for Serial interface
  Serial.begin(115200);

  Serial.println("boot bird");
  Serial.println("LED init");
  // init all other leds
  FastLED.addLeds<WS2811_PORTD, NUM_STRIPS>(leds, NUM_LEDS_PER_STRIP);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 5000);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  clear();
  FastLED.clear();
  FastLED.show();

  if (PAD_CONNECTED)
  {
    Serial.println("pad init");
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
  }
  else
  {
    Serial.println("pad disabled!");
  }
  Serial.println("motion init");
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

  // initialize the digital pin as an output.
  pinMode(TEENSY_LED, OUTPUT);

  Serial.println("3W LED init");

  // init 3w leds serial connection
  pixieSerial.begin(115200);
  pixieSerial2.begin(115200);
  pixieSerial3.begin(115200);

  adc->setAveraging(16, ADC_1);                                    // set number of averages
  adc->setResolution(16, ADC_1);                                   // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED, ADC_1);     // change the sampling speed
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1);
  adc->adc1->recalibrate();

  AudioMemory(8);

  // adc->setAveraging(16, ADC_1);
  // adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_1);
  // adc->setResolution(10, ADC_1); // adc->setResolution(10, ADC_1):

  delay(10); // if we fucked it up - great idea by fastled :D

  // start with mode number 0
  modes[0][1]();
}

void PrintE()
{
  float e, n;
  int b, bands, bins, count = 0, d;

  bands = 30; // Frequency bands; (Adjust to desired value)
  bins = 512; // FFT bins; (Adjust to desired value)

  e = FindE(bands, bins); // Find calculated E value
  if (e)
  {                                  // If a value was returned continue
    Serial.printf("E = %4.4f\n", e); // Print calculated E value
    for (b = 0; b < bands; b++)
    { // Test and print the bins from the calculated E
      n = pow(e, b);
      d = int(n + 0.5);

      Serial.printf("%4d ", count); // Print low bin
      count += d - 1;
      Serial.printf("%4d\n", count); // Print high bin
      ++count;
    }
  }
  else
    Serial.println("Error\n"); // Error, something happened
}

float FindE(int bands, int bins)
{
  float increment = 0.1, eTest, n;
  int b, count, d;

  for (eTest = 1; eTest < bins; eTest += increment)
  { // Find E through brute force calculations
    count = 0;
    for (b = 0; b < bands; b++)
    { // Calculate full log values
      n = pow(eTest, b);
      d = int(n + 0.5);
      count += d;
    }
    if (count > bins)
    {                     // We calculated over our last bin
      eTest -= increment; // Revert back to previous calculation increment
      increment /= 10.0;  // Get a finer detailed calculation & increment a decimal point lower
    }
    else if (count == bins)    // We found the correct E
      return eTest;            // Return calculated E
    if (increment < 0.0000001) // Ran out of calculations. Return previous E. Last bin will be lower than (bins-1)
      return (eTest - increment);
  }
  return 0; // Return error 0
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
    nextMode(1);
  }
}

static uint16_t potentiometer = 0;

void checkPotentiometer()
{
  potentiometer = adc->analogRead(POTENTIOMETER_PIN, ADC_1);

  uint8_t brightness = map(potentiometer, 0, 65535, 0, 255); // potentiometer / 4;
  // Serial.println(brightness);
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

void flashTestSetup()
{
  // currentDelay = 2;
  currentDelay = 5;
  shouldClear = false;
}

void flashTest()
{
  // clear();
  fadeToBlackBy(leds, NUM_LEDS, 2);
  EVERY_N_MILLISECONDS(1000)
  {
    for (int j = 0; j < 20; j++)
    {
      int x = random(0, NUM_LEDS);
      leds[x] = CRGB::White;
    }
  }
}
void randomBluePixelsSetup()
{
  // currentDelay = 2;
  currentDelay = 0;
  shouldClear = false;
}

void randomBluePixels()
{
  // clear();
  fadeToBlackBy(leds, NUM_LEDS, 40);
  for (int j = 0; j < 3; j++)
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

#define LW_FRONT_START 0
#define LW_FRONT_PEAK 28
#define LW_FRONT_END 46

#define LW_MIDDLE_START 0 + NUM_LEDS_PER_STRIP
#define LW_MIDDLE_PEAK 33 + NUM_LEDS_PER_STRIP
#define LW_MIDDLE_END 68 + NUM_LEDS_PER_STRIP

#define LW_BACK_START 46
#define LW_BACK_PEAK 73
#define LW_BACK_END 101

#define RW_FRONT_START 0 + (NUM_LEDS_PER_STRIP * 2)
#define RW_FRONT_PEAK 28 + (NUM_LEDS_PER_STRIP * 2)
#define RW_FRONT_END 45 + (NUM_LEDS_PER_STRIP * 2)

#define RW_MIDDLE_START 0 + (NUM_LEDS_PER_STRIP * 3)
#define RW_MIDDLE_PEAK 31 + (NUM_LEDS_PER_STRIP * 3)
#define RW_MIDDLE_END 68 + (NUM_LEDS_PER_STRIP * 3)

#define RW_BACK_START 45 + (NUM_LEDS_PER_STRIP * 2)
#define RW_BACK_PEAK 70 + (NUM_LEDS_PER_STRIP * 2)
#define RW_BACK_END 96 + (NUM_LEDS_PER_STRIP * 2)

#define BODY_START 0 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_MIDDLE 25 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_END 50 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_AND_HEAD_END 70 + (NUM_LEDS_PER_STRIP * 4)

#define BODY_FRONT_START 36 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_FRONT_END 50 + (NUM_LEDS_PER_STRIP * 4)

#define BODY_BACK_START 0 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_BACK_END 36 + (NUM_LEDS_PER_STRIP * 4)

#define HEAD_START 50 + (NUM_LEDS_PER_STRIP * 4)
#define HEAD_END 70 + (NUM_LEDS_PER_STRIP * 4)

#define HEAD_LEFT_START 50 + (NUM_LEDS_PER_STRIP * 4)
#define HEAD_LEFT_END 60 + (NUM_LEDS_PER_STRIP * 4)

#define HEAD_RIGHT_START 60 + (NUM_LEDS_PER_STRIP * 4)
#define HEAD_RIGHT_END 70 + (NUM_LEDS_PER_STRIP * 4)

Segment lwFront = Segment(LW_FRONT_START, LW_FRONT_END);
Segment lwFrontInner = Segment(LW_FRONT_START, LW_FRONT_PEAK);
Segment lwFrontOuter = Segment(LW_FRONT_PEAK, LW_FRONT_END);

Segment lwMiddleTop = Segment(LW_MIDDLE_START, LW_MIDDLE_PEAK);
Segment lwMiddleBottom = Segment(LW_MIDDLE_PEAK, LW_MIDDLE_END);

Segment lwBackTop = Segment(LW_BACK_START, LW_BACK_PEAK);
Segment lwBackBottom = Segment(LW_BACK_PEAK, LW_BACK_END);

Segment rwFront = Segment(RW_FRONT_START, RW_FRONT_END);
Segment rwFrontInner = Segment(RW_FRONT_START, RW_FRONT_PEAK);
Segment rwFrontOuter = Segment(RW_FRONT_PEAK, RW_FRONT_END);
Segment rwMiddleTop = Segment(RW_MIDDLE_PEAK, RW_MIDDLE_END);
Segment rwMiddleBottom = Segment(RW_MIDDLE_START, RW_MIDDLE_PEAK);
Segment rwBackTop = Segment(RW_BACK_START, RW_BACK_PEAK);
Segment rwBackBottom = Segment(RW_BACK_PEAK, RW_BACK_END);

Segment bodyFront = Segment(BODY_FRONT_START, BODY_FRONT_END);
Segment bodyBack = Segment(BODY_BACK_START, BODY_BACK_END);

Segment headLeft = Segment(HEAD_LEFT_START, HEAD_LEFT_END);
Segment headRight = Segment(HEAD_RIGHT_START, HEAD_RIGHT_END);

void fill(uint16_t from, uint16_t to, CRGB c)
{
  fill_solid(leds + from, to - from, c);
}

void wingTest()
{

  lwFrontInner.lerpTo(255, CRGB::Green);
  lwFrontOuter.lerpTo(255, CRGB::Red);

  // lwMiddleTop.lerpTo(255, CRGB::Blue);

  // lwBackTop.lerpTo(255, CRGB::Yellow);

  // rwMiddleTop.lerpTo(255, CRGB::Blue);
  // rwMiddleBottom.lerpTo(255, CRGB::Green);

  // rwBackTop.lerpTo(255, CRGB::Yellow);
  // rwBackBottom.lerpTo(255, CRGB::Green);

  // fill(LW_FRONT_START, LW_FRONT_PEAK, CRGB::Green);
  // fill(LW_FRONT_PEAK, LW_FRONT_END, CRGB::Red);

  // fill(LW_MIDDLE_START, LW_MIDDLE_PEAK, CRGB::Yellow);
  // fill(LW_MIDDLE_PEAK, LW_MIDDLE_END, CRGB::Red);

  // fill(LW_BACK_START, LW_BACK_PEAK, CRGB::Red);
  // fill(LW_BACK_PEAK, LW_BACK_END + 20, CRGB::Yellow);

  // fill(RW_FRONT_START, RW_FRONT_PEAK, CRGB::Green);
  // fill(RW_FRONT_PEAK, RW_FRONT_END, CRGB::Red);

  // fill(RW_MIDDLE_START, RW_MIDDLE_PEAK, CRGB::Green);
  // fill(RW_MIDDLE_PEAK, RW_MIDDLE_END, CRGB::Red);

  // fill(RW_BACK_START, RW_BACK_PEAK, CRGB::Red);
  // fill(RW_BACK_PEAK, RW_BACK_END + 20, CRGB::Green);

  // fill(BODY_FRONT_START, BODY_FRONT_END, CRGB::Red);
  // fill(BODY_BACK_START, BODY_BACK_END, CRGB::Green);

  // fill(HEAD_LEFT_START, HEAD_LEFT_END, CRGB::Green);
  // fill(HEAD_RIGHT_START, HEAD_RIGHT_END, CRGB::Blue);

  // fill(BODY_START, BODY_END, CRGB::Green);
  // fill(HEAD_START, HEAD_END, CRGB::Red);
}

// #define LW_S1A_START 46
// #define LW_S1A_END 71
// #define LW_S1B_START 72
// #define LW_S1B_END 94
// #define LW_S2A_START 0 + NUM_LEDS_PER_STRIP
// #define LW_S2A_END 31 + NUM_LEDS_PER_STRIP
// #define LW_S2B_START 32 + NUM_LEDS_PER_STRIP
// #define LW_S2B_END 62 + NUM_LEDS_PER_STRIP

// #define RW_FRONT_START (0 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_FRONT_END (44 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1A_START (45 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1A_END (71 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1B_START (72 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1B_END (99 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S2A_START (0 + (NUM_LEDS_PER_STRIP * 3))
// #define RW_S2A_END (33 + (NUM_LEDS_PER_STRIP * 3))
// #define RW_S2B_START (34 + (NUM_LEDS_PER_STRIP * 3))
// #define RW_S2B_END (67 + (NUM_LEDS_PER_STRIP * 3))

// #define BODY_FRONT_START (36 + NUM_LEDS_PER_STRIP * 4)
// #define BODY_FRONT_END (67 + NUM_LEDS_PER_STRIP * 4)
// #define BODY_BACK_START (0 + NUM_LEDS_PER_STRIP * 4)
// #define BODY_BACK_END (35 + NUM_LEDS_PER_STRIP * 4)

#define LW_LERP_OFFSET 100.0
#define RW_LERP_OFFSET 100.0
// 5 leds per wing at the same position (0-255)

void leftWingLinear(uint8_t x, CRGB c)
{
  leds[lerp8by8(LW_FRONT_START, LW_FRONT_PEAK, x)] += c;
  leds[lerp8by8(LW_MIDDLE_START, LW_MIDDLE_PEAK, x)] += c;
  leds[lerp8by8(LW_MIDDLE_END, LW_MIDDLE_PEAK, x)] += c;

  if (float(x) > LW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
    leds[lerp8by8(LW_BACK_START, LW_BACK_PEAK, xx)] += c;
    leds[lerp8by8(LW_BACK_END, LW_BACK_PEAK, xx)] += c;
  }
}

void rightWingLinear(uint8_t x, CRGB c)
{
  leds[lerp16by8(RW_FRONT_START, RW_FRONT_PEAK, x)] += c;
  leds[lerp16by8(RW_MIDDLE_START, RW_MIDDLE_PEAK, x)] += c;
  leds[lerp16by8(RW_MIDDLE_END, RW_MIDDLE_PEAK, x)] += c;

  if (float(x) > RW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - RW_LERP_OFFSET) * (255.0 / (255.0 - RW_LERP_OFFSET)));
    leds[lerp16by8(RW_BACK_START, RW_BACK_PEAK, xx)] += c;
    leds[lerp16by8(RW_BACK_END, RW_BACK_PEAK, xx)] += c;
  }
}

void linearTest()
{
  leftWingLinear(globalP * 4, CRGB::Red);
  rightWingLinear(globalP * 4, CRGB::Green);
}

// void bodyFront(uint8_t x, CRGB c)
// {
//   leds[lerp16by8(BODY_FRONT_START, BODY_FRONT_END, x)] += c;
// }
// void bodyBack(uint8_t x, CRGB c)
// {
//   leds[lerp16by8(BODY_BACK_END, BODY_BACK_START, x)] += c;
// }

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
    // bodyFront(i, CHSV(hue2 + 100, 255, 255));
    // bodyBack(i, CHSV(hue2 + 100, 255, 255));
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
  // bodyFront(f2, CHSV(hue2 + 100, 255, 255));
  // bodyBack(f2, CHSV(hue2 + 100, 255, 255));

  // }
}

void ftest()
{
  clear();
  CRGB stripColor[5] = {CRGB::Green, CRGB::Red, CRGB::Blue, CRGB::Yellow, CRGB::White};
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

// looking from front

uint16_t fiberright(int8_t n)
{
  // maybe +2
  return NUM_LEDS_PER_STRIP * 2 + 96 + n;
}

uint16_t fiberleft(uint8_t n)
{
  return 101 + n;
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

  leds[fiberleft(0)] = CRGB::Green;
  leds[fiberleft(1)] = CRGB::Blue;
  leds[fiberleft(2)] = CRGB::Red;
  leds[fiberleft(3)] = CRGB::Yellow;

  // leds[fiberright(4)] = CRGB::Yellow;
  // leds[fiberright(5)] = CRGB::Yellow;
  // leds[fiberright(6)] = CRGB::Yellow;
  // leds[fiberright(7)] = CRGB::Yellow;

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

void prideSetup()
{
  currentDelay = 50;
}

void pride()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  static uint8_t start = 0;
  start++;
  // EVERY_N_MILLISECONDS(60) { start++; }
  start = start % 2;

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

  for (uint16_t i = start; i < NUM_LEDS; i += 2)
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
  // CRGB c(0, 255 0);
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

void pixieTestSetup()
{
  usePixies = 1;
  currentDelay = 10;
  shouldClear = true;
}
void pixieTest()
{

  eyes.setPixelColor(0, 255, 0, 0);
  fiberTail.setPixelColor(0, 0, 255, 0);
  fiberHead.setPixelColor(0, 0, 0, 255);
  eyes.setBrightness(50);
  fiberTail.setBrightness(0);
  fiberHead.setBrightness(0);
  // EVERY_N_MILLISECONDS(500)
  // {

  // all three must be shown in this order - otherwise it will flicker
  fiberTail.show();
  fiberHead.show();
  eyes.show();
  // }
}

void fiberPulseSetup()
{
  usePixies = 1;
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
//     if (audioPeak.available())
//     {
//       audioFps = 0;
//       int monoPeak = audioPeak.read();
//       Serial.println(monoPeak);
//       for (int cnt = 0; cnt < monoPeak; cnt++)
//       {
//         leftWingLinear(cnt, CHSV(hue, 255, 255));
//       }
//     }
//   }
// }

// ---------------------------------
// Functional test
#include "effects/tests.cpp"

AudioInputAnalog adc1(MIC_PIN);
AudioAnalyzePeak audioPeak;
AudioConnection patchCord1(adc1, audioPeak);

// AudioAnalyzeRMS rms;
// AudioConnection patchCord3(adc1, rms);
// AudioAnalyzeFFT256 fft;
// AudioConnection patchCord2(adc1, fft);

void betterAudioSetup()
{
  currentDelay = 2; // 500Hz
  shouldClear = true;
}

#define AUDIO_SAMPLES 60 // samples for the mic buffer
#define MIN_DIST_AUDIO_LEVELS 0.1
#define MIN_AUDIO_LEVEL 0.01

float audioSamples[AUDIO_SAMPLES];
float minAudioAvg = 0;
float maxAudioAvg = 5.0;
byte audioSampleIdx = 0;

void audioUpdate(float sample)
{
  float min, max;

  if (sample < MIN_AUDIO_LEVEL)
    sample = 0;

  audioSamples[audioSampleIdx] = sample;
  if (++audioSampleIdx >= AUDIO_SAMPLES)
    audioSampleIdx = 0;

  min = max = audioSamples[0];
  for (uint8_t i = 1; i < AUDIO_SAMPLES; i++)
  {
    if (audioSamples[i] < min)
      min = audioSamples[i];
    else if (audioSamples[i] > max)
      max = audioSamples[i];
  }
  if ((max - min) < MIN_DIST_AUDIO_LEVELS)
    max = min + MIN_DIST_AUDIO_LEVELS;
  minAudioAvg = (minAudioAvg * (AUDIO_SAMPLES - 1) + min) / AUDIO_SAMPLES;
  maxAudioAvg = (maxAudioAvg * (AUDIO_SAMPLES - 1) + max) / AUDIO_SAMPLES;
}

// void leftWingLinearUpto(uint8_t x, CRGB c)
// {
//   leds[lerp8by8(LW_FRONT_START, LW_FRONT_PEAK, x)] += c;
//   leds[lerp8by8(LW_MIDDLE_START, LW_MIDDLE_PEAK, x)] += c;
//   leds[lerp8by8(LW_MIDDLE_END, LW_MIDDLE_PEAK, x)] += c;

//   if (float(x) > LW_LERP_OFFSET)
//   {
//     uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
//     leds[lerp8by8(LW_BACK_START, LW_BACK_PEAK, xx)] += c;
//     leds[lerp8by8(LW_BACK_END, LW_BACK_PEAK, xx)] += c;
//   }
// }

void betterAudio()
{
  // fadeToBlackBy(leds, NUM_LEDS, 5);

  fill(HEAD_LEFT_START, HEAD_LEFT_END, CRGB::Green);
  fill(HEAD_RIGHT_START, HEAD_RIGHT_END, CRGB::Blue);

  static uint8_t hue = 0;
  static float lastPeak = 0;
  hue++;

  hue %= 128;

  if (audioPeak.available())
  {
    float peak = audioPeak.read();
    audioUpdate(peak);

    if (peak < minAudioAvg)
    {
      peak = minAudioAvg;
    }

    int p = 128.0 * (peak - minAudioAvg) / (maxAudioAvg - minAudioAvg);

    lwFrontInner.lerpTo(p, CHSV(HUE_BLUE, 230, p));
    // lwMiddleTop.lerpTo(p, CHSV(HUE_BLUE + p, 230, p));
    // leftWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
    // leftWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
    rightWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
    // bodyFront(p, CHSV(HUE_BLUE + p, 230, p));
    // bodyBack(p, CHSV(HUE_BLUE + p, 230, p));

    // if (p > 210)
    // {
    //   fill(LW_BACK_PEAK, LW_BACK_END + 20, CRGB::Yellow);
    // }

    // if (p > 190)
    // {
    //   eyes.setPixelColor(0, 155, 0, 0);
    //   fiberTail.setPixelColor(0, 0, 255, 0);
    //   fiberHead.setPixelColor(0, 0, 0, 255);
    //   eyes.setBrightness(50);
    //   fiberTail.setBrightness(p);
    //   fiberHead.setBrightness(p);

    //   // all three must be shown in this order - otherwise it will flicker
    //   fiberTail.show();
    //   fiberHead.show();
    //   eyes.show();
    // }

    // if (peak > lastPeak)
    // {
    //   lastPeak = peak;
    // }
    // // leftWingLinear(peak * 255.0, CHSV(255, 255, 255));
    // // rightWingLinear(peak * 255.0, CHSV(255, 255, 255));

    // lastPeak *= 0.99;
  }
}

void audioUpdate2(float sample)
{
  float min, max;

  if (sample < MIN_AUDIO_LEVEL)
    sample = 0;

  audioSamples[audioSampleIdx] = sample;
  if (++audioSampleIdx >= AUDIO_SAMPLES)
    audioSampleIdx = 0;

  min = max = audioSamples[0];
  for (uint8_t i = 1; i < AUDIO_SAMPLES; i++)
  {
    if (audioSamples[i] < min)
      min = audioSamples[i];
    else if (audioSamples[i] > max)
      max = audioSamples[i];
  }
  if ((max - min) < MIN_DIST_AUDIO_LEVELS)
    max = min + MIN_DIST_AUDIO_LEVELS;
  minAudioAvg = (minAudioAvg * (AUDIO_SAMPLES - 1) + min) / AUDIO_SAMPLES;
  maxAudioAvg = (maxAudioAvg * (AUDIO_SAMPLES - 1) + max) / AUDIO_SAMPLES;
}

elapsedMicros audioSamplingTimer;
elapsedMillis audioShowTimer;
boolean beatDetected = false;

AudioAnalyzeRMS rms;
// AudioConnection patchCord2(adc1, rms);

AudioAnalyzeFFT1024 fft;
AudioConnection patchCord3(adc1, fft);

#define AUDIO_SAMPLES 100
int sampleCount = 0;

#define BEAT_BANDS 30
#define LONGTERM_SAMPLES 60
#define BEAT_COUNTER_SAMPLES 400
#define BEAT_AVG_SAMPLES 100

#define CUTOFF 3
#define DELTA_SAMPLES 300

#define MAX_TIME 200

float deltaSamples[BEAT_BANDS][DELTA_SAMPLES];
float deltas[BEAT_BANDS];
float longtermSamples[BEAT_BANDS][LONGTERM_SAMPLES];
float avgSamples[LONGTERM_SAMPLES];
float delta[BEAT_BANDS];
float totalShort[BEAT_BANDS];
float longtermAvg[BEAT_BANDS];
float band[BEAT_BANDS];

int count[BEAT_BANDS];
int beatSpread[MAX_TIME];
float c[BEAT_BANDS];
int beatCounter[BEAT_COUNTER_SAMPLES];
int beatAvg[BEAT_AVG_SAMPLES];

int bandMap[BEAT_BANDS][2] = {
    {0, 0},
    {1, 1},
    {2, 3},
    {4, 5},
    {6, 8},
    {9, 11},
    {12, 15},
    {16, 21},
    {22, 28},
    {29, 37},
    {38, 49},
    {50, 64},
    {65, 84},
    {85, 110},  // 3400 - 4400 (13)
    {111, 143}, // 4440 - 5720
    {144, 185},
    {186, 239},
    {240, 308},
    {309, 397},
    {398, 511}};

float totalBeat;
int longPos = 0;
int deltaPos = 0;
int beatCounterPos = 0;
int nextBeatCounter = 0;
int beat = 0;
int beatPos = 0;
float threshold = 0;
float predictiveInfluenceConstant = .1;
float predictiveInfluence;
int cyclePerBeatIntensity;
float standardDeviation;
int cyclesPerBeat;

void newAudioSetup()
{
  currentDelay = 0; // 500Hz
  // shouldClear = true;
  shouldClear = false;
  shouldShow = true;

  for (int i = 0; i < BEAT_BANDS; i += 1)
  {
    count[i] = 0;
    longtermAvg[i] = 0;
    delta[i] = 0;
    c[i] = 1.5;
  }
}

int amax(int *array, int size)
{
  int maxIndex = 0;
  int max = array[maxIndex];
  for (int i = 1; i < size; i++)
  {
    if (max < array[i])
    {
      max = array[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}
int amode(int *a, int size)
{
  int modeMap[size];
  int maxEl = a[0];
  int maxCount = 1;

  for (int i = 0; i < size; i++)
  {
    int el = a[i];
    if (modeMap[el] == 0)
    {
      modeMap[el] = 1;
    }
    else
    {
      modeMap[el]++;
    }

    if (modeMap[el] > maxCount)
    {
      maxEl = el;
      maxCount = modeMap[el];
    }
  }
  return maxEl;
}

void newAudio()
{

  fadeToBlackBy(leds, NUM_LEDS, 5);

  if (fft.available())
  {
    longPos++;
    if (longPos >= LONGTERM_SAMPLES)
      longPos = 0;
    deltaPos++;
    if (deltaPos >= DELTA_SAMPLES)
      deltaPos = 0;
    beatPos++;
    if (beatPos >= DELTA_SAMPLES)
      beatPos = 0;

    float currentAvg = 0;
    for (int i = 0; i < BEAT_BANDS; i++)
    {

      Serial.print(float(i), i > 9 ? 1 : 2);
      Serial.print(" ");
    }
    Serial.println("");

    for (int i = 0; i < BEAT_BANDS; i++)
    {
      band[i] = fft.read(bandMap[i][0], bandMap[i][1]);
      currentAvg += band[i];

      longtermAvg[i] = 0;
      for (int j = 0; j < LONGTERM_SAMPLES; j++)
      {
        longtermAvg[i] += longtermSamples[i][j];
      }

      longtermSamples[i][longPos] = band[i];
      if (band[i] >= 0.01)
      {
        Serial.print(band[i]);
        Serial.print(" ");
      }
      else
      {
        Serial.print("  -  ");
      }
    }
    currentAvg = currentAvg / float(BEAT_BANDS);
    Serial.println("");

    for (int i = 0; i < BEAT_BANDS; i++)
    {
      delta[i] = 0.0;
      longtermAvg[i] = longtermAvg[i] / float(LONGTERM_SAMPLES);

      // store a delta between the current value and the longterm avg per band
      deltaSamples[i][deltaPos] = pow(abs(longtermAvg[i] - band[i]), 2);

      // calculate the per band delta avg for all samples we have
      for (int j = 0; j < DELTA_SAMPLES; j += 1)
      {
        delta[i] += deltaSamples[i][j];
        // Serial.print(delta[i], 5);
        // Serial.print(" ");
      }
      // Serial.println();
      delta[i] = delta[i] / float(DELTA_SAMPLES);

      c[i] = 1.3 + constrain(map(delta[i], 0, 3000, 0, .4), 0, .4) +       //delta is usually bellow 2000
             map(constrain(pow(longtermAvg[i], .5), 0, 6), 0, 20, .3, 0) + //possibly comment this out, adds weight to the lower end
             map(constrain(count[i], 0, 15), 0, 15, 1, 0) -
             map(constrain(count[i], 30, 200), 30, 200, 0, .75);

      if (cyclePerBeatIntensity / standardDeviation > 3.5)
      {
        predictiveInfluence = predictiveInfluenceConstant * (1 - cos((float(nextBeatCounter) * TWO_PI) / float(cyclesPerBeat)));
        predictiveInfluence *= map(constrain(cyclePerBeatIntensity / standardDeviation, 3.5, 20), 3.5, 15, 1, 6);
        if (cyclesPerBeat > 10)
          c[i] = c[i] + predictiveInfluence;
      }
      c[i] = 1.5;
    }

    // avg over some frequency + calc global avg LONGTERM_SAMPLES
    // one bin is 43 HZ
    avgSamples[longPos] = fft.read(0, 425) / 425.0;
    float globalAvg = 0;
    for (int j = 0; j < LONGTERM_SAMPLES; j += 1)
      globalAvg += avgSamples[j];
    globalAvg = globalAvg / float(LONGTERM_SAMPLES);

    // Serial.print("global: ");
    // Serial.println(globalAvg, 8);

    beat = 0;
    for (int i = 0; i < BEAT_BANDS; i += 1)
    {
      if (band[i] > longtermAvg[i] * c[i] & count[i] > 7)
      {
        if (count[i] > 12 & count[i] < 200)
        {
          beatCounter[beatCounterPos % BEAT_COUNTER_SAMPLES] = count[i];
          beatCounterPos += 1;
        }
        count[i] = 0;
      }
    }

    // if there was a beat we reset the counter to zero
    for (int i = 0; i < BEAT_BANDS; i += 1)
      if (count[i] < 2)
        beat += 1;

    beatAvg[beatPos] = beat;
    for (int i = 0; i < BEAT_AVG_SAMPLES; i += 1)
      totalBeat += beatAvg[i];
    totalBeat = totalBeat / float(BEAT_AVG_SAMPLES);

    c[0] = 3.25 + map(constrain(nextBeatCounter, 0, 5), 0, 5, 5, 0);
    if (cyclesPerBeat > 10)
      c[0] = c[0] + .75 * (1 - cos((float(nextBeatCounter) * TWO_PI) / float(cyclesPerBeat)));

    c[0] = 1.5;

    threshold = constrain(c[0] * totalBeat + map(constrain(globalAvg, 0, 2), 0, 2, 4, 0), 5, 1000);
    // Serial.print(" c0: ");
    // Serial.print(c[0], 8);
    // Serial.print(" globalAvg: ");
    // Serial.print(globalAvg, 8);
    // Serial.print(" beat: ");
    // Serial.print(beat);
    // Serial.print(" threshold: ");
    // Serial.print(threshold, 8);
    // Serial.print(" nextBeat: ");
    // Serial.println(nextBeatCounter);

    if (beat > threshold & nextBeatCounter > 5)
    {
      // Serial.print(" c0: ");
      // Serial.print(c[0], 8);
      // Serial.print(" globalAvg: ");
      // Serial.print(globalAvg, 8);
      // Serial.print(" beats: ");
      // Serial.print(beat);
      // Serial.print(" threshold: ");
      // Serial.print(threshold, 8);
      // Serial.print(" nextBeat: ");

      // Serial.print(" total beat avg=");
      // Serial.print(totalBeat, 8);

      // Serial.print(" currentAvg: ");
      // Serial.print(currentAvg, 8);

      float d = currentAvg - globalAvg;
      // Serial.print(" d=");
      // Serial.print(d, 6);

      int p = constrain(d * 25500.0, 0, 255);
      // Serial.print(" P=");
      // Serial.print(p);

      lwFrontInner.lerpTo(150, CHSV(HUE_RED, 230, 255));
      rwFrontInner.lerpTo(150, CHSV(HUE_RED, 230, 255));
      for (int i = BODY_FRONT_START + 1; i < BODY_FRONT_END - 9; i++)
      {
        leds[i].setHSV(0, 255, 255);
      }
      // Serial.println(" => BEAT!");
      nextBeatCounter = 0;
    }

    for (int i = 0; i < MAX_TIME; i++)
      beatSpread[i] = 0;
    for (int i = 0; i < BEAT_COUNTER_SAMPLES; i++)
    {
      beatSpread[beatCounter[i]] += 1;
    }

    cyclesPerBeat = amode(beatCounter, BEAT_COUNTER_SAMPLES);
    if (cyclesPerBeat < 20)
      cyclesPerBeat *= 2;

    cyclePerBeatIntensity = amax(beatSpread, MAX_TIME);

    standardDeviation = 0;
    for (int i = 0; i < MAX_TIME; i++)
      standardDeviation += pow(BEAT_COUNTER_SAMPLES / MAX_TIME - beatSpread[i], 2);
    standardDeviation = pow(standardDeviation / MAX_TIME, .5);

    for (int i = 0; i < BEAT_BANDS; i += 1)
      count[i] += 1;
    deltaPos += 1;
    nextBeatCounter += 1;
    beatPos += 1;

    // Serial.print(audioSamplingTimer);
    // audioSamplingTimer = 0;
    // Serial.print("|FFT: ");
    // for (int i = 0; i < 20; i++)
    // {
    //   float n = fft.read(i);
    //   if (n >= 0.01)
    //   {
    //     Serial.print(n);
    //     Serial.print(" ");
    //   }
    //   else
    //   {
    //     Serial.print("  -  ");
    //   }
    // }
    // Serial.println();
  }

  // EVERY_N_MILLISECONDS(1000)
  // {
  //   Serial.print("fft=");
  //   Serial.print(fft.processorUsage());
  //   Serial.print(",");
  //   Serial.print(fft.processorUsageMax());
  //   Serial.print(" Memory: ");
  //   Serial.print(AudioMemoryUsage());
  //   Serial.print(",");
  //   Serial.print(AudioMemoryUsageMax());
  //   Serial.println("");
  // }

  // // a sample is ready roughly every 3ms - lets test that assumption
  // if (rms.available())
  // {
  //   float peak = rms.read();
  //   Serial.println("peak " + String(peak) + " t=" + String(audioSamplingTimer));
  //   audioSamplingTimer = 0;
  //   sampleCount++;

  //   if(sampleCount >= AUDIO_SAMPLES) {

  //   }
  // }

  // a sample is ready roughly every 3ms - lets test that assumption
  // if (audioSamplingTimer > 22)
  // {
  //   if (audioPeak.available())
  //   {
  //     audioSamplingTimer = 0;
  //     float peak = audioPeak.read();

  //   }
  // }
  // if (audioShowTimer > 10)
  // {
  //   audioShowTimer = 0;

  //   lwFrontInner.lerpTo(p, CHSV(HUE_BLUE, 230, p));
  // }
  // fadeToBlackBy(leds, NUM_LEDS, 5);

  // fill(HEAD_LEFT_START, HEAD_LEFT_END, CRGB::Green);
  // fill(HEAD_RIGHT_START, HEAD_RIGHT_END, CRGB::Blue);

  // if (audioPeak.available())
  // {
  //   float peak = audioPeak.read();
  //   audioUpdate(peak);

  //   if (peak < minAudioAvg)
  //   {
  //     peak = minAudioAvg;
  //   }

  //   int p = 128.0 * (peak - minAudioAvg) / (maxAudioAvg - minAudioAvg);

  //   lwFrontInner.lerpTo(p, CHSV(HUE_BLUE, 230, p));
  //   rightWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
  // }
}

void heartbeatSetup()
{
  currentDelay = 10;
  shouldClear = false;
  usePotentiometer = 0;
  LEDS.setBrightness(255);
  usePixies = true;
  shouldShow = true;
}

elapsedMillis timeElapsed;

void heartbeat()
{
  static int fadeSpeed = 5;
  fadeToBlackBy(leds, NUM_LEDS, fadeSpeed);
  static int b = 0;

  // uint8_t b = beat8(60);
  // if (b < 40)
  // {
  //   b = 40;
  // }
  // fill(BODY_FRONT_START, BODY_FRONT_END, CHSV(0, 230, 40));
  if (timeElapsed > 1000)
  {
    timeElapsed = 0;
    fadeSpeed = 15;
    b = 1;
    for (int i = BODY_FRONT_START + 1; i < BODY_FRONT_END - 9; i++)
    {
      leds[i].setHSV(0, 255, 255);
    }
  }
  if (timeElapsed > 200 && b == 1)
  {
    // timeElapsed = 0;
    fadeSpeed = 10;
    b = 0;
    for (int i = BODY_FRONT_START + 1; i < BODY_FRONT_END - 9; i++)
    {
      leds[i].setHSV(0, 255, 255);
    }
  }

  // EVERY_N_MILLISECONDS(1500)
  // {
  //   fiberTail.setPixelColor(0, 255, 0, 0);
  //   fiberTail.setBrightness(255);
  //   fiberTail.show();

  //   eyes.setPixelColor(0, 255, 0, 0);
  //   eyes.setBrightness(50);
  //   eyes.show();

  //   fiberHead.setPixelColor(0, 240, 10, 10);
  //   fiberHead.setBrightness(150);
  //   fiberHead.show();
  // }
}

void wingBeatSetup()
{
  shouldClear = false;
  FastLED.setBrightness(100);
  eyes.setPixelColor(0, 255, 25, 50);
  eyes.setBrightness(40);
  fiberHead.setBrightness(255);
  fiberHead.setPixelColor(0, 255, 25, 50);

  fiberTail.setBrightness(255);
  fiberTail.setPixelColor(0, 255, 25, 50);
}

void wingBeat()
{
  static uint8_t gHue = 0;
  fadeToBlackBy(leds, NUM_LEDS, 3);

  int speed = 60;

  int wingColor = beatsin16(10, 0, 255);
  int bodyColor = beatsin16(3, 0, 255);

  int diff = 25; //BODY_MIDDLE - BODY_START;
  int pos = beatsin16(speed, 0, diff);
  leds[BODY_MIDDLE-pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, BODY_MIDDLE, BODY_END);
  leds[pos] += CHSV(bodyColor, 255, 192);


  pos = beatsin16(speed, HEAD_LEFT_START, HEAD_LEFT_END);
  leds[pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, HEAD_RIGHT_START, HEAD_RIGHT_END);
  leds[pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, LW_FRONT_START, LW_FRONT_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, LW_MIDDLE_START, LW_MIDDLE_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, LW_BACK_START, LW_BACK_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, RW_FRONT_START, RW_FRONT_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  diff = 68; // RW_MIDDLE_START - RW_MIDDLE_END
  pos = beatsin16(speed, 0, diff);
  leds[RW_MIDDLE_END-pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, RW_BACK_START, RW_BACK_END);
  leds[pos] += CHSV(wingColor, 255, 192);



  leds[fiberleft(0)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(1)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(2)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(3)] += CHSV(wingColor, 255, 192);

  leds[fiberright(0)] += CHSV(wingColor, 255, 192);
  leds[fiberright(1)] += CHSV(wingColor, 255, 192);
  leds[fiberright(2)] += CHSV(wingColor, 255, 192);
  leds[fiberright(3)] += CHSV(wingColor, 255, 192);

  // EVERY_N_MILLISECONDS(5) { gHue++; }
  fiberTail.show();
  fiberHead.show();
  eyes.show();
}

void slowBeatSetup()
{
  shouldClear = false;
  FastLED.setBrightness(100);
  eyes.setPixelColor(0, 255, 25, 50);
  eyes.setBrightness(40);
  fiberHead.setBrightness(255);
  fiberHead.setPixelColor(0, 255, 25, 50);

  fiberTail.setBrightness(255);
  fiberTail.setPixelColor(0, 255, 25, 50);
}

void slowBeat()
{
  static uint8_t gHue = 0;
  static bool shouldFade = true;
  if(shouldFade) {
    fadeToBlackBy(leds, NUM_LEDS, 1);

  }
  shouldFade = !shouldFade;
  int speed = 9;

  int wingColor = beatsin16(3, 0, 255);
  int bodyColor = beatsin16(2, 0, 255);

  int diff = 25; //BODY_MIDDLE - BODY_START;
  int pos = beatsin16(speed, 0, diff);
  leds[BODY_MIDDLE-pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, BODY_MIDDLE, BODY_END);
  leds[pos] += CHSV(bodyColor, 255, 192);


  pos = beatsin16(speed, HEAD_LEFT_START, HEAD_LEFT_END);
  leds[pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, HEAD_RIGHT_START, HEAD_RIGHT_END);
  leds[pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, LW_FRONT_START, LW_FRONT_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, LW_MIDDLE_START, LW_MIDDLE_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, LW_BACK_START, LW_BACK_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, RW_FRONT_START, RW_FRONT_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  diff = 68; // RW_MIDDLE_START - RW_MIDDLE_END
  pos = beatsin16(speed, 0, diff);
  leds[RW_MIDDLE_END-pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, RW_BACK_START, RW_BACK_END);
  leds[pos] += CHSV(wingColor, 255, 192);



  leds[fiberleft(0)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(1)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(2)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(3)] += CHSV(wingColor, 255, 192);

  leds[fiberright(0)] += CHSV(wingColor, 255, 192);
  leds[fiberright(1)] += CHSV(wingColor, 255, 192);
  leds[fiberright(2)] += CHSV(wingColor, 255, 192);
  leds[fiberright(3)] += CHSV(wingColor, 255, 192);

  // EVERY_N_MILLISECONDS(5) { gHue++; }
  fiberTail.show();
  fiberHead.show();
  eyes.show();
}

void sameBeatSetup()
{
  shouldClear = false;
  FastLED.setBrightness(100);
  eyes.setPixelColor(0, 255, 25, 50);
  eyes.setBrightness(40);
  fiberHead.setBrightness(255);
  fiberHead.setPixelColor(0, 255, 25, 50);

  fiberTail.setBrightness(255);
  fiberTail.setPixelColor(0, 255, 25, 50);
}

void sameBeat()
{
  static uint8_t gHue = 0;
  fadeToBlackBy(leds, NUM_LEDS, 1);

  int speed = 9;

  int wingColor = beatsin16(3, 0, 255);
  int bodyColor = wingColor;//beatsin16(2, 0, 255);

  int diff = 25; //BODY_MIDDLE - BODY_START;
  int pos = beatsin16(speed, 0, diff);
  leds[BODY_MIDDLE-pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, BODY_MIDDLE, BODY_END);
  leds[pos] += CHSV(bodyColor, 255, 192);


  pos = beatsin16(speed, HEAD_LEFT_START, HEAD_LEFT_END);
  leds[pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, HEAD_RIGHT_START, HEAD_RIGHT_END);
  leds[pos] += CHSV(bodyColor, 255, 192);

  pos = beatsin16(speed, LW_FRONT_START, LW_FRONT_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, LW_MIDDLE_START, LW_MIDDLE_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, LW_BACK_START, LW_BACK_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, RW_FRONT_START, RW_FRONT_END);
  leds[pos] += CHSV(wingColor, 255, 192);

  diff = 68; // RW_MIDDLE_START - RW_MIDDLE_END
  pos = beatsin16(speed, 0, diff);
  leds[RW_MIDDLE_END-pos] += CHSV(wingColor, 255, 192);

  pos = beatsin16(speed, RW_BACK_START, RW_BACK_END);
  leds[pos] += CHSV(wingColor, 255, 192);



  leds[fiberleft(0)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(1)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(2)] += CHSV(wingColor, 255, 192);
  leds[fiberleft(3)] += CHSV(wingColor, 255, 192);

  leds[fiberright(0)] += CHSV(wingColor, 255, 192);
  leds[fiberright(1)] += CHSV(wingColor, 255, 192);
  leds[fiberright(2)] += CHSV(wingColor, 255, 192);
  leds[fiberright(3)] += CHSV(wingColor, 255, 192);

  // EVERY_N_MILLISECONDS(5) { gHue++; }
  fiberTail.show();
  fiberHead.show();
  eyes.show();
}


// the loop routine runs over and over again forever:
void loop()
{

  if (shouldClear)
    clear();

  if (!usePixies)
    basicPowerLedMode();

  modes[currentMode][0]();

  if (shouldShow)
    FastLED.show();

  EVERY_N_MILLISECONDS(100) { checkPotentiometer(); }
  if (PAD_CONNECTED)
  {
    EVERY_N_MILLISECONDS(30) { checkButtons(); }
  }
  EVERY_N_MILLISECONDS(500) { testled(); }
  EVERY_N_MILLISECONDS(100) { checkSerial(); }

  // EVERY_N_MILLISECONDS(250) { reportSensor(); }
  if (currentDelay > 0)
    delay(currentDelay);
}
