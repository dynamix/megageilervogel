#include <FastLED.h>

// PINS
// 1x Potentiometer
// 1x Mic
// 1x Button
// 2x Accel - SDA/SCL - 38/37 or 3/2 or 18/19
// 4x per Wing, 3x Body, 1x rest
// OLED eyes SI->MOSI (0,11), CL->SCLK (32,13) +3 pins

int led = 13;

// the setup routine runs once when you press reset:
void setup()
{
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop()
{
  digitalWrite(led, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(2000);             // wait for a second
  digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW
  delay(2000);             // wait for a second
}
