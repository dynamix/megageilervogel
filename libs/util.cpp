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
  shouldShow = 1;
  usePixies = 0;
  modes[currentMode][1]();
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
