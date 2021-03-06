// This is a demonstration on how to use an input device to trigger changes on your neo pixels.
// You should wire a momentary push button to connect from ground to a digital IO pin.  When you
// press the button it will change to a new pixel animation.  Note that you need to press the
// button once to start the first animation!

#include <Adafruit_NeoPixel.h>
#include <avr/sleep.h>

//#define ARDUINO_UNO 1
#define GEMMA 1

// Digital IO pin connected to the button.  This will be
// held low with an external with a pull-down resistor
// so the switch should pull the pin high momentarily.
// On a low -> high transition, the button press logic
// will execute.
#ifdef ARDUINO_UNO
#define BUTTON_PIN   2
#define BTN_LED_PIN  8
#elif GEMMA
#define BUTTON_PIN   1
#endif

// Analog input pin used to adjust brightness.
#ifdef ARDUINO_UNO
#define POT_PIN     A0
#endif

// Digital IO pin connected to the NeoPixels.
#ifdef ARDUINO_UNO
#define PIXEL_PIN    6
#elif GEMMA
#define PIXEL_PIN    0
#endif

// Number of milliseconds the button must be held down for to sleep/wake the processor.
#define SLEEP_BUTTON_HOLD_TIME 3000

// Pin used to enable power output. Goes to the base of a transistor sitting between power and the Neopixels.
#ifdef ARDUINO_UNO
#define POWER_EN_PIN 7
#elif GEMMA
#define POWER_EN_PIN 2
#endif

// number of pixels
#define PIXEL_COUNT 16
#define TWO_PIXEL_COUNT PIXEL_COUNT*2

// these are the minimum brightness for any given color
#ifdef ARDUINO_UNO
#define MIN 5
#define RAINBOW_MIN 45
#elif GEMMA
#define MIN 65 // the gemma runs on 3.3v, so the pixels aren't as bright and just end up flickering more
#endif

typedef enum {
  RAINBOW_CYCLE, BETTER_THEATER_CHASE, SNAKE, RAIN, FIRE, NONE, FLASH
} SHOW_TYPE;

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = NULL;

uint32_t sparkles[PIXEL_COUNT];
uint32_t rand_next = 3179389096;
uint32_t changeTimer = millis();
uint16_t idx = 0;
uint16_t currTime, btnHighTime = 0;
#ifdef ARDUINO_UNO
uint8_t brightness = 150;
#endif
SHOW_TYPE showType = RAINBOW_CYCLE;
bool oldState = LOW;

void setup() {
#ifdef ARDUINO_UNO
  Serial.begin(9600);
#endif

  // TURN THIS PIN ON BEFORE ANYTHING ELSE!!!!!
  // (more specifically, turn this pin on before the PIXEL_PIN)
#ifdef POWER_EN_PIN
  pinMode(POWER_EN_PIN, OUTPUT);
  digitalWrite(POWER_EN_PIN, HIGH);
  delay(10); // wait for the capacitor to saturate before doing anything else
#endif

  pinMode(BUTTON_PIN, INPUT);
#ifdef BTN_LED_PIN
  pinMode(BTN_LED_PIN, OUTPUT);
#endif

  //pinMode(PIXEL_PIN, OUTPUT);
  strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  uint8_t prevShowType;
  bool newState;

  // Get current button state.
  newState = digitalRead(BUTTON_PIN);
#ifdef BTN_LED_PIN
  digitalWrite(BTN_LED_PIN, newState);
#endif

  // Check if state changed from low to high (button press).
  currTime = millis();
  if (newState == HIGH)
  {
    if (btnHighTime > 0 && currTime > btnHighTime + SLEEP_BUTTON_HOLD_TIME)
    {
      doSleep();
      newState = LOW;
    }
    else if (oldState == LOW)
    {
      btnHighTime = currTime;
    }
    changeTimer = millis(); // reset auto-change countdown whenever a button press is detected
  }
  else // newState = LOW
  {
    btnHighTime = 0;
    if (oldState == HIGH)
    {
      // Short delay to debounce button.
      delay(20);
      // Check if button is still high after debounce.
      newState = digitalRead(BUTTON_PIN);
      if (newState == LOW)
      {
        // update the display
        showType = (int)showType + 1;
        if (showType >= NONE)
          showType = 0;
        showTypeChanged();
      }
    }
  }

  // Set the last button state to the old state.
  oldState = newState;

  // auto-cycle through show types once/hour
  if (changeTimer > millis()) // deal with clock rollover
    changeTimer = millis();
  else if (millis() - changeTimer > 3600000) // = 60 * 60 * 1000
  {
    prevShowType = showType;
    while (showType == prevShowType)
      showType = smallRand(NONE);
    showTypeChanged();
    changeTimer = millis();
  }

  // set the brightness
#ifdef ARDUINO_UNO
  strip.setBrightness(brightness);
#endif

  // run the show
  runShow();
  idx++;
}

void showTypeChanged()
{
  uint8_t i;

  uniform(0);
  memset(sparkles, 0, PIXEL_COUNT * 4);
}

void runShow()
{
  uint16_t delayMills = 0;
  strip.setBrightness(255);

  switch (showType) {
    case RAINBOW_CYCLE:
      delayMills = rainbowCycle();
      break;
    case BETTER_THEATER_CHASE:
      delayMills = betterTheaterChase();
      break;
    case SNAKE:
      delayMills = snake();
      break;
    case RAIN:
      delayMills = rain();
      break;
    case FIRE:
      delayMills = fire();
      break;
    case NONE:
      delayMills = uniform(0);
      break;
    case FLASH:
      flash(idx % 2);
      delayMills = 250;
      break;
  }

  strip.show();
  delay(delayMills);
}

uint16_t uniform(uint32_t c)
{
  uint8_t i;

  for (i = 0; i < PIXEL_COUNT; i++)
  {
    strip.setPixelColor(i, c);
  }

  return 100;
}

uint16_t rainbowCycle()
{
  uint32_t c;
  uint16_t i, j, k, interval;

  j = clkDivide(idx, 20) % PIXEL_COUNT; // essentially pixel position
  k = clkDivide(idx, 2) % 10;

  interval = 256 / PIXEL_COUNT;
  for (i = 0; i < strip.numPixels(); i++) {
    c = Wheel((i + j) * interval + (k * interval / 10));
    strip.setPixelColor(i, c);
  }
  strip.setBrightness(120);

  return 60;
}

uint16_t betterTheaterChase()
{
  uint8_t pos = clkDivide(idx, 3) % (TWO_PIXEL_COUNT);
  uint8_t i, revPos;

  uniform(0);

  for (i = 0; i < 3; i++)
  {
    // draw current position
    revPos = PIXEL_COUNT - (pos / 2) - 1;
    if (pos == TWO_PIXEL_COUNT - 1)
    {
      strip.setPixelColor(PIXEL_COUNT - 1, 100, 100, 100);
      strip.setPixelColor(0,               100, 100, 100);
    }
    else if (pos % 2 == 0)
    {
      strip.setPixelColor(revPos, 150, 150, 150);
    }
    else //if (pos < TWO_PIXEL_COUNT)
    {
      strip.setPixelColor(revPos,     100, 100, 100);
      strip.setPixelColor(revPos - 1, 100, 100, 100);
    }

    // prepare for next position
    pos += (TWO_PIXEL_COUNT * 4) / (4 * 3);
    pos %= TWO_PIXEL_COUNT;
  }

  return 40;
}

uint16_t snake()
{
  uint16_t c;
  uint8_t pos = clkDivide(idx, 3) % PIXEL_COUNT;
  uint8_t i;
  int8_t j;

  c = 150;
  for (i = 0; i < PIXEL_COUNT; i++)
  {
    j = pos - i;
    if (j < 0) j += PIXEL_COUNT;
    j = PIXEL_COUNT - j - 1;
    strip.setPixelColor(j, c, c, c);
    if (c > 5)
      c = c * 10 / 16;
    else
      c = 0;
  }

  return 40;
}

uint16_t rain()
{
	// sparkle(baseColor, sparkleColorLow, singleChannelRange, numSparkles, subVal)
  sparkle(0x0000001E, 0x00505050, 80, 4, 2);

  return 60;
}

uint16_t fire()
{
  // create the base red color
  uint32_t baseColor = (idx % 25) * 2;
  if (baseColor > 25)
    baseColor = 50 - baseColor;
  baseColor += 25;
  baseColor = baseColor << 16;

	// sparkle(baseColor, sparkleColorLow, singleChannelRange, numSparkles, subVal)
  sparkle(baseColor, 0x00969600, 60, PIXEL_COUNT - 6, 2);

  return 20;
}

/**
   A psuedo-random number generator that takes less space than the built in random().
   https://en.wikipedia.org/wiki/Xorshift
*/
uint32_t smallRand(uint32_t limit)
{
  /* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
  rand_next ^= rand_next << 13;
  rand_next ^= rand_next >> 17;
  rand_next ^= rand_next << 5;
  //    Serial.println(rand_next);
  return rand_next % limit;
}

/** Creates a faded sparkling animation.
 * @param baseColor The color for non-sparkles.
 * @param sparkleColorLow The brightest possible sparkle color.
 * @param singleChannelRange The smallest span in the R, G, or B channel between baseColor and sparkleColorLow.
 * @param numSparkles The number of pixels to have sparkle at a time.
 * @param subVal The value to subtract from current sparkle pixels.
 */
void sparkle(uint32_t baseColor, uint32_t sparkleColorLow, uint32_t singleChannelRange, uint8_t numSparkles, uint8_t subVal)
{
  uint32_t rangeRnd;
  uint8_t i, pixel, cnt = 0;
  uint8_t *pixPtr, *basePtr;

  // how many sparkles do we have?
  for (pixel = 0; pixel < PIXEL_COUNT; pixel++)
  {
    cnt += (sparkles[pixel] > 0) ? 1 : 0;
  }

  // create new sparkles!
  while (cnt < numSparkles)
  {
    pixel = smallRand(PIXEL_COUNT);
    if (sparkles[pixel] == 0)
    {
      rangeRnd = smallRand(singleChannelRange);
      sparkles[pixel] = ( (sparkleColorLow & 0x00FF0000) + (rangeRnd << 16) |
                          (sparkleColorLow & 0x0000FF00) + (rangeRnd <<  8) |
                          (sparkleColorLow & 0x000000FF) +         rangeRnd );
      cnt++;
    }
  }

  // draw the sparkles
  for (pixel = 0; pixel < PIXEL_COUNT; pixel++)
  {
    if (sparkles[pixel] > 0)
    {
      pixPtr = (uint8_t*)(sparkles + pixel);
      basePtr = (uint8_t*)&baseColor;
      for (i = 0; i < 4; i++)
      {
        if (pixPtr[i] > 0)
        {
          if (pixPtr[i] > subVal)
            pixPtr[i] -= subVal;
          else
            pixPtr[i] = 0;
        }
        if (pixPtr[i] < basePtr[i])
          pixPtr[i] = basePtr[i];
      }
      strip.setPixelColor(pixel, *((uint32_t*)pixPtr) );
      if (sparkles[pixel] == baseColor)
      {
        sparkles[pixel] = 0;
      }
    }
    else
    {
      strip.setPixelColor(pixel, baseColor);
    }
  }
}

uint16_t flash(bool on)
{
  uniform(on ? 0x80808080 : 0);
  strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

/**
   Divides the given in by divVal.
   Examples:
   1) in     = 0
      divVal = 10
      ret    = 0
   2) in     = 45
      divVal = 10
      ret    = 4
   3) in     = 45
      divVal = 1
      ret    = 45
*/
uint16_t clkDivide(uint16_t in, uint16_t divVal)
{
  return (in - (in % divVal)) / divVal;
}

/**
   Turns off the ADC and puts the processor in an extremely low power mode.
   Holding the button down for 3 seconds will turn the processor back on, but it won't do anything until the button is released.
   Once the processor is awake it will also re-enable everything it had turned off.
*/
void doSleep()
{
  uint8_t adcsra_prev, i;
  bool flashVal, wakeReady;

  wakeReady = false;
  flashVal = true;

  // let the user know we are going to sleep
#ifdef ARDUINO_UNO
  Serial.println("Going to sleep now!");
  delay(50);
#endif
  for (i = 0; i < 3; i++);
  {
    flash(true);
    delay(250);
    flash(false);
    delay(250);
  }

  // wait for the user to release the button
  while (digitalRead(BUTTON_PIN) == HIGH)
  {
    delay(100);
  }
  delay(50); // debounce time

  // turn off the button LED
#ifdef BTN_LED_PIN
  digitalWrite(BTN_LED_PIN, LOW);
#endif

  // disable ADC
#ifdef ARDUINO_UNO
  Serial.println("...disabling ADC");
  delay(50);
#endif
  adcsra_prev = ADCSRA;
#ifdef ARDUINO_UNO
  ADCSRA = 0;
#elif GEMMA
  ADCSRA &= ~_BV(ADEN);
#endif

  // prepare shut down
#ifdef ARDUINO_UNO
  Serial.println("...preparing to shut down");
  delay(50);
#endif
  enableSleep();

  // shut down
#ifdef ARDUINO_UNO
  Serial.println("...shutting down");
  delay(50);
#endif
  safeSleep();
#ifdef ARDUINO_UNO
  Serial.println("...temp wake up");
  delay(50);
#endif

  /////////////////////////////////////////////////////////////////
  /////////////////////// Now we are asleep. //////////////////////
  ////// Once we pass this line, we check for wake signal. ////////
  /////////////////////////////////////////////////////////////////

  // disable sleep while waiting on button press status
  disableSleep();

  // check if button is held down for long enough
  while (1)
  {
#ifdef ARDUINO_UNO
    Serial.println("Check for wake up!");
    delay(50);
#endif
    currTime = millis();
    btnHighTime = currTime;

    // debounce time
    delay(50);

    // wait for the button to be released
#ifdef BTN_LED_PIN
    digitalWrite(BTN_LED_PIN, HIGH);
#endif
    while (digitalRead(BUTTON_PIN) == HIGH)
    {
      delay(100);
      currTime = millis();
      if (currTime > btnHighTime + SLEEP_BUTTON_HOLD_TIME)
      {
        // Held down for 3 seconds, time to wake up!
        wakeReady = true;

        // Let the user know we are awake and wait for button to be released.
        wakeReady = true;
        flash(flashVal);
        flashVal = !flashVal;
        delay(250);
      }
    }
    delay(50); // debounce time
    if (wakeReady)
    {
      break;
    }

    // not held long enough, go to sleep
#ifdef BTN_LED_PIN
    digitalWrite(BTN_LED_PIN, LOW);
#endif
    enableSleep();
    safeSleep();    // go back to sleep, wait for next button press
    disableSleep(); // wake for next cycle button press check
  }

  ////////////////////////////////////////
  ////////// Now we are awake. ///////////
  ////////////////////////////////////////

  // disable sleep
  sleep_disable();

  // enable ADC
  ADCSRA = adcsra_prev;

  // get everything to a good state
  btnHighTime = 0;
#ifdef ARDUINO_UNO
  Serial.println("Time to wake up!");
  delay(50);
#endif
  changeTimer = millis();
}

/**
   Enables sleep and registers the interrupt button as an external wake interrupt.
*/
void enableSleep()
{
  // enable interrupt pin INT0 (arduino uno digital pin 2)
#ifdef ARDUINO_UNO
  //EICRA = _BV(ISC01);            //configure INT0 to trigger on falling edge
  //EIMSK = _BV(INT0);             //enable INT0
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), sleepInterruptHandler, HIGH);
#elif GEMMA
  PCMSK = _BV(PCINT1);             // Set change mask for pin 1
  GIMSK = _BV(PCIE);               // Enable pin change interrupt
#endif

  cli();                        // Disable interrupts

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  sei();                        // Enable interrupts
}

/**
   Disables sleep and unregisters the interrupt button as an external wake interrupt.
*/
void disableSleep()
{
  // disable interrupt pin INT0 (arduino uno digital pin 2)
#ifdef ARDUINO_UNO
  //EICRA = ~_BV(ISC01);            //configure INT0 to trigger on falling edge
  //EIMSK = ~_BV(INT0);             //enable INT0
  detachInterrupt(0);
#elif GEMMA
  GIMSK = 0;                        // Disable Pin Change Interrupts
  PCMSK = 0;                        // Turn off interrupt pin
#endif

  cli();                        // Disable interrupts
  sleep_disable();              // Clear SE bit

  sei();                        // Enable interrupts
}

/**
   Turns off the neopixels, turns off the power pin, and then sleeps.
   After sleep, turns the power pin back on.
*/
void safeSleep()
{
  uniform(0);                       // turn off the neopixels
  strip.show();                     // tell the neopixels that they're off
  digitalWrite(PIXEL_PIN, LOW);     // no really, turn them off
  delay(10);                        // wait for them to realize they're off, just to be extra safe
#ifdef POWER_EN_PIN
  digitalWrite(POWER_EN_PIN, LOW);  // turn off the power to the neopixels
#endif
  sleep_cpu();                      // go to sleep
#ifdef POWER_EN_PIN
  digitalWrite(POWER_EN_PIN, HIGH); // turn on power to the neopixels
  delay(10); // wait for the capacitor to saturate before doing anything else
#endif
}

//external interrupt 0 wakes the MCU
#ifdef ARDUINO_UNO
void sleepInterruptHandler()//INT0_vect)
#elif GEMMA
ISR(PCINT0_vect)
#endif
{
  disableSleep();
}
