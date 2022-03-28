/*!
 *  @file PCA9685.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_PWMServoDriver.h"
#include <Wire.h>

//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 */
PCA9685::PCA9685(): _i2caddr(PCA9685_I2C_ADDRESS), _i2c(&Wire) {}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
PCA9685::PCA9685(const uint8_t addr): _i2caddr(addr), _i2c(&Wire) {}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 *  @param  i2c  A reference to a 'TwoWire' object that we'll use to communicate
 *  with
 */
PCA9685::PCA9685(const uint8_t addr,TwoWire &i2c): _i2caddr(addr), _i2c(&i2c) {}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
void PCA9685::begin(uint16_t PWM_freq, double clk_freq) {
  _i2c->begin();
  reset();

  if (clk_freq>0) {
    setOscillatorFrequency(clk_freq);
  } else {
    // set the default internal frequency
    setOscillatorFrequency(FREQUENCY_OSCILLATOR);
  }

  if (PWM_freq) {
    setPWMFreq(PWM_freq);
  } else {
    // set a default frequency
    setPWMFreq(50);
  }

  setAllconf();
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void PCA9685::reset() {
  write8(PCA9685_MODE1, MODE1_RESTART);
  delay(10);
}

/*!
 *  @brief  Puts board into sleep mode
 */
void PCA9685::sleep() {
  uint8_t awake = read8(PCA9685_MODE1);
  uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
  write8(PCA9685_MODE1, sleep);
  delay(5); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void PCA9685::wakeup() {
  uint8_t sleep = read8(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  write8(PCA9685_MODE1, wakeup);
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void PCA9685::enableExtClk() {
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));
  delay(5);
  // clear the SLEEP bit to start
  write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void PCA9685::setPWMFreq(double freq) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Attempting to set freq ");
  Serial.println(freq);
#endif
  // Range output modulation frequency is dependant on oscillator
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  double prescaleval = (_oscillator_freq / (freq * 4096.0)) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;

  setPrescale(round(prescaleval));
}

void PCA9685::setPrescale(uint8_t prescale){
  _prescale = prescale;
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode);                             // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  // This sets the MODE1 register to turn on auto increment.
  write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void PCA9685::setOutputMode(bool totempole) {
  uint8_t oldmode = read8(PCA9685_MODE2);
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
  write8(PCA9685_MODE2, newmode);
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting output mode: ");
  Serial.print(totempole ? "totempole" : "open drain");
  Serial.print(" by setting MODE2 to ");
  Serial.println(newmode);
#endif
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t PCA9685::readPrescale(void) {
  return read8(PCA9685_PRESCALE);
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void PCA9685::setPin(uint8_t num, uint16_t val, bool invert) {
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, (uint16_t)4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    } else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, 4095 - val);
    }
  } else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    } else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, val);
    }
  }
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
uint8_t PCA9685::getPWM(uint8_t num) {
  _i2c->requestFrom((int)_i2caddr, PCA9685_LED0_ON_L + 4 * num, (int)4);
  return _i2c->read();
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM ");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(on);
  Serial.print("->");
  Serial.println(off);
#endif

  _i2c->beginTransmission(_i2caddr);
  _i2c->write(PCA9685_LED0_ON_L + 4 * num);
  _i2c->write(on);
  _i2c->write(on >> 8);
  _i2c->write(off);
  _i2c->write(off >> 8);
  _i2c->endTransmission();
}

/*!
 *  PWM_array = [
 *    [PWM0_on,PWM0_off],
 *    [PWM1_on,PWM1_off],
 *    [PWM2_on,PWM2_off],
 *    ...
 *    [PWM14_on,PWM14_off],
 *    [PWM15_on,PWM15_off]
 *  ]
 */
void PCA9685::setAllPWM(uint16_t PWM_array[16][2]){
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(PCA9685_LED0_ON_L);

  for(int i = 0; i<16;i++){
    if(PWM_array[i][0] > 4096){
      _i2c->write(0xff);
      _i2c->write(0xff);
    }
    else{
      _i2c->write(PWM_array[i][0]);
      _i2c->write(PWM_array[i][0] >> 8);
    }
    
    if(PWM_array[i][1] > 4096){
      _i2c->write(0xff);
      _i2c->write(0xff);
    }
    else{
      _i2c->write(PWM_array[i][1]);
      _i2c->write(PWM_array[i][1] >> 8);
    }
  }

  _i2c->endTransmission();
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void PCA9685::writeMicroseconds(uint8_t num,uint16_t Microseconds) {
  setPWM(num, 0, us_to_count(Microseconds));
}

void PCA9685::writeAllMicroseconds(uint16_t Microseconds_array[16]){
  uint16_t PWM_array[16][2];

  for(int i = 0; i<16;i++){
    PWM_array[i][0] = 0;
    PWM_array[i][1] = us_to_count(Microseconds_array[i]);
  }

  setAllPWM(PWM_array);
}

void PCA9685::write(uint8_t num,float angle){
  writeMicroseconds(num,angle_to_us(num,angle));
}

void PCA9685::writeALL(float angle_array[16]){
  uint16_t duty_us_array[16];

  for(int i = 0; i<16;i++){
    duty_us_array[i] = angle_to_us(i,angle_array[i]);
  }

  writeAllMicroseconds(duty_us_array);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
double PCA9685::getOscillatorFrequency(void) {
  return _oscillator_freq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void PCA9685::setOscillatorFrequency(double freq) {
  _oscillator_freq = freq;
}

uint16_t PCA9685::us_to_count(uint16_t duty_us){
  return constrain(round((((double)duty_us)/((double)_prescale+1))*(_oscillator_freq/1000000.0)),0,4095);
}

uint16_t PCA9685::angle_to_us(uint8_t num,float angle){
  return round(map(angle,0,180,angle_to_us_param[num][0],angle_to_us_param[num][1]));
}

void PCA9685::setconf(uint8_t num, uint16_t min_duty_us,uint16_t max_duty_us){
  if(min_duty_us){
    angle_to_us_param[num][0] = 500;
  }
  else{
    angle_to_us_param[num][0] = min_duty_us;
  }

  if(max_duty_us){
    angle_to_us_param[num][1] = 2500;
  }
  else{
    angle_to_us_param[num][1] = max_duty_us;
  }
}

void PCA9685::setAllconf(uint16_t min_duty_us,uint16_t max_duty_us){
  for(int i = 0;i<16;i++){
    setconf(i,min_duty_us,max_duty_us);
  }
}

/******************* Low level I2C interface */
uint8_t PCA9685::read8(uint8_t addr) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return _i2c->read();
}

void PCA9685::write8(uint8_t addr, uint8_t d) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->write(d);
  _i2c->endTransmission();
}
