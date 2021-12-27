/* 
 * This file is part of the RPi_Environmental_Monitor_System distribution
 *   (https://github.com/nuncio-bitis/RPi_Environmental_Monitor_System
 * Copyright (c) 2021 James P. Parziale.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// ============================================================================
// ADS1115 I2C device class
// Note that the ADS1115 uses 16-bit registers, not 8-bit registers.
// ============================================================================

#include <stdio.h>
#include <cstring>
#include "ADS1115.h"

/** Default constructor, uses default I2C address.
 * @see ADS1115_DEFAULT_ADDRESS
 */
ADS1115::ADS1115() :
    mI2Cport(NULL),
    devAddr(ADS1115_DEFAULT_ADDRESS),
    devMode(false),
    muxMode(ADS1115_MUX_P0_N1),
    pgaMode(ADS1115_PGA_4P096)
{
    memset(buffer, 0, sizeof(buffer));
    mI2Cport = new I2Cdev(devAddr);
}

/** Specific address constructor.
 * @param address I2C address
 * @see ADS1115_DEFAULT_ADDRESS
 * @see ADS1115_ADDRESS_ADDR_GND
 * @see ADS1115_ADDRESS_ADDR_VDD
 * @see ADS1115_ADDRESS_ADDR_SDA
 * @see ADS1115_ADDRESS_ADDR_SDL
 */
ADS1115::ADS1115(uint8_t address) :
    mI2Cport(NULL),
    devAddr(address),
    devMode(false),
    muxMode(ADS1115_MUX_P0_N1),
    pgaMode(ADS1115_PGA_4P096)
{
    memset(buffer, 0, sizeof(buffer));
    mI2Cport = new I2Cdev(devAddr);
}

ADS1115::~ADS1115()
{
    // Delete I2C port
    delete mI2Cport;
}

/** Power on and prepare for general usage.
 * This device is ready to use automatically upon power-up. It defaults to
 * single-shot read mode, P0/N1 mux, 2.048v gain, 128 samples/sec, default
 * comparator with hysterysis, active-low polarity, non-latching comparator,
 * and comparater-disabled operation.
 */
void ADS1115::initialize()
{
    mI2Cport->openI2C();
    setMultiplexer(ADS1115_MUX_P0_N1);
    setGain(ADS1115_PGA_4P096);
    setMode(ADS1115_MODE_SINGLESHOT);
    setRate(ADS1115_RATE_128);
    setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);
    setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
    setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);
    setComparatorQueueMode(ADS1115_COMP_QUE_DISABLE);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ADS1115::testConnection() {
    // Read one word (2 bytes)
    return mI2Cport->readWord(ADS1115_RA_CONVERSION, buffer) == 2;
}

/** Poll the operational status bit until the conversion is finished
 * Retry at most 'max_retries' times
 * conversion is finished, then return true;
 * @see ADS1115_CFG_OS_BIT
 * @return True if data is available, false otherwise
 */
bool ADS1115::pollConversion(uint16_t max_retries) {
  for(uint16_t i = 0; i < max_retries; i++) {
    if (isConversionReady()) return true;
  }
  return false;
}

/** Read differential value based on current MUX configuration.
 * The default MUX setting sets the device to get the differential between the
 * AIN0 and AIN1 pins. There are 8 possible MUX settings, but if you are using
 * all four input pins as single-end voltage sensors, then the default option is
 * not what you want; instead you will need to set the MUX to compare the
 * desired AIN* pin with GND. There are shortcut methods (getConversion*) to do
 * this conveniently, but you can also do it manually with setMultiplexer()
 * followed by this method.
 *
 * In single-shot mode, this register may not have fresh data. You need to write
 * a 1 bit to the MSB of the CONFIG register to trigger a single read/conversion
 * before this will be populated with fresh data. This technique is not as
 * effortless, but it has enormous potential to save power by only running the
 * comparison circuitry when needed.
 *
 * @param triggerAndPoll If true (and only in singleshot mode) the conversion trigger
 *        will be executed and the conversion results will be polled.
 * @return 16-bit signed differential value
 * @see getConversionP0N1();
 * @see getConversionPON3();
 * @see getConversionP1N3();
 * @see getConversionP2N3();
 * @see getConversionP0GND();
 * @see getConversionP1GND();
 * @see getConversionP2GND();
 * @see getConversionP3GND);
 * @see setMultiplexer();
 * @see ADS1115_RA_CONVERSION
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 */
int16_t ADS1115::getConversion(bool triggerAndPoll) {
    if (triggerAndPoll && devMode == ADS1115_MODE_SINGLESHOT) {
      triggerConversion();
      pollConversion(I2CDEV_DEFAULT_READ_TIMEOUT);
    }
    mI2Cport->readWord(ADS1115_RA_CONVERSION, buffer);
    return buffer[0];
}

/** Get AIN0/N1 differential.
 * This changes the MUX setting to AIN0/N1 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0N1() {
    if (muxMode != ADS1115_MUX_P0_N1) setMultiplexer(ADS1115_MUX_P0_N1);
    return getConversion();
}

/** Get AIN0/N3 differential.
 * This changes the MUX setting to AIN0/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0N3() {
    if (muxMode != ADS1115_MUX_P0_N3) setMultiplexer(ADS1115_MUX_P0_N3);
    return getConversion();
}

/** Get AIN1/N3 differential.
 * This changes the MUX setting to AIN1/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP1N3() {
    if (muxMode != ADS1115_MUX_P1_N3) setMultiplexer(ADS1115_MUX_P1_N3);
    return getConversion();
}

/** Get AIN2/N3 differential.
 * This changes the MUX setting to AIN2/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP2N3() {
    if (muxMode != ADS1115_MUX_P2_N3) setMultiplexer(ADS1115_MUX_P2_N3);
    return getConversion();
}

/** Get AIN0/GND differential.
 * This changes the MUX setting to AIN0/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0GND() {
    if (muxMode != ADS1115_MUX_P0_NG) setMultiplexer(ADS1115_MUX_P0_NG);
    return getConversion();
}
/** Get AIN1/GND differential.
 * This changes the MUX setting to AIN1/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP1GND() {
    if (muxMode != ADS1115_MUX_P1_NG) setMultiplexer(ADS1115_MUX_P1_NG);
    return getConversion();
}
/** Get AIN2/GND differential.
 * This changes the MUX setting to AIN2/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP2GND() {
    if (muxMode != ADS1115_MUX_P2_NG) setMultiplexer(ADS1115_MUX_P2_NG);
    return getConversion();
}
/** Get AIN3/GND differential.
 * This changes the MUX setting to AIN3/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP3GND() {
    if (muxMode != ADS1115_MUX_P3_NG) setMultiplexer(ADS1115_MUX_P3_NG);
    return getConversion();
}

/** Get the current voltage reading
 * Read the current differential and return it multiplied
 * by the constant for the current gain.  mV is returned to
 * increase the precision of the voltage
 * @param triggerAndPoll If true (and only in singleshot mode) the conversion trigger
 *        will be executed and the conversion results will be polled.
 */
float ADS1115::getMilliVolts(bool triggerAndPoll) {
  switch (pgaMode) {
    case ADS1115_PGA_6P144:
      return (getConversion(triggerAndPoll) * ADS1115_MV_6P144);
      break;
    case ADS1115_PGA_4P096:
      return (getConversion(triggerAndPoll) * ADS1115_MV_4P096);
      break;
    case ADS1115_PGA_2P048:
      return (getConversion(triggerAndPoll) * ADS1115_MV_2P048);
      break;
    case ADS1115_PGA_1P024:
      return (getConversion(triggerAndPoll) * ADS1115_MV_1P024);
      break;
    case ADS1115_PGA_0P512:
      return (getConversion(triggerAndPoll) * ADS1115_MV_0P512);
      break;
    case ADS1115_PGA_0P256:
    case ADS1115_PGA_0P256B:
    case ADS1115_PGA_0P256C:
      return (getConversion(triggerAndPoll) * ADS1115_MV_0P256);
      break;
  }

  return 0.0;
}

/**
 * Return the current multiplier for the PGA setting.
 *
 * This may be directly retreived by using getMilliVolts(),
 * but this causes an independent read.  This function could
 * be used to average a number of reads from the getConversion()
 * getConversionx() functions and cut downon the number of
 * floating-point calculations needed.
 *
 */

float ADS1115::getMvPerCount() {
  switch (pgaMode) {
    case ADS1115_PGA_6P144:
      return ADS1115_MV_6P144;
      break;
    case ADS1115_PGA_4P096:
      return  ADS1115_MV_4P096;
      break;
    case ADS1115_PGA_2P048:
      return ADS1115_MV_2P048;
      break;
    case ADS1115_PGA_1P024:
      return ADS1115_MV_1P024;
      break;
    case ADS1115_PGA_0P512:
      return ADS1115_MV_0P512;
      break;
    case ADS1115_PGA_0P256:
    case ADS1115_PGA_0P256B:
    case ADS1115_PGA_0P256C:
      return ADS1115_MV_0P256;
      break;
  }

  return 0.0;
}

// CONFIG register

/** Get operational status.
 * @return Current operational status (false for active conversion, true for inactive)
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
bool ADS1115::isConversionReady() {
    mI2Cport->readBitW(ADS1115_RA_CONFIG, ADS1115_CFG_OS_BIT, buffer);
    return buffer[0];
}
/** Trigger a new conversion.
 * Writing to this bit will only have effect while in power-down mode (no conversions active).
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
void ADS1115::triggerConversion() {
    mI2Cport->writeBitW(ADS1115_RA_CONFIG, ADS1115_CFG_OS_BIT, 1);
}
/** Get multiplexer connection.
 * @return Current multiplexer connection setting
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_BIT
 * @see ADS1115_CFG_MUX_LENGTH
 */
uint8_t ADS1115::getMultiplexer() {
    mI2Cport->readBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_MUX_BIT, ADS1115_CFG_MUX_LENGTH, buffer);
    muxMode = (uint8_t)buffer[0];
    return muxMode;
}
/** Set multiplexer connection.  Continous mode may fill the conversion register
 * with data before the MUX setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param mux New multiplexer connection setting
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_BIT
 * @see ADS1115_CFG_MUX_LENGTH
 */
void ADS1115::setMultiplexer(uint8_t mux) {
    if (mI2Cport->writeBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_MUX_BIT, ADS1115_CFG_MUX_LENGTH, mux)) {
        muxMode = mux;
        if (devMode == ADS1115_MODE_CONTINUOUS) {
          // Force a stop/start
          setMode(ADS1115_MODE_SINGLESHOT);
          getConversion();
          setMode(ADS1115_MODE_CONTINUOUS);
        }
    }

}
/** Get programmable gain amplifier level.
 * @return Current programmable gain amplifier level
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_BIT
 * @see ADS1115_CFG_PGA_LENGTH
 */
uint8_t ADS1115::getGain() {
    mI2Cport->readBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_PGA_BIT, ADS1115_CFG_PGA_LENGTH, buffer);
    pgaMode=(uint8_t)buffer[0];
    return pgaMode;
}
/** Set programmable gain amplifier level.
 * Continous mode may fill the conversion register
 * with data before the gain setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param gain New programmable gain amplifier level
 * @see ADS1115_PGA_6P144
 * @see ADS1115_PGA_4P096
 * @see ADS1115_PGA_2P048
 * @see ADS1115_PGA_1P024
 * @see ADS1115_PGA_0P512
 * @see ADS1115_PGA_0P256
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_BIT
 * @see ADS1115_CFG_PGA_LENGTH
 */
void ADS1115::setGain(uint8_t gain) {
    if (mI2Cport->writeBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_PGA_BIT, ADS1115_CFG_PGA_LENGTH, gain)) {
      pgaMode = gain;
         if (devMode == ADS1115_MODE_CONTINUOUS) {
            // Force a stop/start
            setMode(ADS1115_MODE_SINGLESHOT);
            getConversion();
            setMode(ADS1115_MODE_CONTINUOUS);
         }
    }
}
/** Get device mode.
 * @return Current device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
bool ADS1115::getMode() {
    mI2Cport->readBitW(ADS1115_RA_CONFIG, ADS1115_CFG_MODE_BIT, buffer);
    devMode = buffer[0];
    return devMode;
}
/** Set device mode.
 * @param mode New device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
void ADS1115::setMode(bool mode) {
    if (mI2Cport->writeBitW(ADS1115_RA_CONFIG, ADS1115_CFG_MODE_BIT, mode)) {
        devMode = mode;
    }
}
/** Get data rate.
 * @return Current data rate
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_BIT
 * @see ADS1115_CFG_DR_LENGTH
 */
uint8_t ADS1115::getRate() {
    mI2Cport->readBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_DR_BIT, ADS1115_CFG_DR_LENGTH, buffer);
    return (uint8_t)buffer[0];
}
/** Set data rate.
 * @param rate New data rate
 * @see ADS1115_RATE_8
 * @see ADS1115_RATE_16
 * @see ADS1115_RATE_32
 * @see ADS1115_RATE_64
 * @see ADS1115_RATE_128
 * @see ADS1115_RATE_250
 * @see ADS1115_RATE_475
 * @see ADS1115_RATE_860
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_BIT
 * @see ADS1115_CFG_DR_LENGTH
 */
void ADS1115::setRate(uint8_t rate) {
    mI2Cport->writeBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_DR_BIT, ADS1115_CFG_DR_LENGTH, rate);
}
/** Get comparator mode.
 * @return Current comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
bool ADS1115::getComparatorMode() {
    mI2Cport->readBitW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_MODE_BIT, buffer);
    return buffer[0];
}
/** Set comparator mode.
 * @param mode New comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
void ADS1115::setComparatorMode(bool mode) {
    mI2Cport->writeBitW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_MODE_BIT, mode);
}
/** Get comparator polarity setting.
 * @return Current comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
bool ADS1115::getComparatorPolarity() {
    mI2Cport->readBitW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_POL_BIT, buffer);
    return buffer[0];
}
/** Set comparator polarity setting.
 * @param polarity New comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
void ADS1115::setComparatorPolarity(bool polarity) {
    mI2Cport->writeBitW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_POL_BIT, polarity);
}
/** Get comparator latch enabled value.
 * @return Current comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
bool ADS1115::getComparatorLatchEnabled() {
    mI2Cport->readBitW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_LAT_BIT, buffer);
    return buffer[0];
}
/** Set comparator latch enabled value.
 * @param enabled New comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
void ADS1115::setComparatorLatchEnabled(bool enabled) {
    mI2Cport->writeBitW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_LAT_BIT, enabled);
}
/** Get comparator queue mode.
 * @return Current comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_BIT
 * @see ADS1115_CFG_COMP_QUE_LENGTH
 */
uint8_t ADS1115::getComparatorQueueMode() {
    mI2Cport->readBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_QUE_BIT, ADS1115_CFG_COMP_QUE_LENGTH, buffer);
    return (uint8_t)buffer[0];
}
/** Set comparator queue mode.
 * @param mode New comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_BIT
 * @see ADS1115_CFG_COMP_QUE_LENGTH
 */
void ADS1115::setComparatorQueueMode(uint8_t mode) {
    mI2Cport->writeBitsW(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_QUE_BIT, ADS1115_CFG_COMP_QUE_LENGTH, mode);
}

// *_THRESH registers

/** Get low threshold value.
 * @return Current low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
int16_t ADS1115::getLowThreshold() {
    mI2Cport->readWord(ADS1115_RA_LO_THRESH, buffer);
    return buffer[0];
}
/** Set low threshold value.
 * @param threshold New low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
void ADS1115::setLowThreshold(int16_t threshold) {
    mI2Cport->writeWord(ADS1115_RA_LO_THRESH, threshold);
}
/** Get high threshold value.
 * @return Current high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
int16_t ADS1115::getHighThreshold() {
    mI2Cport->readWord(ADS1115_RA_HI_THRESH, buffer);
    return buffer[0];
}
/** Set high threshold value.
 * @param threshold New high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
void ADS1115::setHighThreshold(int16_t threshold) {
    mI2Cport->writeWord(ADS1115_RA_HI_THRESH, threshold);
}

/** Configures ALERT/RDY pin as a conversion ready pin.
 *  It does this by setting the MSB of the high threshold register to '1' and the MSB
 *  of the low threshold register to '0'. COMP_POL and COMP_QUE bits will be set to '0'.
 *  Note: ALERT/RDY pin requires a pull up resistor.
 */
void ADS1115::setConversionReadyPinMode() {
    mI2Cport->writeBitW(ADS1115_RA_HI_THRESH, 15, 1);
    mI2Cport->writeBitW(ADS1115_RA_LO_THRESH, 15, 0);
    setComparatorPolarity(0);
    setComparatorQueueMode(0);
}

// Create a mask between two bits
unsigned createMask(unsigned a, unsigned b) {
   unsigned mask = 0;
   for (unsigned i=a; i<=b; i++)
       mask |= 1 << i;
   return mask;
}

uint16_t shiftDown(uint16_t extractFrom, int places) {
  return (extractFrom >> places);
}


uint16_t getValueFromBits(uint16_t extractFrom, int high, int length) {
   int low= high-length +1;
   uint16_t mask = createMask(low ,high);
   return shiftDown(extractFrom & mask, low);
}

/** Show all the config register settings
 */
void ADS1115::showConfigRegister() {
    mI2Cport->readWord(ADS1115_RA_CONFIG, buffer);
    uint16_t configRegister = buffer[0];
    printf("ConfigRegister: 0x%04X\n", configRegister);

    #ifdef ADS1115_DEBUG
      mDebug.println(">>> ADS1115_DEBUG >>>");

      mDebug.print("Register is:");
      mDebug.println(configRegister,BIN);

      mDebug.print("OS:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_OS_BIT,1), BIN);
      mDebug.print("MUX:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_MUX_BIT,ADS1115_CFG_MUX_LENGTH), BIN);

      mDebug.print("PGA:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_PGA_BIT,ADS1115_CFG_PGA_LENGTH), BIN);

      mDebug.print("MODE:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_MODE_BIT,1), BIN);

      mDebug.print("DR:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_DR_BIT,ADS1115_CFG_DR_LENGTH), BIN);

      mDebug.print("CMP_MODE:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_COMP_MODE_BIT,1), BIN);

      mDebug.print("CMP_POL:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_COMP_POL_BIT,1), BIN);

      mDebug.print("CMP_LAT:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_COMP_LAT_BIT,1), BIN);

      mDebug.print("CMP_QUE:\t");
      mDebug.println(getValueFromBits(configRegister,
        ADS1115_CFG_COMP_QUE_BIT,ADS1115_CFG_COMP_QUE_LENGTH), BIN);

      mDebug.println("<<< ADS1115_DEBUG <<<");
    #endif
};
