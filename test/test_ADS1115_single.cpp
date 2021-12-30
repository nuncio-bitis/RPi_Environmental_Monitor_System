// I2C device class (I2Cdev) demonstration Arduino sketch for ADS1115 class
// Example of reading two differential inputs of the ADS1115 and showing the value in mV
// 2016-03-22 by Eadf (https://github.com/eadf)
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2016-03-22 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <iostream>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <stdarg.h>
#include <unistd.h>

#include "ADS1115.h"
#include "Debug.h"

//******************************************************************************

static Debug mDebug;
static ADS1115 adc0(ADS1115_DEFAULT_ADDRESS);

// Wire ADS1115 ALERT/RDY pin to Arduino pin 2
// const int alertReadyPin = 2;

//******************************************************************************

void setup()
{
    //I2Cdev::begin();  // join I2C bus

    adc0.initialize(); // initialize ADS1115 16 bit A/D chip

    mDebug.println("Testing device connections...");
    mDebug.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");

    // We're going to do single shot sampling
    printf("setMode(ADS1115_MODE_SINGLESHOT)...\n"); // XXX
    adc0.setMode(ADS1115_MODE_SINGLESHOT);

    // Slow things down so that we can see that the "poll for conversion" code works
    printf("setRate(ADS1115_RATE_250)...\n"); // XXX
    adc0.setRate(ADS1115_RATE_250);

    // Set the gain (PGA) +/- 6.144V
    // Note that any analog input must be higher than –0.3V and less than VDD +0.3
    printf("setGain(ADS1115_PGA_6P144)...\n"); // XXX
    adc0.setGain(ADS1115_PGA_6P144);
    // ALERT/RDY pin will indicate when conversion is ready

//    pinMode(alertReadyPin,INPUT_PULLUP);
    adc0.setConversionReadyPinMode();

    // To get output from this method, you'll need to turn on the
    // ADS1115_DEBUG definition for the hardware library
    #ifdef ADS1115_DEBUG
    adc0.showConfigRegister();
    mDebug.print("ADS1115_SERIAL_DEBUG: HighThreshold="); mDebug.println(adc0.getHighThreshold(),BIN);
    mDebug.print("ADS1115_SERIAL_DEBUG: LowThreshold="); mDebug.println(adc0.getLowThreshold(),BIN);
    #endif
}

//******************************************************************************

/** Poll the assigned pin for conversion status
 */
void pollAlertReadyPin() {
    usleep(10 * 1000);
//    for (uint32_t i = 0; i<100000; i++)
//      if (!digitalRead(alertReadyPin)) return;
//   mDebug.println("Failed to wait for AlertReadyPin, it's stuck high!");
}

//******************************************************************************

int main(int argc, char *argv[])
{
    setup();
    do
    {
        std::cout << "========================================" << std::endl;

        // The below method sets the mux and gets a reading.
        printf("setMultiplexer(ADS1115_MUX_P0_NG)...\n"); // XXX
        adc0.setMultiplexer(ADS1115_MUX_P0_NG);
        adc0.triggerConversion();
        adc0.showConfigRegister();
        pollAlertReadyPin();
        mDebug.print("A0: "); mDebug.print(adc0.getMilliVolts(false)); mDebug.println("mV");
        usleep(100 * 1000);

        printf("setMultiplexer(ADS1115_MUX_P1_NG)...\n"); // XXX
        adc0.setMultiplexer(ADS1115_MUX_P1_NG);
        adc0.triggerConversion();
        adc0.showConfigRegister();
        pollAlertReadyPin();
        mDebug.print("A1: "); mDebug.print(adc0.getMilliVolts(false)); mDebug.println("mV");
        usleep(100 * 1000);

        printf("setMultiplexer(ADS1115_MUX_P2_NG)...\n"); // XXX
        adc0.setMultiplexer(ADS1115_MUX_P2_NG);
        adc0.triggerConversion();
        adc0.showConfigRegister();
        pollAlertReadyPin();
        mDebug.print("A2: "); mDebug.print(adc0.getMilliVolts(false)); mDebug.println("mV");
        usleep(100 * 1000);

        printf("setMultiplexer(ADS1115_MUX_P3_NG)...\n"); // XXX
        adc0.setMultiplexer(ADS1115_MUX_P3_NG);
        // Do conversion polling via I2C on this last reading:
        adc0.showConfigRegister();
        mDebug.print("A3: "); mDebug.print(adc0.getMilliVolts(true)); mDebug.println("mV");
        usleep(100 * 1000);

        mDebug.println();

    } while(true);
    std::cout << "========================================" << std::endl;

    return EXIT_SUCCESS;
}

//******************************************************************************
