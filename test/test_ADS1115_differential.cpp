// I2C device class (I2Cdev) demonstration Arduino sketch for ADS1115 class
// Example of reading two differential inputs of the ADS1115 and showing the value in mV
// 06 May 2013 by Frederick Farzanegan (frederick1@farzanegan.org)
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2013-05-13 - initial release

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

Debug mDebug;

ADS1115 adc0(ADS1115_DEFAULT_ADDRESS);

void setup()
{
    mDebug.println("Initializing I2C devices...");
    adc0.initialize(); // initialize ADS1115 16 bit A/D chip

    mDebug.println("Testing device connections...");
    mDebug.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");

    // To get output from this method, you'll need to turn on the
    //#define ADS1115_SERIAL_DEBUG // in the ADS1115.h file
    adc0.showConfigRegister();

    // We're going to do continuous sampling
    printf("setMode(ADS1115_MODE_CONTINUOUS)...\n"); // @XXX
    adc0.setMode(ADS1115_MODE_CONTINUOUS);
}

//******************************************************************************

int main(int argc, char *argv[])
{
	setup();
	do
	{
		std::cout << "========================================" << std::endl;

        // Sensor is on P0/N1 (pins 4/5)
        mDebug.println("Sensor 1 ************************");
        // Set the gain (PGA) +/- 0.256v
        printf("setGain(ADS1115_PGA_0P256)...\n"); // @XXX
        adc0.setGain(ADS1115_PGA_0P256);

        printf("setMultiplexer(ADS1115_MUX_P1_N0)...\n"); // @XXX

        // Get the number of counts of the accumulator
        mDebug.print("Counts for sensor 1 is: ");

        // The below method sets the mux and gets a reading.
        int sensorOneCounts=adc0.getConversionP0N1();  // counts up to 16-bits
        mDebug.println(sensorOneCounts);

        // To turn the counts into a voltage, we can use
        mDebug.print("Voltage for sensor 1 is: ");
        mDebug.println(sensorOneCounts*adc0.getMvPerCount());
        mDebug.println(adc0.getMilliVolts());  // Convenience method to calculate voltage

        mDebug.println();

        // 2nd sensor is on P2/N3 (pins 6/7)
        mDebug.println("Sensor 2 ************************");
        // Set the gain (PGA) +/- 6.144v
        printf("setGain(ADS1115_PGA_6P144)...\n"); // @XXX
        adc0.setGain(ADS1115_PGA_6P144);

        // Manually set the MUX  // could have used the getConversionP* above
        printf("setMultiplexer(ADS1115_MUX_P2_N3)...\n"); // @XXX
        adc0.setMultiplexer(ADS1115_MUX_P2_N3);

        mDebug.print("Counts for sensor 2 is: ");
        mDebug.println(adc0.getConversion());

        mDebug.print("Voltage sensor 2 is: ");
        mDebug.println(adc0.getMilliVolts());  // Convenience method to calculate voltage

        mDebug.println();

        sleep(1);

	} while(false);
	std::cout << "========================================" << std::endl;

	return EXIT_SUCCESS;
}

//******************************************************************************
