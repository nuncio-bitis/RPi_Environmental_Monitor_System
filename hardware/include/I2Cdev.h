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

#ifndef _I2CDEV_H_
#define _I2CDEV_H_
//******************************************************************************

#include <stdint.h>
#include <stddef.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

// -----------------------------------------------------------------------------

#define I2C1DeviceName "/dev/i2c-1"
#define I2CDEV_DEFAULT_READ_TIMEOUT 10 // max retries

// -----------------------------------------------------------------------------

class I2Cdev {
public:
    I2Cdev(uint8_t devAddr, const char *i2cDevName = NULL);
    virtual ~I2Cdev();

    int openI2C(void);
    int closeI2C(void);

    int8_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    int8_t readBitW(uint8_t regAddr, uint8_t bitNum, uint16_t *data);
    int8_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    int8_t readBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);
    int8_t readByte(uint8_t regAddr, uint8_t *data);
    int8_t readWord(uint8_t regAddr, uint16_t *data);
    int8_t readBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
    int8_t readWords(uint8_t regAddr, uint8_t length, uint16_t *data);

    bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    bool writeBitW(uint8_t regAddr, uint8_t bitNum, uint16_t data);
    bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    bool writeBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
    bool writeByte(uint8_t regAddr, uint8_t data);
    bool writeWord(uint8_t regAddr, uint16_t data);
    bool writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
    bool writeWords(uint8_t regAddr, uint8_t length, uint16_t *data);

private:
    char mI2CDeviceName[32];
    volatile int mI2Cfd;  // I2C file descriptor
    uint8_t mDevAddr;

#ifdef I2CDEV_DEBUG
    Debug mDebug;
#endif // I2CDEV_DEBUG
};

//******************************************************************************
#endif // _I2CDEV_H_
