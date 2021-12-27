#ifndef _I2CDEV_H_
#define _I2CDEV_H_
//******************************************************************************

#include <stdint.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// Include debug print statements
// See the CMakeLists.txt file to enable/disable
// -----------------------------------------------------------------------------
//#define I2CDEV_DEBUG
#ifdef I2CDEV_DEBUG
    #include "Debug.h"
#endif // I2CDEV_DEBUG

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
