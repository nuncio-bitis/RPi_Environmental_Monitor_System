#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <stdarg.h>

#include <unistd.h>               //Needed for I2C port
#include <fcntl.h>                //Needed for I2C port
#include <sys/ioctl.h>            //Needed for I2C port
#include <linux/i2c-dev.h>        //Needed for I2C port

#include "I2Cdev.h"

#ifndef BUFFER_LENGTH
// band-aid fix for platforms without Wire-defined BUFFER_LENGTH (removed from some official implementations)
#define BUFFER_LENGTH 32
#endif

/** Default constructor.
 */
I2Cdev::I2Cdev(uint8_t devAddr, const char *i2cDevName) :
        mI2Cfd(-1),
        mDevAddr(devAddr)
{
    if (i2cDevName == NULL) {
        strncpy(mI2CDeviceName, I2C1DeviceName, sizeof(mI2CDeviceName)-1);
    } else {
        strncpy(mI2CDeviceName, i2cDevName, sizeof(mI2CDeviceName)-1);
    }

// Have object owner open the port when they're ready.
//    // Open I2C port
//    mI2Cfd = openI2C();
//    if (-1 == mI2Cfd) {
//        perror("openI2C");
//    }
}

/** Default destructor.
 */
I2Cdev::~I2Cdev()
{
    // Close I2C port
    (void) closeI2C();
}

//******************************************************************************

int I2Cdev::openI2C()
{
    //----- OPEN THE I2C BUS -----
    mI2Cfd = open(mI2CDeviceName, O_RDWR);
    if (mI2Cfd < 0) {
        //ERROR HANDLING: you can check errno to see what went wrong
        printf("Failed to open the i2c bus\n");
        perror("i2cOpen");
        return -1;
    }
    if (ioctl(mI2Cfd, I2C_SLAVE, mDevAddr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        //ERROR HANDLING; you can check errno to see what went wrong
        perror("i2cSetAddress");
        return -1;
    }
    return mI2Cfd;
}

int I2Cdev::closeI2C()
{
    // Close I2C port
    return close(mI2Cfd);
}

//******************************************************************************

/** Read a single bit from an 8-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t b;
    uint8_t count = readByte(regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBitW(uint8_t regAddr, uint8_t bitNum, uint16_t *data)
{
    uint16_t b;
    uint8_t count = readWord(regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2Cdev::readBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data)
{
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(regAddr, &w)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(uint8_t regAddr, uint8_t *data)
{
    return readBytes(regAddr, 1, data);
}

/** Read single word from a 16-bit device register.
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord(uint8_t regAddr, uint16_t *data)
{
    return readWords(regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t I2Cdev::readBytes(uint8_t regAddr, uint8_t length, uint8_t *data)
{
#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: I2C (0x");
    mDebug.print(mDevAddr, HEX);
    mDebug.print(") reading ");
    mDebug.print(length, DEC);
    mDebug.print(" bytes from 0x");
    mDebug.print(regAddr, HEX);
    mDebug.println("...");
#endif

    // Write register to read
    if (write(mI2Cfd, &regAddr, 1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus, reg #%d\n", regAddr);
        return -1;
    }

    //----- READ BYTES -----
    int nread = read(mI2Cfd, data, length);
    if (nread != length) {
        //ERROR HANDLING: i2c transaction failed
        printf("Failed to read from the i2c bus.\n");
        return -1;
    }

#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: Done (");
    mDebug.print(nread, DEC);
    mDebug.println(" bytes read).");
#endif

    return nread;
}

/** Read multiple words from a 16-bit device register.
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords(uint8_t regAddr, uint8_t length, uint16_t *data)
{
#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: I2C (0x");
    mDebug.print(mDevAddr, HEX);
    mDebug.print(") reading ");
    mDebug.print(length, DEC);
    mDebug.print(" words from 0x");
    mDebug.print(regAddr, HEX);
    mDebug.println("...");
#endif

    // Write register to read
    if (write(mI2Cfd, &regAddr, 1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus, reg #%d\n", regAddr);
        return -1;
    }

    // Read bytes, form into words (converting BE to LE)
    int count = 0;
    for (; count < length; ++count) {
        uint8_t byt[2];
        int16_t ret = 0;

        if (read(mI2Cfd, &byt, 2) != 2) {
            //ERROR HANDLING: i2c transaction failed
            printf("Failed to read from the i2c bus.\n");
            return -1;
        }
        ret = (int16_t) ((byt[0] << 8) + byt[1]);

        data[count++] = ret;
    }

#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: Done (");
    mDebug.print(count, DEC);
    mDebug.println(" words read).");
#endif

    return count;
}

/** write a single bit in an 8-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    readByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(uint8_t regAddr, uint8_t bitNum, uint16_t data)
{
    uint16_t w;
    readWord(regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(regAddr, &w) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(regAddr, w);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t regAddr, uint8_t data)
{
    return writeBytes(regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t regAddr, uint16_t data)
{
    return writeWords(regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data)
{
#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: I2C (0x");
    mDebug.print(mDevAddr, HEX);
    mDebug.print(") writing ");
    mDebug.print(length, DEC);
    mDebug.print(" bytes to 0x");
    mDebug.print(regAddr, HEX);
    mDebug.println("...");
#endif

    // Combine register & data into one buffer to write all at once.
    uint8_t buffer[1 + length];
    buffer[0] = regAddr;
    memcpy(&buffer[1], data, length);

    //----- WRITE WORDS -----
    if (write(mI2Cfd, buffer, sizeof(buffer)) != (ssize_t)sizeof(buffer)) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
        return false;
    }

#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: Done (");
    mDebug.print(length, DEC);
    mDebug.println(" bytes written).");
#endif

    return true;
}

/** Write multiple words to a 16-bit device register.
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(uint8_t regAddr, uint8_t length, uint16_t *data)
{
#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: I2C (0x");
    mDebug.print(mDevAddr, HEX);
    mDebug.print(") writing ");
    mDebug.print(length, DEC);
    mDebug.print(" words to 0x");
    mDebug.print(regAddr, HEX);
    mDebug.println("...");
#endif

    // Combine register & data into one buffer to write all at once.
    uint8_t buffer[1 + 2*length];
    buffer[0] = regAddr;

    int idx = 1;
    for (int count = 0; count < length; ++count) {
        buffer[idx] = (data[count] >> 8);
        buffer[idx + 1] = (data[count] & 0xFF);
        idx += 2;
    }

    //----- WRITE WORDS -----
    int rslt = write(mI2Cfd, buffer, sizeof(buffer));
    if ((-1 == rslt) || (rslt != (ssize_t)sizeof(buffer))) {
        /* ERROR HANDLING: i2c transaction failed */
        perror("write");
        printf("Failed to write to the i2c bus.\n");
        return false;
    }

#ifdef I2CDEV_DEBUG
    mDebug.print("I2CDEV_DEBUG: Done (");
    mDebug.print(length, DEC);
    mDebug.println(" words written).");
#endif

    return true;
}
