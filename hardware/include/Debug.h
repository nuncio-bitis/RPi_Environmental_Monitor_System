#ifndef _DEBUG_H_
#define _DEBUG_H_
//******************************************************************************

#include <stdint.h>

// Output number format specification
enum OutputNumberFormat {
    BIN = 0,
    DEC = 1,
    HEX = 2
};

// -----------------------------------------------------------------------------

class Debug {
public:
    Debug();
    ~Debug();

    static void print(const char *str);
    static void print(int data);
    static void print(uint8_t data, OutputNumberFormat format);

    static void println();
    static void println(const char *str);
    static void println(int data);
    static void println(uint32_t data, OutputNumberFormat format);
};

//******************************************************************************
#endif // _DEBUG_H_
