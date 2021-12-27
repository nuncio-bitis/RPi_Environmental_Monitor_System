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
