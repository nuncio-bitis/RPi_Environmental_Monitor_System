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

#include <iostream>
#include <string>

#include "Debug.h"


Debug::Debug()
{
}

Debug::~Debug()
{
}

void Debug::print(const char *str)
{
    std::cout << str;
}

void Debug::print(int data)
{
    printf("%d", data);
}

void Debug::print(uint8_t data, OutputNumberFormat format)
{
	switch (format)
	{
	case BIN:
	case HEX:
	    printf("%02X", data);
		break;

	case DEC:
	default:
	    printf("%d", data);
		break;
	}
}

void Debug::println()
{
    std::cout << std::endl;
}

void Debug::println(const char *str)
{
    std::cout << str << std::endl;
}

void Debug::println(int data)
{
    printf("%d\n", data);
}

void Debug::println(uint32_t data, OutputNumberFormat format)
{
	switch (format)
	{
	case BIN:
	case HEX:
	    printf("%08X\n", data);
		break;

	case DEC:
	default:
	    printf("%d\n", data);
		break;
	}
}
