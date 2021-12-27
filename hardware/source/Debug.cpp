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
