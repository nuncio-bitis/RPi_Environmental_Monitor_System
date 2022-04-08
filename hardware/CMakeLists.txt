cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(Hardware VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# @DEBUG : Uncomment to add I2CDev debug statements
#add_compile_definitions(I2CDEV_DEBUG)
# @DEBUG : Uncomment to add ADS1115 debug statements
#add_compile_definitions(ADS1115_DEBUG)

add_library(hardware
    source/ADS1115.cpp
    source/bme68x.cpp
    source/bme280.cpp
    source/bsec_integration.cpp
    source/I2Cdev.cpp
    source/lcd_16x2.c
)
install(TARGETS hardware DESTINATION "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" )

target_include_directories(hardware PUBLIC
    ${PROJECT_SOURCE_DIR}/../../../C-Projects/Tools/include
    ${PROJECT_SOURCE_DIR}/include
    )
