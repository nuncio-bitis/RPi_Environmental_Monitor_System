cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(Sensors VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

add_library(sensors
    source/ADC_Sensors.cpp
    source/BME280Sensor.cpp
    source/BME680Sensor.cpp
    )

install(TARGETS sensors DESTINATION "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" )

target_include_directories(sensors PUBLIC
    ${PROJECT_SOURCE_DIR}/../../../C-Projects/Tools/include
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/..
    ${PROJECT_SOURCE_DIR}/../hardware/include
    )
