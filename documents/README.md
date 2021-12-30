# Raspberry Pi Environental Monitoring System

## Overview
Sample software system used to monitor data from various sensors attached to a Raspberry Pi.

## Files

```text
RPi_Environmental_Monitor_System
  |-- CMakeLists.txt
  |-- DataStore.cpp
  |-- DataStore.h
  |-- EnvironmentMonitor.cpp : main program
  |-- EnvironmentMonitor.h.in
  |-- LICENSE
  |-- Makefile
  |-- MasterTask.cpp
  |-- MasterTask.h
  |-- README.md
  |-- bin : Binary outputs
  |-- build : Directory to run cmake
  |-- data
  |   |-- config.bin : Used by BME680/BSEC library
  |   `-- state.bin : Used by BME680/BSEC library
  |-- documents
  |   |-- LICENSE
  |   |-- LICENSE-BME280
  |   |-- README-BME280.md
  |   |-- README-BME68x-Sensor-API.md
  |   |-- README.md
  |   `-- RPiEnvMonSys_SW_Architecture.drawio
  |-- hardware
  |   |-- CMakeLists.txt
  |   |-- include
  |   |   |-- ADS1115.h
  |   |   |-- Debug.h
  |   |   |-- I2Cdev.h
  |   |   |-- bme280.h
  |   |   |-- bme280_defs.h
  |   |   |-- bme68x.h
  |   |   |-- bme68x_defs.h
  |   |   |-- bsec_datatypes.h
  |   |   |-- bsec_integration.h
  |   |   `-- bsec_interface.h
  |   |-- libalgobsec.a : Externally-provided BME680/BSEC library
  |   `-- source
  |       |-- ADS1115.cpp
  |       |-- Debug.cpp
  |       |-- I2Cdev.cpp
  |       |-- bme280.cpp
  |       |-- bme68x.cpp
  |       `-- bsec_integration.cpp
  |-- sensors
  │   |-- CMakeLists.txt
  │   |-- include
  │   │   |-- BME280Sensor.h
  │   │   |-- BME680Sensor.h
  │   │   `-- ADC_Sensors.h
  │   |-- README.md
  │   `-- source
  │       |-- BME280Sensor.cpp
  │       |-- BME680Sensor.cpp
  │       `-- ADC_Sensors.cpp
  |-- test
  |   |-- Makefile
  |   |-- test_ADS1115_differential
  |   |-- test_ADS1115_differential.cpp
  |   |-- test_ADS1115_single
  |   |-- test_ADS1115_single.cpp
  |   |-- test_bme280
  |   |-- test_bme280.cpp
  |   |-- test_bsec_rpi
  |   `-- test_bsec_rpi.cpp
  `-- ui
      |-- include
      |   `-- UITask.h
      `-- source
          `-- UITask.cpp
```

## External Dependencies

* [CPP Tools library](https://github.com/nuncio-bitis/Tools)  
  Provides the implementation for AppTask, Command Line Argument parsing, Data Items, time stamps, and the System Logger.

## Description

![Data Gathering System SW Architecture](RPiEnvMonSys_SW_Architecture%2Edrawio)  
![Data Gathering System SW Architecture](RPiEnvMonSys_SW_Architecture%2Epng)  

The main() function of the application provides the "background" processing for the application. All data gathering, handling, and logging is done using subordinate application tasks, which are controlled by a master task (which in turn can be controlled by the main function.)

The data to be monitored is owned by an overall data-store object. Each data item in the data-store is a publish-subscribe object that maintains its own value, state (valid, out-of-range, stale), ID, units, upper and lower value limits, and time-to-expiration (staleness).

Each hardware sensor is accessed and controlled by a subtask, which is instantiated by the main application, given parameters it needs to identify itself, the data item it's updating, the hardware sample rate, and the reporting time (used for averaging samples).

Any task can subscribe to a data item to be notified of a change. All data items can also be polled any time to get its value and state. The UI Task polls data items on a regular basis to update the user interface. Since each data item contains its own name, units, value, and state, this makes it easier to display the item in a generic way.

The CLI (TBD/TODO) can be used for maintenance, configuration, and/or control of the system.

An Alarm Handler (TBD/TODO) can be used for reporting/logging system error conditions, as well as invalid, expired, or out-of-range data.

The System Log contains as much or as little system information as desired, depending on the log level the system is configured for (Critical, High, Medium, Low, Info, Debug). By default, the system would normally be set to allow as low as Low or Info levels. For develpment, the Debug level is recommended.  
Multiple loggers can be used for various reasons, such as to maintain a separate log file for each type of data, or for multiple parts of the system.

