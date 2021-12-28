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
/*
 * DataStore.cpp
 *
 *  Created on: Dec 31, 2020
 *      Author: jparziale
 */
// ****************************************************************************

#include "DataStore.h"
#include <iostream>
#include <cstdint>

// ****************************************************************************

DataStore* DataStore::s_pInstance = nullptr;

// List of data items. Initialize its size with the number of sensors.
std::vector<DataItemPublisher*> DataStore::m_DataItemList(NUM_DATA_ITEMS);

// ****************************************************************************

DataStore::DataStore()
{
    s_pInstance = this;

    // Create data items. They will be referenced by index, so create them by index.
    // Format:
    // m_DataItemList[DID] = new DataItem<TYPE>(NAME, UNITS, DID, INITIAL_VAL, LOW_LIMIT, HIGH_LIMIT, STALE_TIME);

    // CPU sensor items
    m_DataItemList[CPU_MEM_FREE] =
        new DataItem<uint64_t>("CPU_Mem_Free", "bytes", CPU_MEM_FREE, 0, 0, ((uint64_t)8*1024*1024*1024), 5000);
    m_DataItemList[CPU_TEMP]     = new DataItem<double>("CPU_Temp", "︒C", CPU_TEMP, 0, -40.0, 80.0, 5000);

    // ADS1115 ADC sensors
    m_DataItemList[LIGHT_SENSE   ] = new DataItem<double>("Ambient_Light", "V", LIGHT_SENSE   , 0.0, 0.0, 6.0, 1000);
    m_DataItemList[PWR_5V_SENSE  ] = new DataItem<double>("Pwr_5V"       , "V", PWR_5V_SENSE  , 0.0, 0.0, 6.0, 10000);
    m_DataItemList[PWR_3P3V_SENSE] = new DataItem<double>("Pwr_3p3V"     , "V", PWR_3P3V_SENSE, 0.0, 0.0, 6.0, 10000);

    // BME280 sensor items
    m_DataItemList[BME280_TEMP   ] = new DataItem<double>("BME280_Ambient_Temp", "︒C"  , BME280_TEMP , 0.0, 0.0, 100.0, 5000);
    m_DataItemList[BME280_RHUM   ] = new DataItem<double>("BME280_Rel_Humidity", "%"   , BME280_RHUM , 0.0, 0.0, 100.0, 5000);
    m_DataItemList[BME280_PRESS  ] = new DataItem<double>("BME280_Pressure"    , "inHg", BME280_PRESS, 0.0, 13.0, 20.0, 5000);

    // BME680 sensor items
    m_DataItemList[BME680_TEMP   ] = new DataItem<double>("BME680_Ambient_Temp"  , "︒C"  , BME680_TEMP  , 0.0, 0.0, 100.0, 5000);
    m_DataItemList[BME680_RHUM   ] = new DataItem<double>("BME680_Rel_Humidity"  , "%"   , BME680_RHUM  , 0.0, 0.0, 100.0, 5000);
    m_DataItemList[BME680_PRESS  ] = new DataItem<double>("BME680_Pressure"      , "inHg", BME680_PRESS , 0.0, 13.0, 20.0, 5000);
    m_DataItemList[BME680_GASRES ] = new DataItem<double>("BME680_Gas_Resistance", "Ohms", BME680_GASRES, 0.0, 0.0, 100.0e03, 5000);
    m_DataItemList[BME680_IAQA   ] = new DataItem<uint32_t>("BME680_IAQ_Accuracy", ""    , BME680_IAQA  , 0.0, 0, 3, 5000);
    m_DataItemList[BME680_IAQ    ] = new DataItem<double>("BME680_IAQ"           , ""    , BME680_IAQ   , 0.0, 0.0, 1000.0, 5000);
}

DataStore::~DataStore()
{
    // Delete all created data items
    for (auto p : m_DataItemList) {
        delete p;
    }
}

//******************************************************************************
