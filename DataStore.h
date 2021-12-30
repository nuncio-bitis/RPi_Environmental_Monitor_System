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
 * DataStore.h
 *
 *  Created on: Dec 31, 2020
 *      Author: jparziale
 */

#ifndef DATASTORE_H_
#define DATASTORE_H_
// ****************************************************************************

#include "Logger.h"
#include "DataItem.h"

#include <vector>

// ****************************************************************************

// IDs of all data items in the system.
// These double as task IDs also.
enum DataItemId {
    CPU_MEM_FREE,   // /proc/meminfo (MemFree:)
    CPU_TEMP,       // Pi: /opt/vc/bin/vcgencmd measure_temp ; Linux: /sys/class/thermal/thermal_zone0/temp

    ADC_BASE,
    LIGHT_SENSE = ADC_BASE, // ADS1115, A3
    PWR_5V_SENSE,   // ADS1115, A2
    PWR_3P3V_SENSE, // ADS1115, TBD

    BME280_BASE,    // Base ID for all BME280 data items
    BME280_TEMPERATURE = BME280_BASE, // Temp (︒F)
    BME280_RHUM,  // RelHum (%)
    BME280_PRESSURE, // Pressure (inHg)

    BME680_BASE,    // Base ID for all BME680 data items
    BME680_TEMPERATURE = BME680_BASE, // Temp (︒F)
    BME680_RHUM,  // RelHum (%)
    BME680_PRESSURE, // Pressure (inHg)
    BME680_GASRES,
    BME680_IAQA,
    BME680_IAQ,

    UI_TASK_ID,

    NUM_DATA_ITEMS
};

// ****************************************************************************

class DataStore : public std::enable_shared_from_this<DataStore>
{
    // ------------------------------------------------------------------------
public:
    DataStore();
    virtual ~DataStore();

    static DataItemPublisher* GetDataItem(DataItemId id) { return m_DataItemList[id]; };

    static DataStore* getInstance() { return s_pInstance; };

    // ------------------------------------------------------------------------
private:
    static DataStore* s_pInstance;

    // List of all data items
    static std::vector<DataItemPublisher*> m_DataItemList;
};

// ****************************************************************************
#endif /* DATASTORE_H_ */
