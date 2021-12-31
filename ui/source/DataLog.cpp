/**
 * This file is part of the DataGatheringSystem distribution
 *   (https://github.com/nuncio-bitis/DataGatheringSystem
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
/**
 * DataLog.cpp
 *
 * Created on: Feb 20, 2020
 * Author: jparziale
 */
// ****************************************************************************

#include "DataLog.h"
#include "DataStore.h"

#include <iostream>
#include <fstream>

// ****************************************************************************

DataLogTask::DataLogTask(const std::string name, int id, Logger* pLog) :
    AppTask(name, pLog),
    m_id(id),
    m_pLog(pLog)
{
    cpu_mem_free        = dynamic_cast<DataItem<uint64_t> *>(DataStore::getInstance()->GetDataItem(CPU_MEM_FREE));
    cpu_temp            = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(CPU_TEMP));
    ambientLight        = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(LIGHT_SENSE));
    pwr5v               = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(PWR_5V_SENSE));
    pwr3p3v             = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(PWR_3P3V_SENSE));
    bme280Temp          = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME280_TEMPERATURE));
    bme280RelHum        = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME280_RHUM));
    bme280Pressure      = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME280_PRESSURE));
    bme680Temp          = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_TEMPERATURE));
    bme680RelHum        = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_RHUM));
    bme680Pressure      = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_PRESSURE));
    bme680GasResistance = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_GASRES));
    bme680IAQaccuracy   = dynamic_cast<DataItem<uint32_t> *>(DataStore::getInstance()->GetDataItem(BME680_IAQA));
    bme680IAQ           = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_IAQ));

    m_pLog->log(eLOG_DEBUG, "%s(%d) : CREATED", GetName().c_str(), m_id);
}

DataLogTask::~DataLogTask()
{
    m_pLog->log(eLOG_DEBUG, "%s.%s : DONE", GetName().c_str(), __FUNCTION__);
}

// ****************************************************************************

/**
 * Notifies the DataLogTask when a data item has been updated.
 */
void DataLogTask::DataItemUpdated(int id)
{
    // NOP - this task polls the datastore instead of subscribing to data changes.
}

// ****************************************************************************

void DataLogTask::OpenDataLog(void)
{
    // Check if file already exists by opening for input.
    std::ifstream ifile;
    ifile.open(csvFilePath.c_str());
    if (ifile)
    {
        ifile.close();

        m_pLog->log(eLOG_INFO, "[DataLog] %s exists and will be appended.", csvFilePath.c_str());

        dataLogFile.open(csvFilePath.c_str(), std::fstream::out | std::fstream::app);

        // File exists and is now open for append. Nothing more needs to be done.
        // Create a noticeable human-readable gap in the data to indicate the
        // system has been restarted.
        dataLogFile << m_pLog->TimeStamp() << " '" << GetName() << "' restarted." << std::endl;
        dataLogFile.close();
    }
    else
    {
        ifile.close();

        // Create new file.
        m_pLog->log(eLOG_INFO, "[DataLog] %s doesn't exist. Creating new file.", csvFilePath.c_str());

        // Write file header.
        WriteHeader();
    }
}

// ****************************************************************************

/**
 * Format:
 * --,Item1Name (Units),Item2Name (Units), ...
 * Upper Limit,Item1UL,Item2UL, ...
 * Lower Limit,Item1LL,Item2LL, ...
 * --
 * Time Stamp,Item1Name (Units),Item2Name (Units), ...
 */
void DataLogTask::WriteHeader(void)
{
    // Open file
    dataLogFile.open(csvFilePath.c_str(), std::fstream::out | std::fstream::app);

    dataLogFile << std::endl;

    // Write Upper/Lower limit header
    dataLogFile
        << "--,"
        << cpu_mem_free->name() << " (" << cpu_mem_free->getUnits() << "),"
        << cpu_temp->name() << " (" << cpu_temp->getUnits() << "),"
        << ambientLight->name() << " (" << ambientLight->getUnits() << "),"
        << pwr5v->name() << " (" << pwr5v->getUnits() << "),"
        << pwr3p3v->name() << " (" << pwr3p3v->getUnits() << "),"
        << bme280Temp->name() << " (" << bme280Temp->getUnits() << "),"
        << bme280RelHum->name() << " (" << bme280RelHum->getUnits() << "),"
        << bme280Pressure->name() << " (" << bme280Pressure->getUnits() << "),"
        << bme680Temp->name() << " (" << bme680Temp->getUnits() << "),"
        << bme680RelHum->name() << " (" << bme680RelHum->getUnits() << "),"
        << bme680Pressure->name() << " (" << bme680Pressure->getUnits() << "),"
        << bme680GasResistance->name() << " (" << bme680GasResistance->getUnits() << "),"
        << bme680IAQaccuracy->name() << " (" << bme680IAQaccuracy->getUnits() << "),"
        << bme680IAQ->name() << " (" << bme680IAQ->getUnits() << "),"
        << std::endl;
    dataLogFile
        << "Upper Limit,"
        << cpu_mem_free->getUpperLimit() << ","
        << cpu_temp->getUpperLimit() << ","
        << ambientLight->getUpperLimit() << ","
        << pwr5v->getUpperLimit() << ","
        << pwr3p3v->getUpperLimit() << ","
        << bme280Temp->getUpperLimit() << ","
        << bme280RelHum->getUpperLimit() << ","
        << bme280Pressure->getUpperLimit() << ","
        << bme680Temp->getUpperLimit() << ","
        << bme680RelHum->getUpperLimit() << ","
        << bme680Pressure->getUpperLimit() << ","
        << bme680GasResistance->getUpperLimit() << ","
        << bme680IAQaccuracy->getUpperLimit() << ","
        << bme680IAQ->getUpperLimit() << ","
        << std::endl;
    dataLogFile
        << "Lower Limit,"
        << cpu_mem_free->getLowerLimit() << ","
        << cpu_temp->getLowerLimit() << ","
        << ambientLight->getLowerLimit() << ","
        << pwr5v->getLowerLimit() << ","
        << pwr3p3v->getLowerLimit() << ","
        << bme280Temp->getLowerLimit() << ","
        << bme280RelHum->getLowerLimit() << ","
        << bme280Pressure->getLowerLimit() << ","
        << bme680Temp->getLowerLimit() << ","
        << bme680RelHum->getLowerLimit() << ","
        << bme680Pressure->getLowerLimit() << ","
        << bme680GasResistance->getLowerLimit() << ","
        << bme680IAQaccuracy->getLowerLimit() << ","
        << bme680IAQ->getLowerLimit() << ","
        << std::endl;

    dataLogFile << "--" << std::endl;

    // Write data header
    dataLogFile
        << "Time Stamp,"
        << cpu_mem_free->name() << ","
        << cpu_temp->name() << ","
        << ambientLight->name() << ","
        << pwr5v->name() << ","
        << pwr3p3v->name() << ","
        << bme280Temp->name() << ","
        << bme280RelHum->name() << ","
        << bme280Pressure->name() << ","
        << bme680Temp->name() << ","
        << bme680RelHum->name() << ","
        << bme680Pressure->name() << ","
        << bme680GasResistance->name() << ","
        << bme680IAQaccuracy->name() << ","
        << bme680IAQ->name() << ","
        << std::endl;

    // Header done. Close file.
    dataLogFile.close();
}

// ****************************************************************************

void DataLogTask::ReportData(void)
{
    m_pLog->log(eLOG_INFO, "[DataLog] Writing data to %s.", csvFilePath.c_str());

    // Open file
    dataLogFile.open(csvFilePath.c_str(), std::fstream::out | std::fstream::app);

    // Write data
    dataLogFile
        << m_pLog->TimeStamp() << ","
        << cpu_mem_free->getValue() << ","
        << cpu_temp->getValue() << ","
        << ambientLight->getValue() << ","
        << pwr5v->getValue() << ","
        << pwr3p3v->getValue() << ","
        << bme280Temp->getValue() << ","
        << bme280RelHum->getValue() << ","
        << bme280Pressure->getValue() << ","
        << bme680Temp->getValue() << ","
        << bme680RelHum->getValue() << ","
        << bme680Pressure->getValue() << ","
        << bme680GasResistance->getValue() << ","
        << bme680IAQaccuracy->getValue() << ","
        << bme680IAQ->getValue() << ","
        << std::endl;

    // Write done. Close file.
    dataLogFile.close();
}

// ****************************************************************************

void DataLogTask::Entry()
{
    waitForBeginOperation();

    // ------------------------------------------------
    // Task initialization

    // Create data log if it doesn't already exist.
    // Otherwise just open it for append.
    OpenDataLog();

    m_pLog->log(eLOG_DEBUG, "%s: BEGIN + Initialization", GetName().c_str());

    // ------------------------------------------------
    // This task will poll all data items every 15 minutes
    // and add them to the data log.

    TaskState prevState = GetState();
    while (!isStopped())
    {
        if (prevState != GetState())
        {
            m_pLog->log(eLOG_DEBUG, "%s: %s", GetName().c_str(), GetStateName().c_str());
        }

        // --------------------------------------------
        // Task work - poll values from data store to update the data log.

        // 1 second wait every cycle.
        // This allows the task to catch Pause/Continue/Terminate events from MasterTask.
        Sleep(1000);

        static uint32_t report_count = 0;

        // Poll data every 10 minutes
        if (++report_count >= (10 * 60))
        {
            report_count = 0;

            // Add all data item values on one line, separated by commas.
            ReportData();
        }

        // --------------------------------------------

        // PAUSED: Must wait to be told to continue.
        if (isPaused())
        {
            m_pLog->log(eLOG_DEBUG, "--- %s Paused", GetName().c_str());
            waitForContinue();
            m_pLog->log(eLOG_DEBUG, "--- %s Continuing", GetName().c_str());
        }

        prevState = GetState();
    } // end while running

    // ------------------------------------------------
    // Task cleanup...

    m_pLog->log(eLOG_DEBUG, "%s.%s : CLEANUP", GetName().c_str(), __FUNCTION__);

    // ------------------------------------------------
}

// ****************************************************************************
