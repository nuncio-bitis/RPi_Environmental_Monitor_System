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
 * Pwr_3p3V_Sensor.cpp
 *
 * Created on: Dec 28, 2021
 * Author: jparziale
 */
// ****************************************************************************

#include "DataStore.h"
#include "Pwr_3p3V_Sensor.h"

#include <iostream>

// ****************************************************************************

Pwr3p3VSensorTask::Pwr3p3VSensorTask(const std::string name, DataItemId id, Logger* pLog,
                       const std::string type, double sampleFreq, double reportPeriod) :
    AppTask(name, pLog),
    m_id(id),
    m_pLog(pLog),
    m_type(type),
    m_sampleFreq(sampleFreq),
    m_reportPeriod(reportPeriod),
    m_samplesPerReport(sampleFreq * reportPeriod),
    m_sampleCount(0),
    pDataItem(nullptr)
{
    m_pLog->log(eLOG_DEBUG, "%s : CREATED", GetName().c_str());
}

Pwr3p3VSensorTask::~Pwr3p3VSensorTask()
{
    m_pLog->log(eLOG_DEBUG, "%s.%s : DONE", GetName().c_str(), __FUNCTION__);
}

// ****************************************************************************

void Pwr3p3VSensorTask::Entry()
{
    waitForBeginOperation();

    // ------------------------------------------------
    // @TODO Task initialization

    m_pLog->log(eLOG_DEBUG, "%s: BEGIN + Initialization", GetName().c_str());

    double current = 0.0;
    double cumulative = 0.0;
    double newValue = 0.0;

    pDataItem = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(m_id));

    // Set stale time = 2 x reportPeriod
    pDataItem->setStaleTime(2 * m_reportPeriod * 1000 /* ms */);

    // ------------------------------------------------

    TaskState prevState = GetState();

    while (!isStopped())
    {
        if (prevState != GetState())
        {
            m_pLog->log(eLOG_DEBUG, "%s: %s", GetName().c_str(), GetStateName().c_str());
        }

        // --------------------------------------------
        // @TODO Task work - gather sensor data

        // Get sample at sample frequency
        Sleep(1000 / m_sampleFreq);

        // @TODO cumulative += value read from hardware

        if (++m_sampleCount >= m_samplesPerReport) {
            newValue = cumulative / m_sampleCount;
            pDataItem->setValue(newValue);

            m_sampleCount = 0;
            cumulative = 0.0;
        }
        // --------------------------------------------

        // PAUSED: Must wait to be told to continue.
        if (isPaused())
        {
            // @TODO Pause data-gathering timer

            m_pLog->log(eLOG_DEBUG, "--- %s Paused", GetName().c_str());
            waitForContinue();
            m_pLog->log(eLOG_DEBUG, "--- %s Continuing", GetName().c_str());
        }

        prevState = GetState();
    } // end while running

    // ------------------------------------------------
    // @TODO Task cleanup before exit
    m_pLog->log(eLOG_DEBUG, "%s.%s : CLEANUP", GetName().c_str(), __FUNCTION__);
    // ------------------------------------------------
}

// ****************************************************************************
