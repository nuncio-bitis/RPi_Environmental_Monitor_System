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
 * LightSensor.cpp
 *
 * Created on: Dec 28, 2021
 * Author: jparziale
 */
// ****************************************************************************

#include "DataStore.h"
#include "LightSensor.h"

#include <iostream>

// ****************************************************************************

LightSensorTask::LightSensorTask(const std::string name, DataItemId id, Logger* pLog,
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
    pADC0 = new ADS1115(ADS1115_DEFAULT_ADDRESS);
    m_pLog->log(eLOG_DEBUG, "%s : CREATED", GetName().c_str());
}

LightSensorTask::~LightSensorTask()
{
    delete(pADC0);
    m_pLog->log(eLOG_DEBUG, "%s.%s : DONE", GetName().c_str(), __FUNCTION__);
}

// ****************************************************************************

void LightSensorTask::Entry()
{
    waitForBeginOperation();

    // ------------------------------------------------
    // Task initialization

    m_pLog->log(eLOG_DEBUG, "%s: BEGIN + Initialization", GetName().c_str());

    // ADC setup
    Setup();

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
        // Task work - gather sensor data

        // Get sample at sample frequency
        Sleep(1000 / m_sampleFreq);

        pADC0->setMultiplexer(ADS1115_MUX_P3_NG);
        pADC0->triggerConversion();
        pADC0->setComparatorMode(false);
        pADC0->setComparatorQueueMode(false);
        Sleep(10);
        current = pADC0->getMilliVolts(true);
        m_pLog->log(eLOG_DEBUG, "A3: %.2f mV", current);

        // cumulative += value read from hardware
        cumulative += current;

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
            // TODO Pause data-gathering timer

            m_pLog->log(eLOG_DEBUG, "--- %s Paused", GetName().c_str());
            waitForContinue();
            Setup();
            m_pLog->log(eLOG_DEBUG, "--- %s Continuing", GetName().c_str());
        }

        prevState = GetState();
    } // end while running

    // ------------------------------------------------
    // Task cleanup before exit
    m_pLog->log(eLOG_DEBUG, "%s.%s : CLEANUP", GetName().c_str(), __FUNCTION__);
    // ------------------------------------------------
}

// ****************************************************************************

void LightSensorTask::Setup()
{
    pADC0->initialize(); // initialize ADS1115 16 bit A/D chip

    m_pLog->log(eLOG_DEBUG, "%s: Testing device connections...", GetName().c_str());
    m_pLog->log(eLOG_DEBUG, pADC0->testConnection() ? "%s: ADS1115 connection successful"
                                                    : "%s: ADS1115 connection failed",
        GetName().c_str());

    // We're going to do single shot sampling
    //m_pLog->log(eLOG_DEBUG, "%s: setMode(ADS1115_MODE_SINGLESHOT)...", GetName().c_str());
    pADC0->setMode(ADS1115_MODE_SINGLESHOT);

    // Slow things down so that we can see that the "poll for conversion" code works
    //m_pLog->log(eLOG_DEBUG, "%s: setRate(ADS1115_RATE_250)...", GetName().c_str());
    pADC0->setRate(ADS1115_RATE_250);

    // Set the gain (PGA) +/- 6.144V
    // Note that any analog input must be higher than –0.3V and less than VDD +0.3
    //m_pLog->log(eLOG_DEBUG, "%s: setGain(ADS1115_PGA_6P144)...", GetName().c_str());
    pADC0->setGain(ADS1115_PGA_6P144);
    // ALERT/RDY pin will indicate when conversion is ready

    pADC0->setComparatorMode(false);

    pADC0->setComparatorQueueMode(false);

    pADC0->setConversionReadyPinMode();

    pADC0->showConfigRegister();
}

// ****************************************************************************
