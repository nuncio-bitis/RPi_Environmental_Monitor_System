/* 
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
/*
 * UITask.cpp
 *
 * Created on: Feb 20, 2020
 * Author: jparziale
 */
// ****************************************************************************

#include "UITask.h"
#include "DataStore.h"

#include <iostream>

// ****************************************************************************

UITask::UITask(const std::string name, int id, Logger* pLog) :
    AppTask(name, pLog),
    m_id(id),
    m_pLog(pLog),
    cpu_mem_free_tok(0),
    cpu_temp_tok(0),
    ambientLight_tok(0),
    pwr5v_tok(0),
    pwr3p3v_tok(0),
    bme280Temp_tok(0),
    bme280RelHum_tok(0),
    bme280Pressure_tok(0),
    bme680Temp_tok(0),
    bme680RelHum_tok(0),
    bme680Pressure_tok(0),
    bme680GasResistance_tok(0),
    bme680IAQaccuracy_tok(0),
    bme680IAQ_tok(0)
{
    cpu_mem_free = dynamic_cast<DataItem<uint64_t> *>(DataStore::getInstance()->GetDataItem(CPU_MEM_FREE));
    cpu_temp = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(CPU_TEMP));

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

UITask::~UITask()
{
    m_pLog->log(eLOG_DEBUG, "%s.%s : DONE", GetName().c_str(), __FUNCTION__);
}

// ****************************************************************************

void UITask::UpdateItem(DataItem<uint32_t> *item)
{
    uint32_t value;
    DataItemState state = item->getValue(value);

    if (state == DataItemState::Invalid) {
        return;
    }

    std::string sstate;
    if (state == DataItemState::OutOfRangeLow) {
        sstate = "OORL";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %ld %s *LOW*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::OutOfRangeHigh) {
        sstate = "OORH";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %ld %s *HIGH*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::Stale) {
        sstate = "STALE";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %ld %s *STALE*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::Valid) {
        sstate = "OK";
        m_pLog->log(eLOG_DEBUG, "[UI] (%s) %s %ld %s",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    }

    // @TODO Update value on UI
}

void UITask::UpdateItem(DataItem<uint64_t> *item)
{
    uint64_t value;
    DataItemState state = item->getValue(value);

    if (state == DataItemState::Invalid) {
        return;
    }

    std::string sstate;
    if (state == DataItemState::OutOfRangeLow) {
        sstate = "OORL";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %lld %s *LOW*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::OutOfRangeHigh) {
        sstate = "OORH";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %lld %s *HIGH*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::Stale) {
        sstate = "STALE";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %lld %s *STALE*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::Valid) {
        sstate = "OK";
        m_pLog->log(eLOG_DEBUG, "[UI] (%s) %s %lld %s",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    }

    // @TODO Update value on UI
}

void UITask::UpdateItem(DataItem<double> *item)
{
    double value;
    DataItemState state = item->getValue(value);

    if (state == DataItemState::Invalid) {
        return;
    }

    std::string sstate;
    if (state == DataItemState::OutOfRangeLow) {
        sstate = "OORL";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %lg %s *LOW*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::OutOfRangeHigh) {
        sstate = "OORH";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %lg %s *HIGH*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::Stale) {
        sstate = "STALE";
        m_pLog->log(eLOG_MED, "[UI] (%s) %s %lg %s *STALE*",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    } else if (state == DataItemState::Valid) {
        sstate = "OK";
        m_pLog->log(eLOG_DEBUG, "[UI] (%s) %s %lg %s",
                sstate.c_str(), item->name().c_str(), value, item->getUnits().c_str());
    }

    // @TODO Update value on UI
}

// ****************************************************************************

// Update the UI when a data item has been updated.
void UITask::DataItemUpdated(int id)
{
//    DataItemPublisher* pdi = DataStore::GetDataItem((DataItemId)id);
//    m_pLog->log(eLOG_DEBUG, "[UI] Data Item '%s' (%d) update detected.", pdi->name().c_str(), id);

    // @TODO Update value on UI
}

// ****************************************************************************

void UITask::Entry()
{
    waitForBeginOperation();

    // ------------------------------------------------
    // Task init... subscribe to data items
    // @NOTE: This isn't necessary if the UI will simply poll the data items.
    cpu_mem_free_tok = cpu_mem_free->subscribe(this);
    cpu_temp_tok     = cpu_temp->subscribe(this);
    ambientLight_tok        = ambientLight->subscribe(this);
    pwr5v_tok               = pwr5v->subscribe(this);
    pwr3p3v_tok             = pwr3p3v->subscribe(this);
    bme280Temp_tok          = bme280Temp->subscribe(this);
    bme280RelHum_tok        = bme280RelHum->subscribe(this);
    bme280Pressure_tok      = bme280Pressure->subscribe(this);
    bme680Temp_tok          = bme680Temp->subscribe(this);
    bme680RelHum_tok        = bme680RelHum->subscribe(this);
    bme680Pressure_tok      = bme680Pressure->subscribe(this);
    bme680GasResistance_tok = bme680GasResistance->subscribe(this);
    bme680IAQaccuracy_tok   = bme680IAQaccuracy->subscribe(this);
    bme680IAQ_tok           = bme680IAQ->subscribe(this);

    m_pLog->log(eLOG_DEBUG, "%s: BEGIN + Initialization", GetName().c_str());
    // ------------------------------------------------

    TaskState prevState = GetState();

    while (!isStopped())
    {
        if (prevState != GetState())
        {
            m_pLog->log(eLOG_DEBUG, "%s: %s", GetName().c_str(), GetStateName().c_str());
        }

        // --------------------------------------------
        // Task work - pull values from data store to update log file.
        // @NOTE this is separate from DataItemUpdated() being called when data is updated.
        // That's also when the UI display should be updated.

        // 100ms wait every cycle.
        Sleep(100);

        static uint32_t count1 = 0;
        static uint32_t count2 = 0;
        static uint32_t count3 = 0;

        // [1] Update these every 10 seconds
        if (++count1 >= (10 * 10))
        {
            count1 = 0;

            // CPU items
            UpdateItem(cpu_mem_free);
            UpdateItem(cpu_temp);
        }

        // [2] Update these every 1 minute
        if (++count2 >= (1 * 60 * 10))
        {
            count2 = 0;

            // ADC items
            UpdateItem(ambientLight);
            UpdateItem(pwr5v);
            UpdateItem(pwr3p3v);
        }

        // [3] Update these every 15 minutes
        if (++count3 >= (15 * 60 * 10))
        {
            count3 = 0;

            // BME280 items
            UpdateItem(bme280Temp);
            UpdateItem(bme280RelHum);
            UpdateItem(bme280Pressure);

            // BME680 items
            UpdateItem(bme680Temp);
            UpdateItem(bme680RelHum);
            UpdateItem(bme680Pressure);
            UpdateItem(bme680GasResistance);
            UpdateItem(bme680IAQaccuracy);
            UpdateItem(bme680IAQ);
        }

        // --------------------------------------------

        // PAUSED: Must wait to be told to continue.
        if (isPaused())
        {
            // @TODO Blank & power off screens,LEDs, etc.

            m_pLog->log(eLOG_DEBUG, "--- %s Paused", GetName().c_str());
            waitForContinue();
            m_pLog->log(eLOG_DEBUG, "--- %s Continuing", GetName().c_str());
        }

        prevState = GetState();
    } // end while running

    // ------------------------------------------------
    // Task cleanup... unsubscribe from data items
    cpu_mem_free->unsubscribe(cpu_mem_free_tok);
    cpu_temp->unsubscribe(cpu_temp_tok);

    ambientLight->unsubscribe(ambientLight_tok);
    pwr5v->unsubscribe(pwr5v_tok);
    pwr3p3v->unsubscribe(pwr3p3v_tok);
    bme280Temp->unsubscribe(bme280Temp_tok);
    bme280RelHum->unsubscribe(bme280RelHum_tok);
    bme280Pressure->unsubscribe(bme280Pressure_tok);
    bme680Temp->unsubscribe(bme680Temp_tok);
    bme680RelHum->unsubscribe(bme680RelHum_tok);
    bme680Pressure->unsubscribe(bme680Pressure_tok);
    bme680GasResistance->unsubscribe(bme680GasResistance_tok);
    bme680IAQaccuracy->unsubscribe(bme680IAQaccuracy_tok);
    bme680IAQ->unsubscribe(bme680IAQ_tok);

    m_pLog->log(eLOG_DEBUG, "%s.%s : CLEANUP", GetName().c_str(), __FUNCTION__);
    // ------------------------------------------------
}

// ****************************************************************************
