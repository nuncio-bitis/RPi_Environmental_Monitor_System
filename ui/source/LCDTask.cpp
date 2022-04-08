/* 
 * This file is part of the DataGatheringSystem distribution
 *   (https://github.com/nuncio-bitis/DataGatheringSystem
 * Copyright (c) 2022 James P. Parziale.
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
 * LCDTask.cpp
 *
 * Created on: Apr 7, 2022
 * Author: jparziale
 */
// ****************************************************************************

#include "LCDTask.h"
#include "DataStore.h"
#include "lcd_16x2.h"

#include <iostream>
#include <string.h>

// ****************************************************************************

LCDTask::LCDTask(const std::string name, int id, Logger* pLog) :
    AppTask(name, pLog),
    m_id(id),
    m_pLog(pLog),
    cpu_mem_total_tok(0),
    cpu_mem_free_tok(0),
    cpu_temp_tok(0),
    ambientLight_tok(0),
    bme680Temp_tok(0),
    bme680RelHum_tok(0),
    bme680Pressure_tok(0)
{
    cpu_mem_total = dynamic_cast<DataItem<uint64_t> *>(DataStore::getInstance()->GetDataItem(CPU_MEM_TOTAL));
    cpu_mem_free = dynamic_cast<DataItem<uint64_t> *>(DataStore::getInstance()->GetDataItem(CPU_MEM_FREE));
    cpu_temp = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(CPU_TEMP));

    bme680Temp          = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_TEMPERATURE));
    bme680RelHum        = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_RHUM));
    bme680Pressure      = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_PRESSURE));
    ambientLight        = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(LIGHT_SENSE));

    // Initialize the LCD display
    lcdInit();
    // Return cursor to home position
    lcdCommand(CMD_RET_HOME);

    m_pLog->log(eLOG_DEBUG, "%s(%d) : CREATED", GetName().c_str(), m_id);
}

LCDTask::~LCDTask()
{
    m_pLog->log(eLOG_DEBUG, "%s.%s : DONE", GetName().c_str(), __FUNCTION__);
}

// ****************************************************************************

// Notification for when a data item is updated.
void LCDTask::DataItemUpdated(int id)
{
//    DataItemPublisher* pdi = DataStore::GetDataItem((DataItemId)id);
//    m_pLog->log(eLOG_DEBUG, "[UI] Data Item '%s' (%d) update detected.", pdi->name().c_str(), id);
}

// ****************************************************************************

void LCDTask::Entry()
{
    uint32_t mode_count = ModeDelay_sec;

    uint64_t total_mem = 8192;
    uint64_t free_mem = 4095;
    double freeMemPct = 50.0;

    double temp = 30.1;
    double light = 2800.1;
    double btemp = 66.4;
    double rhum = 61.2;
    double press = 29.94;

    // Initial display mode
    m_mode = MODE_FIRST;

    waitForBeginOperation();

    // ------------------------------------------------
    // Task init... subscribe to data items
    cpu_mem_total_tok = cpu_mem_total->subscribe(this);
    cpu_mem_free_tok = cpu_mem_free->subscribe(this);
    cpu_temp_tok     = cpu_temp->subscribe(this);
    bme680Temp_tok          = bme680Temp->subscribe(this);
    bme680RelHum_tok        = bme680RelHum->subscribe(this);
    bme680Pressure_tok      = bme680Pressure->subscribe(this);
    ambientLight_tok        = ambientLight->subscribe(this);

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
        // Task work - pull values from data store to update the LCD display.

        // 1 second period
        Sleep(1000);

        // --------------------------------------------
        // Update data items

        DataItemState state;

        // CPU items
        state = cpu_mem_total->getValue(total_mem);
        state = cpu_mem_free->getValue(free_mem);
        state = cpu_temp->getValue(temp);

        freeMemPct = 100.0 * (double)free_mem / (double)total_mem;

        // ADC item
        state = ambientLight->getValue(light);
        // BME680 items
        state = bme680Temp->getValue(btemp);
        state = bme680RelHum->getValue(rhum);
        state = bme680Pressure->getValue(press);

        // --------------------------------------------
        // Write to LCD display, depending on mode

        switch (m_mode) {
        case MODE_CPU:
        {
            // Print CPU temperature and Free Memory as percentages on one line
            // and time on the second line.
            // |++++++++++++++++|
            // |C:xx.x°C M:xx.x%| CPU Temperature, Memory Available %
            // |hh:mm:ss PM EDT | Time
            // |++++++++++++++++|

            // Line 1: C:xx.x% M:xx.x%
            sprintf(line1, "C:%4.1f%cC M:%4.1f%%", temp, 0xdf, freeMemPct);

            // Line 2: hh:mm:ss AM/PM EDT/EST
            time_t now = time(NULL);
            struct tm * local_time = localtime(&now);
            strftime(line2, sizeof(line2), "%r %Z ", local_time);

            break;
        }

        case MODE_ENV:
        {
            // Print local environment temperature and humidity on one line,
            // pressure and ambient light on the second line.
            // |++++++++++++++++|
            // |T:xx.x°F rH:xx% | Temperature, Humidity
            // |P:xx.xx  L:xxxx | Pressure, Light
            // |++++++++++++++++|

            sprintf(line1, "T:%4.1f%cF rH:%2.0f%% ", btemp, 0xdf, rhum);
            sprintf(line2, "P:%5.2f  L:%4.0f ", press, light);

            break;
        }

        default:
            m_pLog->log(eLOG_HIGH, "Invalid LCD display mode; %d", m_mode);
            break;
        } // end switch

        // Write text lines to the display.
        (void)lcdText(line1, LCD_LINE1);
        (void)lcdText(line2, LCD_LINE2);

        // Update mode
        if (--mode_count == 0)
        {
            mode_count = ModeDelay_sec;
            if (m_mode == MODE_FIRST)
            {
                m_mode = MODE_LAST;
            }
            else if (m_mode == MODE_LAST)
            {
                m_mode = MODE_FIRST;
            }
            memset(line1, ' ', sizeof(line2));
            memset(line2, ' ', sizeof(line2));
        }

        // --------------------------------------------

        // PAUSED: Must wait to be told to continue.
        if (isPaused())
        {
            // Clear LCD display
            memset(line1, ' ', sizeof(line2));
            memset(line2, ' ', sizeof(line2));
            lcdCommand(CMD_RET_HOME);
            lcdCommand(CMD_CLR_DISP);

            // @TODO Power off LCD display? (needs to be on a GPIO)

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

    bme680Temp->unsubscribe(bme680Temp_tok);
    bme680RelHum->unsubscribe(bme680RelHum_tok);
    bme680Pressure->unsubscribe(bme680Pressure_tok);
    ambientLight->unsubscribe(ambientLight_tok);

    // Clear LCD display
    memset(line1, ' ', sizeof(line2));
    memset(line2, ' ', sizeof(line2));
    lcdCommand(CMD_RET_HOME);
    lcdCommand(CMD_CLR_DISP);

    m_pLog->log(eLOG_DEBUG, "%s.%s : CLEANUP", GetName().c_str(), __FUNCTION__);
    // ------------------------------------------------
}

// ****************************************************************************

