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
 * LCDTask.h
 *
 * Created on: Apr 7, 2022
 * Author: jparziale
 */

#ifndef LCDTask_H_
#define LCDTask_H_
// ****************************************************************************

#include <stdint.h>
#include <string>

#include "Logger.h"
#include "AppTask.h"
#include "DataItem.h"

class LCDTask : public AppTask, public DataItemSubscriber
{
    // ------------------------------------------------------------------------
public:
    LCDTask(const std::string name, int id, Logger* pLog /* @TODO pass in link to data store */);
    virtual ~LCDTask();

    void DataItemUpdated(int id) override;

    // ------------------------------------------------------------------------
private:
    // Work function of the sensor task.
    void Entry(void) override;

    int m_id;
    Logger* m_pLog;

    // Pointers to data store objects
    DataItem<uint64_t> *cpu_mem_total;
    DataItem<uint64_t> *cpu_mem_free;
    DataItem<double>   *cpu_temp;

    DataItem<double>   *bme680Temp;
    DataItem<double>   *bme680RelHum;
    DataItem<double>   *bme680Pressure;
    DataItem<double>   *ambientLight;

    // Subscription tokens to data store objects
    long cpu_mem_total_tok;
    long cpu_mem_free_tok;
    long cpu_temp_tok;

    long ambientLight_tok;

    long bme680Temp_tok;
    long bme680RelHum_tok;
    long bme680Pressure_tok;

    // Used for determining what to display on the LCD
    enum LCD_Mode {
        MODE_CPU = 0,
        MODE_ENV,
        // -----
        MODE_FIRST = MODE_CPU,
        MODE_LAST  = MODE_ENV
    };

    enum LCD_Mode m_mode = MODE_FIRST;

    // Number of seconds to stay on one display mode.
    const int ModeDelay_sec = 4;

    // Buffers to hold LCD text
    char line1[16];
    char line2[16];
};

// ****************************************************************************
#endif /* LCDTask_H_ */
