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
 * UITask.h
 *
 * Created on: Feb 21, 2020
 * Author: jparziale
 */

#ifndef UITASK_H_
#define UITASK_H_
// ****************************************************************************

#include <stdint.h>
#include <string>

#include "Logger.h"
#include "AppTask.h"
#include "DataItem.h"

class UITask : public AppTask, public DataItemSubscriber
{
    // ------------------------------------------------------------------------
public:
    UITask(const std::string name, int id, Logger* pLog /* TODO pass in link to data store */);
    virtual ~UITask();

    void DataItemUpdated(int id) override;

    // ------------------------------------------------------------------------
private:
    // Work function of the sensor task.
    void Entry(void) override;

    void UpdateItem(DataItem<uint64_t> *item);
    void UpdateItem(DataItem<double> *item);

    int m_id;
    Logger* m_pLog;

    // Pointers to data store objects
    DataItem<uint64_t> *cpu_mem_free;
    DataItem<double>   *cpu_temp;

    DataItem<double>   *ambientLight;
    DataItem<double>   *pwr5v;
    DataItem<double>   *pwr3p3v;

    DataItem<double>   *bme280Temp;
    DataItem<double>   *bme280RelHum;
    DataItem<double>   *bme280Pressure;

    DataItem<double>   *bme680Temp;
    DataItem<double>   *bme680RelHum;
    DataItem<double>   *bme680Pressure;
    DataItem<double>   *bme680GasResistance;
    DataItem<uint32_t> *bme680IAQaccuracy;
    DataItem<double>   *bme680IAQ;

    // Subscription tokens to data store objects
    long cpu_mem_free_tok;
    long cpu_temp_tok;

    long ambientLight;
    long pwr5v;
    long pwr3p3v;

    long bme280Temp;
    long bme280RelHum;
    long bme280Pressure;

    long bme680Temp;
    long bme680RelHum;
    long bme680Pressure;
    long bme680GasResistance;
    long bme680IAQaccuracy;
    long bme680IAQ;
}

// ****************************************************************************
#endif /* UITASK_H_ */
