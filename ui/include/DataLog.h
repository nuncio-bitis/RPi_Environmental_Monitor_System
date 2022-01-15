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
 * DataLog.h
 *
 * Created on: Dec 31, 2021
 * Author: jparziale
 */

#ifndef DATALOGTASK_H_
#define DATALOGTASK_H_
// ****************************************************************************

#include <stdint.h>
#include <string>
#include <fstream>

#include "Logger.h"
#include "AppTask.h"
#include "DataItem.h"

/**
 * This class records all data to a CSV-format file.
 */
class DataLogTask : public AppTask, public DataItemSubscriber
{
    // ------------------------------------------------------------------------
public:
    DataLogTask(const std::string name, int id, Logger* pLog /* @TODO pass in link to data store */);
    virtual ~DataLogTask();

    void DataItemUpdated(int id) override;

    // ------------------------------------------------------------------------
private:
    // Work function of the sensor task.
    void Entry(void) override;

    void OpenDataLog(void);
    void WriteHeader(void);
    void ReportData(void);

    int m_id;
    Logger* m_pLog;

    // Name and path of file containing the data log.
    const std::string csvFilePath = "../data/EnvironmentDataLog.csv";
    std::ofstream dataLogFile;

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
};

// ****************************************************************************
#endif /* DATALOGTASK_H_ */
