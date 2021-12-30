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
 * LightSensor.h
 *
 * Created on: Dec 28, 2021
 * Author: jparziale
 */

#ifndef LIGHTSENSORTASK_H_
#define LIGHTSENSORTASK_H_
// ****************************************************************************

#include "Logger.h"
#include "AppTask.h"
#include "DataItem.h"

#include "ADS1115.h"

class LightSensorTask : public AppTask
{
    // ------------------------------------------------------------------------
public:
    LightSensorTask(const std::string name,
               DataItemId id,
               Logger* pLog,
               const std::string type,
               double sampleFreq,
               double reportPeriod);
    virtual ~LightSensorTask();

    // ------------------------------------------------------------------------
private:
    DataItemId m_id;
    Logger* m_pLog;
    std::string m_type;
    double m_sampleFreq;
    double m_reportPeriod;
    int m_samplesPerReport;
    int m_sampleCount;

    DataItem<double> *pDataItem;

    // Work function of the sensor task.
    void Entry(void) override;

    // The "guts" of the sensor code.
    // The light-sensitive resistor is on A3/GND of the ADS1115
    ADS1115 *pADC0;

    // ADC light sensor setup routine.
    void Setup(void);
};

// ****************************************************************************
#endif /* LIGHTSENSORTASK_H_ */
