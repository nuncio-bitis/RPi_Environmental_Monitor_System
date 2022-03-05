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
 * BME280Sensor.h
 *
 * Created on: Dec 28, 2021
 * Author: jparziale
 */

#ifndef BME280SENSORTASK_H_
#define BME280SENSORTASK_H_
// ****************************************************************************

#include "Logger.h"
#include "AppTask.h"
#include "DataItem.h"

#include <I2Cdev.h>

#define BME280_FLOAT_ENABLE
#include "bme280.h"

// ****************************************************************************

class BME280SensorTask : public AppTask
{
    // ------------------------------------------------------------------------
public:
    BME280SensorTask(const std::string name,
               DataItemId id,
               Logger* pLog,
               const std::string type,
               double sampleFreq,
               double reportPeriod);
    virtual ~BME280SensorTask();

    // ------------------------------------------------------------------------
private:
    DataItemId m_id;
    Logger* m_pLog;
    std::string m_type;
    double m_sampleFreq;
    double m_reportPeriod;
    int m_samplesPerReport;
    int m_sampleCount;

    DataItem<double> *pTemp_DI;
    DataItem<double> *pRelHum_DI;
    DataItem<double> *pPressure_DI;

    // I2C port for reading and writing.
    // Needs to be static for user_i2c_read and user_i2c_write
    static I2Cdev * pI2Cport;

    struct bme280_dev dev;

#ifdef BME280_FLOAT_ENABLE
    // Offsets based on Eve Weather sensor
    const double tempOffset  = +0.52; // degC
    const double humOffset   = +5.50; // %
    const double pressOffset = +1.02; // hPa
#endif

    // Work function of the sensor task.
    void Entry(void) override;

    // These need to be static to store pointers in dev.
    static BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
    static BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
};

// ****************************************************************************
#endif /* BME280SENSORTASK_H_ */
