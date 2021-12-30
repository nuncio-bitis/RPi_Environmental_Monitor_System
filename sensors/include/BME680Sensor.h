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
 * BME680Sensor.h
 *
 * Created on: Dec 28, 2021
 * Author: jparziale
 */

#ifndef BME680SENSORTASK_H_
#define BME680SENSORTASK_H_
// ****************************************************************************

#include "Logger.h"
#include "AppTask.h"
#include "DataItem.h"

#include <I2Cdev.h>

#define BME68X_USE_FPU
#include "bsec_integration.h"

class BME680SensorTask : public AppTask
{
    // ------------------------------------------------------------------------
public:
    BME680SensorTask(const std::string name,
               DataItemId id,
               Logger* pLog,
               const std::string type,
               double sampleFreq,
               double reportPeriod);
    virtual ~BME680SensorTask();

    // ------------------------------------------------------------------------
private:
    DataItemId m_id;
    Logger* m_pLog;
    std::string m_type;
    double m_sampleFreq;
    double m_reportPeriod;
    int m_samplesPerReport;
    int m_sampleCount;

    DataItem<double>   *pTemp_DI;
    DataItem<double>   *pRelHum_DI;
    DataItem<double>   *pPressure_DI;
    DataItem<double>   *pGasResistance_DI;
    DataItem<uint32_t> *pIAQA_DI;
    DataItem<double>   *pIAQ_DI;

    // I2C port for reading and writing.
    // Needs to be static for bus_read and bus_write
    static I2Cdev * pI2Cport;

    static const char *configFile; // = "../data/config.bin";
    static const char *stateFile; // = "../data/state.bin";

#ifdef BME68X_USE_FPU
    // Offsets based on Eve Weather sensor
    static const double tempOffset;
    static const double humOffset;
    static const double pressOffset;
#endif

    static double   temp_current;
    static double   rh_current;
    static double   press_current;
    static double   gas_current;
    static double   iaq_current;
    static uint32_t iaqa_current;

    // Work function of the sensor task.
    void Entry(void) override;

    int64_t get_timestamp_ns(void);
    static void output_ready(
        int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
        float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
        float static_iaq, float co2_equivalent, float breath_voc_equivalent);
    static uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer);
    static uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer);
    static void state_save(const uint8_t *state_buffer, uint32_t length);
    static void SaveConfig(void);
    static void SaveState(void);


    // These need to be static to store pointers in dev.
    static BME68X_INTF_RET_TYPE bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr);
    static BME68X_INTF_RET_TYPE bus_write(uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr);
};

// ****************************************************************************
#endif /* BME680SENSORTASK_H_ */
