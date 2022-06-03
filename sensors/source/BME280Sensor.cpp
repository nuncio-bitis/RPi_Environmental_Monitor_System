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
 * BME280Sensor.cpp
 *
 * Created on: Dec 28, 2021
 * Author: jparziale
 */
// ****************************************************************************

#include "DataStore.h"
#include "BME280Sensor.h"

#include <unistd.h>
#include <iostream>

I2Cdev * BME280SensorTask::pI2Cport = nullptr;

// ****************************************************************************

BME280SensorTask::BME280SensorTask(const std::string name, DataItemId id, Logger* pLog,
                       const std::string type, double sampleFreq, double reportPeriod) :
    AppTask(name, pLog),
    m_id(id),
    m_pLog(pLog),
    m_type(type),
    m_sampleFreq(sampleFreq),
    m_reportPeriod(reportPeriod),
    m_samplesPerReport(sampleFreq * reportPeriod),
    m_sampleCount(0),
    pTemp_DI(nullptr),
    pRelHum_DI(nullptr),
    pPressure_DI(nullptr)
{
    // Create the I2C port based on its chip address.
    pI2Cport = new I2Cdev(BME280_I2C_ADDR_PRIM, I2C1DeviceName);

    m_pLog->log(eLOG_DEBUG, "%s : CREATED", GetName().c_str());
}

BME280SensorTask::~BME280SensorTask()
{
    delete(pI2Cport);
    m_pLog->log(eLOG_DEBUG, "%s.%s : DONE", GetName().c_str(), __FUNCTION__);
}

// ****************************************************************************

void BME280SensorTask::Entry()
{
    waitForBeginOperation();

    // ------------------------------------------------
    // Task initialization

    m_pLog->log(eLOG_DEBUG, "%s: BEGIN + Initialization", GetName().c_str());

    /* Variable to define the result */
    BME280_INTF_RET_TYPE rslt = BME280_OK;

    // Open the I2C port
    if (pI2Cport->openI2C() < 0)
    {
        perror("openI2C");
        m_pLog->log(eLOG_HIGH, "Failed to open the I2C bus %s", I2C1DeviceName);
        return;
    }

    dev.intf = BME280_I2C_INTF;
    dev.read = static_cast<bme280_read_fptr_t>(&user_i2c_read);
    dev.write = user_i2c_write;

    dev.intf_ptr = NULL; // NOT USED IN THIS CODE

    /* Initialize the bme280 */
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK)
    {
        m_pLog->log(eLOG_HIGH, "Failed to initialize the device (code %+d).", rslt);
        return;
    }

    /*
     * Read temperature, humidity, and pressure data in forced mode.
     *
     * @return Result of API execution status
     *
     * rslt : BME280_OK - Success.
     * rslt : BME280_E_NULL_PTR - Error: Null pointer error
     * rslt : BME280_E_COMM_FAIL - Error: Communication fail error
     * rslt : BME280_E_NVM_COPY_FAILED - Error: NVM copy failed
     *
     */
    rslt = BME280_OK;

    /* Variable to store minimum wait time between consecutive measurement in force mode */
    uint32_t req_delay = 0;

    /* Structure to get the pressure, temperature and humidity values */
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_4X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_16X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    /* Define the selecting sensors */
    uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    /* Set the sensor settings */
    rslt = bme280_set_sensor_settings(settings_sel, &dev);
    if (rslt != BME280_OK)
    {
        m_pLog->log(eLOG_HIGH, "Failed to set sensor settings (code %+d).", rslt);
        return;
    }
    else
    {
        // Calculate the minimum delay required between consecutive measurements
        // based upon the sensor enabled and the oversampling configuration.
        req_delay = bme280_cal_meas_delay(&dev.settings);
        m_pLog->log(eLOG_INFO, "bme280_cal_meas_delay: %d uS", req_delay);
    }

    // Initialize the conversion process; Set the sensor to forced mode
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    if (rslt != BME280_OK)
    {
        m_pLog->log(eLOG_HIGH, "Failed to set sensor mode (code %+d).", rslt);
        return;
    }

    // ------------------------------------------------

    pTemp_DI          = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME280_TEMPERATURE));
    pRelHum_DI        = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME280_RHUM));
    pPressure_DI      = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME280_PRESSURE));

    // Set stale time = 2 x reportPeriod (milliseconds)
    pTemp_DI->setStaleTime(2 * m_reportPeriod * 1000);
    pRelHum_DI->setStaleTime(2 * m_reportPeriod * 1000);
    pPressure_DI->setStaleTime(2 * m_reportPeriod * 1000);

    double temp_current = 0.0;
    double temp_cumulative = 0.0;
    double temp_avg = 0.0;

    double rh_current = 0.0;
    double rh_cumulative = 0.0;
    double rh_avg = 0.0;

    double press_current = 0.0;
    double press_cumulative = 0.0;
    double press_avg = 0.0;

    // ------------------------------------------------

    TaskState prevState = GetState();

    while (!isStopped())
    {
        if (prevState != GetState())
        {
            m_pLog->log(eLOG_DEBUG, "%s: %s", GetName().c_str(), GetStateName().c_str());
        }

        // --------------------------------------------

        // Get sample at sample frequency
        Sleep(1000 / m_sampleFreq);

        // Set the sensor to forced mode
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
        if (rslt != BME280_OK)
        {
            m_pLog->log(eLOG_HIGH, "Failed to set sensor mode (code %+d).", rslt);
        }

        // Wait for the measurement to complete and print data
        usleep(req_delay);

        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if (rslt != BME280_OK)
        {
            m_pLog->log(eLOG_HIGH, "Failed to get sensor data (code %+d).", rslt);
        }

        temp_current = DegreesC_to_DegreesF(comp_data.temperature + tempOffset);
        //m_pLog->log(eLOG_DEBUG, "BME280 Temp: %.2f︒F", temp_current);

        press_current = Pa_to_inHg(comp_data.pressure) + pressOffset;
        //m_pLog->log(eLOG_DEBUG, "BME280 Pressure: %.2f inHg", press_current);

        rh_current = comp_data.humidity + humOffset;
        //m_pLog->log(eLOG_DEBUG, "BME280 Rel Hum: %.2f%", rh_current);

        temp_cumulative  += temp_current;
        press_cumulative += press_current;
        rh_cumulative    += rh_current;

        // --------------------------------------------
        // Take averages and report on report period

        if (++m_sampleCount >= m_samplesPerReport)
        {
            temp_avg = temp_cumulative / m_sampleCount;
            pTemp_DI->setValue(temp_avg);
            temp_cumulative = 0.0;

            rh_avg = rh_cumulative / m_sampleCount;
            pRelHum_DI->setValue(rh_avg);
            rh_cumulative = 0.0;

            press_avg = press_cumulative / m_sampleCount;
            pPressure_DI->setValue(press_avg);
            press_cumulative = 0.0;

            m_sampleCount = 0;
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

// *****************************************************************************

/**
 * This function reads the sensor's registers through I2C bus.
 */
BME280_INTF_RET_TYPE BME280SensorTask::user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_OK;
    if (pI2Cport->readBytes(reg_addr, len, data) <= 0)
    {
        ret = BME280_E_COMM_FAIL;
    }

    return ret;
}

// *****************************************************************************

/**
 * This function is for writing the sensor's registers through I2C bus.
 */
BME280_INTF_RET_TYPE BME280SensorTask::user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t ret = BME280_OK;
    if (pI2Cport->writeBytes(reg_addr, len, (uint8_t *)data) <= 0)
    {
        ret = BME280_E_COMM_FAIL;
    }

    return ret;
}

// ****************************************************************************
