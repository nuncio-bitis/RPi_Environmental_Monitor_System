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
 * BME680Sensor.cpp
 *
 * Created on: Dec 28, 2021
 * Author: jparziale
 */
// ****************************************************************************

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <limits.h>

#include <iostream>

#include "DataStore.h"
#include "BME680Sensor.h"

I2Cdev * BME680SensorTask::pI2Cport = nullptr;

const char *BME680SensorTask::configFile = "../data/config.bin";
const char *BME680SensorTask::stateFile = "../data/state.bin";

// Offsets based on Eve Weather sensor
const double BME680SensorTask::tempOffset  = -0.057; // degC/100
const double BME680SensorTask::humOffset   = +4.50;  // %
const double BME680SensorTask::pressOffset = +0.04;  // inHg

double   BME680SensorTask::temp_current = 0.0;
double   BME680SensorTask::rh_current = 0.0;
double   BME680SensorTask::press_current = 0.0;
double   BME680SensorTask::gas_current = 0.0;
double   BME680SensorTask::iaq_current = 0.0;
uint32_t BME680SensorTask::iaqa_current = 0;

// ****************************************************************************

BME680SensorTask::BME680SensorTask(const std::string name, DataItemId id, Logger* pLog,
                       const std::string type, double sampleFreq, double reportPeriod) :
    AppTask(name, pLog),
    m_id(id),
    m_pLog(pLog),
    m_type(type),
    m_sampleFreq(sampleFreq),
    m_reportPeriod(reportPeriod), // Not used. Reporting data every sample period.
    m_samplesPerReport(sampleFreq * reportPeriod),
    m_sampleCount(0),
    pTemp_DI(nullptr),
    pRelHum_DI(nullptr),
    pPressure_DI(nullptr),
    pGasResistance_DI(nullptr),
    pIAQA_DI(nullptr),
    pIAQ_DI(nullptr)
{
    // Create the I2C port based on its chip address.
    pI2Cport = new I2Cdev(BME68X_I2C_ADDR_HIGH, I2C1DeviceName);

    m_pLog->log(eLOG_DEBUG, "%s : CREATED", GetName().c_str());
}

BME680SensorTask::~BME680SensorTask()
{
    delete(pI2Cport);
    m_pLog->log(eLOG_DEBUG, "%s.%s : DONE", GetName().c_str(), __FUNCTION__);
}

// ****************************************************************************

void BME680SensorTask::Entry()
{
    waitForBeginOperation();

    // ------------------------------------------------
    // Task initialization

    m_pLog->log(eLOG_DEBUG, "%s: BEGIN + Initialization", GetName().c_str());

    return_values_init ret = {BME68X_OK, BSEC_OK};

    // Open Linux I2C device
    // Set address of the BME68X
    if (pI2Cport->openI2C() < 0)
    {
        perror("openI2C");
        m_pLog->log(eLOG_HIGH, "Failed to open the I2C bus %s\n", I2C1DeviceName);
        return;
    }

    // -----------------------------------------------------

    bsec_version_t version;
    (void)bsec_get_version(&version);
    m_pLog->log(eLOG_INFO, "BSEC version: %d.%d.%d.%d",version.major, version.minor, version.major_bugfix, version.minor_bugfix);

    double sampleInterval = 1.0 / (double)BSEC_SAMPLE_RATE_LP;
    m_pLog->log(eLOG_INFO, "BSEC sampling is set to Low-Power, interval = %.1f sec", sampleInterval);

    /* Call to the function which initializes the BSEC library
     * Use low-power mode (3 sec interval) and provide temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, tempOffset, bus_write, bus_read, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        m_pLog->log(eLOG_HIGH, "Error while initializing BME68x");
        return;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        m_pLog->log(eLOG_HIGH, "Error while initializing BSEC library");
        return;
    }

    // ------------------------------------------------
    // Set up for the main (endless) loop that queries sensor settings,
    // applies them, and processes the measured data.

    // Save state every 15 minutes, which means every 900 / 3 = 300 samples
    const uint32_t save_intvl = 300;

    /* Timestamp variables */
    uint64_t time_stamp = 0;
    uint64_t time_stamp_interval_ms = 0;

    /* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];

    /* Number of inputs to BSEC */
    uint8_t num_bsec_inputs = 0;

    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;

    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;

    bsec_library_return_t bsec_status = BSEC_OK;

    // ------------------------------------------------

    pTemp_DI          = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_TEMPERATURE));
    pRelHum_DI        = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_RHUM));
    pPressure_DI      = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_PRESSURE));
    pGasResistance_DI = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_GASRES));
    pIAQA_DI          = dynamic_cast<DataItem<uint32_t> *>(DataStore::getInstance()->GetDataItem(BME680_IAQA));
    pIAQ_DI           = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(BME680_IAQ));

    // NOTE: Stale times were set up in DataStore.cpp when the items were created.
    // All data items have the same sample frequency and report period.

    // ------------------------------------------------

    TaskState prevState = GetState();

    while (!isStopped())
    {
        if (prevState != GetState())
        {
            m_pLog->log(eLOG_DEBUG, "%s: %s", GetName().c_str(), GetStateName().c_str());
        }

        // --------------------------------------------

        /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
        time_stamp = (uint64_t)get_timestamp_ns();

        /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
        bsec_sensor_control(time_stamp, &sensor_settings);

        /* Trigger a measurement if necessary */
        bme680_bsec_trigger_measurement(&sensor_settings);

        // Read data from last measurement
        num_bsec_inputs = 0;
        bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);

        /* Time to invoke BSEC to perform the actual processing */
        bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);

        pTemp_DI->setValue(temp_current);
        pRelHum_DI->setValue(rh_current);
        pPressure_DI->setValue(press_current);
        pGasResistance_DI->setValue(gas_current);
        pIAQ_DI->setValue(iaq_current);

        pIAQA_DI->setValue(iaqa_current);
//        DataItemState st = pIAQA_DI->getState();
//        printf("*** Set IAQ Accuracy: %d, state=%d\n", iaqa_current, st); // XXX

        // --------------------------------------------

        // Update state and config files.
        if (++m_sampleCount >= save_intvl)
        {
            bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
            if (bsec_status == BSEC_OK)
            {
                state_save(bsec_state, bsec_state_len);
            }

            m_sampleCount = 0;
        }

        // --------------------------------------------

        /* Compute how long we need to sleep until we can call bsec_sensor_control() next */
        time_stamp_interval_ms = ((uint64_t)sensor_settings.next_call - (uint64_t)get_timestamp_ns()) / 1000000;

        if (time_stamp_interval_ms > 0)
        {
            Sleep(time_stamp_interval_ms);
        }

        // --------------------------------------------

        // PAUSED: Must wait to be told to continue.
        if (isPaused())
        {
            m_pLog->log(eLOG_DEBUG, "--- %s Paused", GetName().c_str());
            waitForContinue();
            m_pLog->log(eLOG_DEBUG, "--- %s Continuing", GetName().c_str());
        }

        prevState = GetState();
    } // end while running

    // Final save of files.
    SaveState();
    SaveConfig();

    // ------------------------------------------------
    // Task cleanup before exit
    m_pLog->log(eLOG_DEBUG, "%s.%s : CLEANUP", GetName().c_str(), __FUNCTION__);
    // ------------------------------------------------
}

// ****************************************************************************

/*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
BME68X_INTF_RET_TYPE BME680SensorTask::bus_write(
    uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    if (pI2Cport->writeBytes(reg_addr, data_len, (uint8_t *)reg_data_ptr) <= 0)
    {
        rslt = BME68X_E_COM_FAIL;
    }

    return rslt;
}

// *****************************************************************************

/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
BME68X_INTF_RET_TYPE BME680SensorTask::bus_read(
    uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    if (pI2Cport->readBytes(reg_addr, data_len, reg_data_ptr) <= 0)
    {
        rslt = BME68X_E_COM_FAIL;
    }
    usleep(150);                 /* Precautionary response delay */

    return rslt;
}

// *****************************************************************************

/*!
 * @brief           Capture the system time in nanoseconds
 *
 * @return          system_current_time    current system timestamp in nanoseconds
 */
int64_t BME680SensorTask::get_timestamp_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t nanos = (uint64_t)ts.tv_nsec + ((uint64_t)ts.tv_sec * 1000000000);
    return nanos;
}

// *****************************************************************************

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void BME680SensorTask::output_ready(
    int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
    float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
    float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    temp_current  = (double)DegreesC_to_DegreesF(temperature * 100.0);
    rh_current    = (double)humidity * 1000.0 + humOffset;
    press_current = (double)Pa_to_inHg(pressure) + pressOffset;
    gas_current   = (double)gas;
    iaq_current   = (double)iaq;
    iaqa_current  = (uint32_t)iaq_accuracy;
}

// *****************************************************************************

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t BME680SensorTask::config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...

    struct stat statbuf;
    int ret = stat(configFile, &statbuf);
    if (ret < 0)
    {
        printf("config_load(): ERROR: Config file not found.\n");
        return 0;
    }

    int fd = open(configFile, 0);
    if (fd < 0)
    {
        printf("state_load(): ERROR: Could not open config file.\n");
        return 0;
    }
    uint32_t nbytes = read(fd, config_buffer, n_buffer);
    if (nbytes != n_buffer)
    {
        printf("state_load(): ERROR reading config file. Got %d bytes, requested %d\n", nbytes, n_buffer);
        return 0;
    }
    close(fd);

    return nbytes;
}

// *****************************************************************************

void BME680SensorTask::SaveConfig()
{
    // Allocate variables
    uint8_t serialized_settings[BSEC_MAX_PROPERTY_BLOB_SIZE];
    uint32_t n_serialized_settings_max = BSEC_MAX_PROPERTY_BLOB_SIZE;
    uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
    uint32_t n_work_buffer = BSEC_MAX_PROPERTY_BLOB_SIZE;
    uint32_t n_serialized_settings = 0;

    // Configuration of BSEC algorithm is stored in 'serialized_settings'
    bsec_library_return_t ret = bsec_get_configuration(
        (const uint8_t )0,
        (uint8_t *)serialized_settings,
        (const uint32_t)n_serialized_settings_max,
        (uint8_t *)work_buffer,
        (const uint32_t)n_work_buffer,
        (uint32_t *)&n_serialized_settings);
    if (ret != BSEC_OK)
    {
        printf("ERROR: bsec_get_configuration() returned %d\n", ret);
        return;
    }

    // -----------------------------------------------------

    int fd = open(configFile, O_CREAT|O_WRONLY, 0777);
    if (fd < 0)
    {
        perror("open");
        printf("SaveConfig(): ERROR: Could not open config file.\n");
        return;
    }
    uint32_t nbytes = write(fd, serialized_settings, n_serialized_settings_max);
    if (nbytes != n_serialized_settings_max)
    {
        perror("write");
        printf("SaveConfig(): ERROR: Could not write to config file. Tried to write %d bytes, %d bytes were written.\n", n_serialized_settings_max, nbytes);
    }
    close(fd);
}

// *****************************************************************************

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t BME680SensorTask::state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    struct stat statbuf;
    int ret = stat(stateFile, &statbuf);
    if (ret < 0)
    {
        printf("state_load(): ERROR: State file not found.\n");
        return 0;
    }

    int fd = open(stateFile, 0);
    if (fd < 0)
    {
        printf("state_load(): ERROR: Could not open state file.\n");
        return 0;
    }
    uint32_t nbytes = read(fd, state_buffer, n_buffer);
    if (nbytes != n_buffer)
    {
        printf("state_load(): ERROR reading state file. Got %d bytes, requested %d\n", nbytes, n_buffer);
        return 0;
    }
    close(fd);

    return nbytes;
}

// *****************************************************************************

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void BME680SensorTask::state_save(const uint8_t *state_buffer, uint32_t length)
{
    int fd = open(stateFile, O_CREAT|O_WRONLY, 0777);
    if (fd < 0)
    {
        perror("open");
        printf("state_save(): ERROR: Could not open state file.\n");
        return;
    }
    uint32_t nbytes = write(fd, state_buffer, length);
    if (nbytes != length)
    {
        perror("write");
        printf("state_save(): ERROR: Could not write to state file. Tried to write %d bytes, %d bytes were written.\n", length, nbytes);
    }
    close(fd);

    SaveConfig();
}

// ****************************************************************************

void BME680SensorTask::SaveState()
{
    // Allocate variables
    uint8_t serialized_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint32_t n_serialized_state_max = BSEC_MAX_STATE_BLOB_SIZE;
    uint32_t  n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;
    uint8_t work_buffer_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint32_t  n_work_buffer_size = BSEC_MAX_STATE_BLOB_SIZE;
    
    // Algorithm state is stored in 'serialized_state'
    bsec_library_return_t ret = bsec_get_state(
        0, serialized_state, n_serialized_state_max, work_buffer_state, n_work_buffer_size, &n_serialized_state);
    if (ret != BSEC_OK)
    {
        printf("ERROR: bsec_get_state() returned %d\n", ret);
        return;
    }

    // -----------------------------------------------------
    int fd = open(stateFile, O_CREAT|O_WRONLY, 0777);
    if (fd < 0)
    {
        perror("open");
        printf("SaveState(): ERROR: Could not open state file.\n");
        return;
    }
    uint32_t nbytes = write(fd, serialized_state, n_serialized_state_max);
    if (nbytes != n_serialized_state_max)
    {
        perror("write");
        printf("SaveState(): ERROR: Could not write to state file. Tried to write %d bytes, %d bytes were written.\n", n_serialized_state_max, nbytes);
    }
    close(fd);
}

// ****************************************************************************
