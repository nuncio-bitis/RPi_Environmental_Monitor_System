/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_iot_example.ino
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>

#include <signal.h>
#include <execinfo.h>

#include <I2Cdev.h>

#include "bsec_integration.h"

/**********************************************************************************************************************/

static const char *dev_name = "/dev/i2c-1";

static const char configFile[] = "../data/config.bin";
static const char stateFile[] = "../data/state.bin";

#ifdef BME68X_USE_FPU
// Offsets based on Eve Weather sensor
static const float tempOffset  = -0.057; // degC/100
static const float humOffset   = +4.50;  // %
static const float pressOffset = +0.04;  // inHg
#endif

static char *outputFile = NULL;

// I2C port for reading and writing.
static I2Cdev * mI2Cport = NULL;

static uint8_t terminate = 0;

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
BME68X_INTF_RET_TYPE bus_write(uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    if (mI2Cport->writeBytes(reg_addr, data_len, (uint8_t *)reg_data_ptr) <= 0)
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
BME68X_INTF_RET_TYPE bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    if (mI2Cport->readBytes(reg_addr, data_len, reg_data_ptr) <= 0)
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
int64_t get_timestamp_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t nanos = (uint64_t)ts.tv_nsec + ((uint64_t)ts.tv_sec * 1000000000);
    return nanos;
}

//******************************************************************************

void timestamp(void)
{
    time_t time_value = time(NULL);
    struct tm *Tm = localtime(&time_value);
    struct timeval detail_time; // For seconds and microseconds
    gettimeofday(&detail_time, NULL);

    char ts[32];
    strftime(ts, 20, "%Y%m%d_%H%M%S", Tm);
    printf("%s.%03ld ", ts, ((long)detail_time.tv_usec / 1000) % 1000);
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
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    static int first = 1;
    if (outputFile != NULL)
    {
        FILE *f = fopen(outputFile, "a");
        if (f == NULL)
        {
            printf("Error opening file %s\n", outputFile);
            return;
        }
        else
        {
            if (first)
            {
                first = 0;
                fclose(f);
                f = fopen(outputFile, "w");
                fprintf(f, "Date,Temperature (°F),Pressure (inHg),Rel. Humidity (%%),Gas Resistance (Ohms),IAQ Accuracy,IAQ\n");
                fclose(f);
                f = fopen(outputFile, "a");

#ifdef DEBUG_CONSOLE
                printf("(Dropping first data.)\n");
#endif
                return; // Don't print first set of values - they don't look right.
            }

#ifdef DEBUG_CONSOLE
            printf("[");
            printf("%.3f", (double)timestamp/1e9); // show in seconds.milliseconds
            printf("] T: ");
            printf("%.2f degF", DegreesC_to_DegreesF(temperature * 100.0));
            printf(" | P: ");
            printf("%.2f inHg", Pa_to_inHg(pressure) + pressOffset);
            printf(" | %%rH: ");
            printf("%.2f", humidity * 1000.0 + humOffset);
            printf(" | Ohms: ");
            printf("%.2f", gas);
            printf(" | IAQ: ");
            printf("%.2f", iaq);
            printf(" (");
            printf("%d", iaq_accuracy);
            printf(" | Static IAQ: ");
            printf("%.2f", static_iaq);
            printf(" | CO2e: ");
            printf("%.2f", co2_equivalent);
            printf(" | bVOC: ");
            printf("%.2f)\n", breath_voc_equivalent);
#endif

            time_t time_value = time(NULL);
            struct tm *Tm = localtime(&time_value);
            struct timeval detail_time; // For seconds and microseconds
            gettimeofday(&detail_time, NULL);

            char ts[32];
            strftime(ts, 20, "%Y%m%d_%H%M%S", Tm);
            fprintf(f, "%s.%03ld", ts, ((long)detail_time.tv_usec / 1000) % 1000);

            fprintf(f, ",%0.2lf", DegreesC_to_DegreesF(temperature * 100.0));
            fprintf(f, ",%0.2lf", Pa_to_inHg(pressure) + pressOffset);
            fprintf(f, ",%0.2lf", humidity * 1000.0 + humOffset);
            fprintf(f, ",%0.2lf", gas);
            fprintf(f, ",%d",     iaq_accuracy);
            fprintf(f, ",%0.2lf", iaq);
            fprintf(f, "\n");
            fclose(f);
        }
    }
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
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    printf("config_load()...\n");

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
    printf("config_load(): OK\n");

    return nbytes;
}

// *****************************************************************************

void SaveConfig()
{
    printf("SaveConfig()...\n");

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

    printf("SaveConfig(): OK\n");
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
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    printf("state_load()...\n");

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
    printf("state_load(): OK\n");

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
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
    printf("state_save()...\n");

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
    printf("state_save(): OK\n");

    // -----------------------------------------------------

    SaveConfig();
}

void SaveState()
{
    printf("SaveState()...\n");

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

    printf("SaveState(): OK\n");
}

// *****************************************************************************

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    int error = 1;
    switch (rslt)
    {
        case BME68X_OK:
            /* Do nothing */
            error = 0;
            printf("[%s] OK\n", api_name);
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\n", api_name, rslt);
            break;
    }
    if (error)
    {
        exit(rslt);
    }
}

//******************************************************************************

void FaultHandler(int signal)
{
    // Stop main loop
    terminate = 1;

    //  ^C is not really a fault...
    if (signal == SIGINT)
    {
        std::cerr << "[interrupted by user]" << std::endl;
    }
    //  ...neither is a system terminate signal
    else if (signal == SIGTERM)
    {
        std::cerr << "[terminated by system]" << std::endl;
    }
    else
    {
        std::cerr << "[terminated by signal " << signal << "]" << std::endl;
    }

    exit(EXIT_SUCCESS);
}

/**
 * Use SIGUSR1 to asyncronously save state and config from an external shell.
 */
void SigUsr1Handler(int signo)
{
    printf("\nCaught signal %d (SIGUSR1) - Save state and configuration.\n", signo);
    SaveState();
    SaveConfig();

    signal(SIGUSR1, SigUsr1Handler);
}

void handleProcessExit(void)
{
    printf("Program exit\n");
}

void ExitFunction(int exit_status, void *arg)
{
    terminate = 1;

    SaveState();
    SaveConfig();
    printf("%s program exit, status = %d\n\n", (char *)arg, exit_status);
}

// *****************************************************************************

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
int main(int argc, char *argv[] )
{
    // Set up handlers
    on_exit(ExitFunction, (void *)argv[0]);
    atexit(handleProcessExit);
    signal(SIGINT, FaultHandler);
    signal(SIGSEGV, FaultHandler);

    signal(SIGUSR1, SigUsr1Handler);

    // -----------------------------------------------------

    // Input argument parser
    if (argc == 2)
    {
        outputFile = argv[1];
    }
    else
    {
        printf("Incorrect number of command line parameters.\n");
        printf("Format: %s output_file\n", argv[0]);
        exit(1);
    }
    if (outputFile)
    {
        printf("Output file: %s\n", outputFile);
    }

    // -----------------------------------------------------

    // Create the I2C port based on its chip address.
    mI2Cport = new I2Cdev(BME68X_I2C_ADDR_HIGH);

    return_values_init ret = {BME68X_OK, BSEC_OK};

    // -----------------------------------------------------

    // Open Linux I2C device
    // Set address of the BME68X
    if (mI2Cport->openI2C() < 0)
    {
        perror("openI2C");
        fprintf(stderr, "Failed to open the I2C bus %s\n", dev_name);
        exit(1);
    }

    // -----------------------------------------------------

    bsec_version_t version;
    (void)bsec_get_version(&version);
    printf("BSEC version: %d.%d.%d.%d\n",version.major, version.minor, version.major_bugfix, version.minor_bugfix);

    double sampleInterval = 1.0 / (double)BSEC_SAMPLE_RATE_LP;
    printf("BSEC sampling is set to Low-Power, interval = %.1f sec\n", sampleInterval);

    /* Call to the function which initializes the BSEC library
     * Use low-power mode (3 sec interval) and provide temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, tempOffset, bus_write, bus_read, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        printf("Error while initializing BME68x\n");
        return ret.bme680_status;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        printf("Error while initializing BSEC library\n");
        return ret.bme680_status;
    }

    // -----------------------------------------------------

    // Run the main (endless) loop that queries sensor settings, applies them, and processes the measured data.
    // State is saved every 10000 samples, which means every 10000 * 3 secs = 500 minutes (8 hrs 20 min)
    // State is saved every 15 minutes, which means every 900 / 3 = 300 samples
    const uint32_t save_intvl = 300;
    // bsec_iot_loop(get_timestamp_us, output_ready, state_save, save_intvl);

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

    // -----------------------------------------------------

    while (!terminate && (n_samples <= 5))
    {
        /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
        time_stamp = (uint64_t)get_timestamp_ns();

        /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
        bsec_sensor_control(time_stamp, &sensor_settings);
        if (0) { // @XXX
            printf("bsec_sensor_control(): sensor_settings\n");
            printf("  next_call                %lld\n", sensor_settings.next_call              );
            printf("  process_data             0x%02X\n", sensor_settings.process_data         );
            printf("  heater_temperature       %d\n", sensor_settings.heater_temperature       );
            printf("  heating_duration         %d\n", sensor_settings.heating_duration         );
            printf("  run_gas                  %d\n", sensor_settings.run_gas                  );
            printf("  pressure_oversampling    %d\n", sensor_settings.pressure_oversampling    );
            printf("  temperature_oversampling %d\n", sensor_settings.temperature_oversampling );
            printf("  humidity_oversampling    %d\n", sensor_settings.humidity_oversampling    );
            printf("  trigger_measurement      %d\n", sensor_settings.trigger_measurement      );
        } // @XXX

        /* Trigger a measurement if necessary */
        bme680_bsec_trigger_measurement(&sensor_settings);

        /* Read data from last measurement */
        num_bsec_inputs = 0;
        bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);

        /* Time to invoke BSEC to perform the actual processing */
        bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);

        /* Increment sample counter */
        n_samples++;

        /* Retrieve and store state */
        if (n_samples >= save_intvl)
        {
            bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
            if (bsec_status == BSEC_OK)
            {
                state_save(bsec_state, bsec_state_len);
            }
            n_samples = 0;
        }

        /* Compute how long we need to sleep until we can call bsec_sensor_control() next */
        time_stamp_interval_ms = ((uint64_t)sensor_settings.next_call - (uint64_t)get_timestamp_ns()) / 1000000;

        if (time_stamp_interval_ms > 0)
        {
            usleep(time_stamp_interval_ms * 1000);
        }
    }

    // -----------------------------------------------------

    // Delete I2C port
    delete mI2Cport;

    return ret.bme680_status;
}

// *****************************************************************************
/*! @}*/

