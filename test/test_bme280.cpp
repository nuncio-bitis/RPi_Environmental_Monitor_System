/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

 /*
  * Build:
  *  gcc -std=c99 test_bme280.c ../hardware/source/bme280.c -o test_bme280 -I/usr/include/ -I../hardware/include -lwiringPi
  */

/**
 * \ingroup bme280
 * \defgroup bme280Examples Examples
 * @brief Reference Examples
 */

/*!
 * @ingroup bme280Examples
 * @defgroup bme280GroupExampleLU linux_userspace
 * @brief Linux userspace test code, simple and mose code directly from the doco.
 * compile like this: gcc linux_userspace.c ../bme280.c -I ../ -o bme280
 * tested: Raspberry Pi.
 * Use like: ./bme280 /dev/i2c-0
 * \include linux_userspace.c
 */

/******************************************************************************/
/*!                         System header files                               */

#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>

#include <signal.h>
#include <execinfo.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <I2Cdev.h>

/******************************************************************************/
/*!                         Own header files                                  */
#define BME280_FLOAT_ENABLE
#include "bme280.h"

/******************************************************************************/
/*!                               Structures                                  */

static const uint32_t gDelay = 2000; // Delay in readings, milliseconds

#define DESTZONE "TZ=EST5EDT"

#ifdef BME280_FLOAT_ENABLE
// Offsets based on Eve Weather sensor
static const float tempOffset  = +0.52; // degC
static const float humOffset   = +6.90; // %
static const float pressOffset = +1.02; // hPa
#endif

static char *outputFile = NULL;

// I2C port for reading and writing.
static I2Cdev * mI2Cport = NULL;

static uint8_t terminate = 0;

/****************************************************************************/
/*!                         Functions                                       */

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs.
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *  @return void.
 *
 */
//void user_delay_us(uint32_t period, void *intf_ptr);

/*!
 * @brief Function for print the temperature, humidity and pressure data.
 *
 * @param[out] comp_data    :   Structure instance of bme280_data
 *
 * @note Sensor data whose can be read
 *
 * sens_list
 * --------------
 * Pressure
 * Temperature
 * Humidity
 *
 */
void print_sensor_data(struct bme280_data *comp_data);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[out] data          : Pointer to the data buffer to store the read data.
 *  @param[in] len            : No of bytes to read.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval 0 -> Success
 *  @retval > 0 -> Failure Info
 *
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[in] data           : Pointer to the data buffer whose value is to be written.
 *  @param[in] len            : No of bytes to write.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 *  @return Status of execution
 *
 *  @retval BME280_OK -> Success
 *  @retval BME280_E_COMM_FAIL -> Communication failure.
 *
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);

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

//******************************************************************************

/*! \brief Process processor exception signals
 *
 *  \param signal   The signal being trapped.
 *  \param siginfo  The signal information
 *  \param context  The process context at the time of the exception
 */
void FaultHandler(int signal, siginfo_t * siginfo, void *context)
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

//******************************************************************************

/*! \brief Create the exception handling system.
 *
 *  This function registers the handler for specified system exceptions.
 */
void SetupExceptionHandler()
{
    struct sigaction action;

    memset(&action, 0, sizeof (action));
    action.sa_sigaction = FaultHandler;
    sigemptyset (&action.sa_mask);
    action.sa_flags = SA_SIGINFO | SA_RESETHAND;

    if (-1 == sigaction (SIGSEGV, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGSEGV...)" << std::endl;
    };

    if (-1 == sigaction (SIGILL, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGILL...)" << std::endl;
    };

    if (-1 == sigaction (SIGINT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGINT...)" << std::endl;
    };

    if (-1 == sigaction (SIGFPE, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGFPE...)" << std::endl;
    };

    if (-1 == sigaction (SIGBUS, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGBUS...)" << std::endl;
    };

    if (-1 == sigaction (SIGQUIT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGQUIT...)" << std::endl;
    };

    if (-1 == sigaction (SIGABRT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGABRT...)" << std::endl;
    };

    if (-1 == sigaction (SIGTERM, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGTERM...)" << std::endl;
    };

#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
    if (-1 == sigaction (SIGSTKFLT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGSTKFLT...)" << std::endl;
    };
#endif

    if (-1 == sigaction (SIGSYS, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGSSYS...)" << std::endl;
    };

    if (-1 == sigaction (SIGUSR1, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGUSR1...)" << std::endl;
    };

    if (-1 == sigaction (SIGUSR2, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGUSR2...)" << std::endl;
    };
};

//******************************************************************************

void ExitFunction(int exit_status, void *arg)
{
    printf("\n%s program exit, status = %d\n\n", (char *)arg, exit_status);
}

// *****************************************************************************

/*!
 * @brief This function starts execution of the program.
 */
int main(int argc, char* argv[])
{
    SetupExceptionHandler();
    putenv((char *)DESTZONE); // Switch to destination time zone
    on_exit(ExitFunction, (void *)argv[0]);

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
    mI2Cport = new I2Cdev(BME280_I2C_ADDR_PRIM);
    struct bme280_dev dev;

    /* Variable to define the result */
    int8_t rslt = BME280_OK;

    // -----------------------------------------------------

    // Open the I2C port
    if (mI2Cport->openI2C() < 0)
    {
        perror("openI2C");
        fprintf(stderr, "Failed to open the I2C bus %s\n", I2C1DeviceName);
        exit(1);
    }

    // -----------------------------------------------------

    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    //dev.delay_us = user_delay_us;

    /* Update interface pointer with the structure that contains both device address and file descriptor */
    dev.intf_ptr = NULL; // NOT NEEDED

    /* Initialize the bme280 */
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
        exit(1);
    }

    // -----------------------------------------------------

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
        fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);
        // Force program termination before it starts.
        terminate = 1;
    }
    else
    {
        /* Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
            * and the oversampling configuration.
            */
        req_delay = bme280_cal_meas_delay(&dev.settings);
        printf("bme280_cal_meas_delay: %d uS\n", req_delay);
        printf("\n");
    }

    // Initialize the conversion process; Set the sensor to forced mode
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
        terminate = 1;
    }

    /* Continuously stream sensor data */
    uint8_t max_iterations = 10; // Limit iterations because this is a test program.
    while (!terminate && max_iterations--)
    {
        // Delay between readings
        usleep(gDelay*1000);

        /* Set the sensor to forced mode */
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
            break;
        }

        /* Wait for the measurement to complete and print data */
        usleep(req_delay);

        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
            break;
        }

        print_sensor_data(&comp_data);
    } // end while(!terminate)

    // Check overall operation result.
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
    }

    // -----------------------------------------------------

    // Delete I2C port
    delete mI2Cport;

    return rslt;
}

// *****************************************************************************

/*!
 * @brief This function reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_OK;
    if (mI2Cport->readBytes(reg_addr, len, data) <= 0)
    {
        ret = BME280_E_COMM_FAIL;
    }

    return ret;
}

// *****************************************************************************

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t ret = BME280_OK;
    if (mI2Cport->writeBytes(reg_addr, len, (uint8_t *)data) <= 0)
    {
        ret = BME280_E_COMM_FAIL;
    }

    return ret;
}

// *****************************************************************************

/*!
 * @brief This API used to print the sensor temperature, pressure and humidity data.
 */
void print_sensor_data(struct bme280_data *comp_data)
{
    float temp, press, hum;

    // Get readings, compensate for offsets
#ifdef BME280_FLOAT_ENABLE
    temp = comp_data->temperature + tempOffset;
    press = (0.01 * comp_data->pressure) + pressOffset;
    hum = comp_data->humidity + humOffset;
#else
#ifdef BME280_64BIT_ENABLE
    temp = 0.01f * comp_data->temperature;
    press = 0.0001f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    timestamp();
    printf("%0.2lf °F %0.2lf inHg %0.2lf %%rH\n", DegreesC_to_DegreesF(temp), hPa_to_inHg(press), hum);

    // Write measurement to output file if specified.
    static int first = 1;
    if (outputFile != NULL)
    {
        FILE *f = fopen(outputFile, "a");
        if (f == NULL)
        {
            printf("Error opening file!\n");
        }
        else
        {
            if (first)
            {
                first = 0;
                fclose(f);
                f = fopen(outputFile, "w");
                fprintf(f, "Date,Temperature (°F),Pressure (inHg),Rel. Humidity (%%)\n");
                fclose(f);
                f = fopen(outputFile, "a");
            }

            time_t time_value = time(NULL);
            struct tm *Tm = localtime(&time_value);
            struct timeval detail_time; // For seconds and microseconds
            gettimeofday(&detail_time, NULL);

            char ts[32];
            strftime(ts, 20, "%Y%m%d_%H%M%S", Tm);
            fprintf(f, "%s.%03ld,", ts, ((long)detail_time.tv_usec / 1000) % 1000);

            fprintf(f, "%0.2lf,%0.2lf,%0.2lf\n", DegreesC_to_DegreesF(temp), hPa_to_inHg(press), hum);
            fclose(f);
        }
    }
}

// *****************************************************************************
