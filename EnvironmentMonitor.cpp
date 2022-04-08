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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <ctime>
#include <cstring>

#include <signal.h>
#include <execinfo.h>

#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE
#endif
#include <ucontext.h>

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include <iterator>

#include "Logger.h"
#include "Timer.h"

#include "MsgQueue.h"
#include "DataItem.h"
#include "DataStore.h"

#include "SystemClock.h"

#include "MasterTask.h"
#include "UITask.h"
#include "DataLog.h"
#include "LCDTask.h"

//#include "LightSensor.h"
//#include "Pwr_5V_Sensor.h"
//#include "Pwr_3p3V_Sensor.h"
#include "ADC_Sensors.h"
#include "BME280Sensor.h"
#include "BME680Sensor.h"

#include "EnvironmentMonitor.h"

//-----------------------------------------------------------------------------

//  Exception handling
void SetupExceptionHandler();

void SaveSystemState(void);

static Logger *pLogger = nullptr;

static int gStateCount = 0;
static uint8_t gTerminate = 0;

//-----------------------------------------------------------------------------

// /proc/meminfo (MemFree:)
int64_t getCPUFreeMem()
{
    int64_t ret = -1;

#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
    const std::string meminfo("/proc/meminfo");
    std::ifstream infofile;
    infofile.open (meminfo, std::ios::in);

    if (infofile.is_open()) {
        std::string line;
        while (std::getline(infofile,line)) {
            // Read lines until find "MemFree:"
            std::string::size_type n = line.find("MemFree:");
            if (n != std::string::npos) {
                break;
            }
        }

        // Split string into words (vector of strings)
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                std::istream_iterator<std::string>(),
                back_inserter(tokens));

        ret = std::stoll(tokens[1]);
        if ("kB" == tokens[2]) {
            ret *= 1024;
        } else if ("MB" == tokens[2]) {
            ret *= 1024 * 1024;
        }

        infofile.close();
    }
#else
    // @TODO
#endif

    return ret;
}

//-----------------------------------------------------------------------------

// /proc/meminfo (MemTotal:)
int64_t getCPUTotalMem()
{
    int64_t ret = -1;

#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
    const std::string meminfo("/proc/meminfo");
    std::ifstream infofile;
    infofile.open (meminfo, std::ios::in);

    if (infofile.is_open()) {
        std::string line;
        while (std::getline(infofile,line)) {
            // Read lines until find "MemTotal:"
            std::string::size_type n = line.find("MemTotal:");
            if (n != std::string::npos) {
                break;
            }
        }

        // Split string into words (vector of strings)
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                std::istream_iterator<std::string>(),
                back_inserter(tokens));

        ret = std::stoll(tokens[1]);
        if ("kB" == tokens[2]) {
            ret *= 1024;
        } else if ("MB" == tokens[2]) {
            ret *= 1024 * 1024;
        }

        infofile.close();
    }
#else
    // @TODO
#endif

    return ret;
}

//-----------------------------------------------------------------------------

// /opt/vc/bin/vcgencmd measure_temp
double getCPUTemp()
{
    double systemp = -1.0;

#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
    double millideg = 0.0;
    FILE *thermal;

    thermal = fopen("/sys/class/thermal/thermal_zone0/temp","r");
    if (thermal != NULL)
    {
        if (fscanf(thermal,"%lf",&millideg) < 1) {
            millideg = -273000.0;
        }
        fclose(thermal);
        systemp = millideg / 1000;
    }
#else
    // @TODO
#endif

    return systemp;
}

//-----------------------------------------------------------------------------

// Timer to read internal sensor data (CPU and system board items)
class StatTimer : public Timer
{
public:
    StatTimer(const std::string &name,
            const IntervalMs &start,
            const IntervalMs &interval,
            Logger *plogger) :
       Timer(name, start, interval),
       m_plogger(plogger)
    {
        cpu_mem_total = dynamic_cast<DataItem<uint64_t> *>(DataStore::getInstance()->GetDataItem(CPU_MEM_TOTAL));
        cpu_mem_free = dynamic_cast<DataItem<uint64_t> *>(DataStore::getInstance()->GetDataItem(CPU_MEM_FREE));
        cpu_temp = dynamic_cast<DataItem<double> *>(DataStore::getInstance()->GetDataItem(CPU_TEMP));
        m_plogger->log(eLOG_INFO, "StatTimer(%s) Start:%d, Intv:%d", name.c_str(), start, interval);
    }

    void Start()
    {
        Timer::Start();
    }

    void Reset(const IntervalMs &interval)
    {
        Timer::Reset(interval);
    }

    void ExpirationRoutine() override
    {
        // CPU_MEM_TOTAL
        int64_t mtotal = getCPUTotalMem();
        if (mtotal > 0)
        {
            cpu_mem_total->setValue(mtotal);
        }

        // CPU_MEM_FREE
        int64_t mfree = getCPUFreeMem();
        if (mfree > 0)
        {
            cpu_mem_free->setValue(mfree);
        }

        // CPU_TEMP
        double ctmp = getCPUTemp();
        if (ctmp > 0.0)
        {
            cpu_temp->setValue(ctmp);
        }
    }

private:
    Logger *m_plogger;
    DataItem<uint64_t> *cpu_mem_total;
    DataItem<uint64_t> *cpu_mem_free;
    DataItem<double> *cpu_temp;
};

//-----------------------------------------------------------------------------

int main(int argc, char **argv)
{
    //  Create the system signal/exception handlers.
    SetupExceptionHandler();

    // Create system logger object
    Logger logger("../data/EnvironmentMonitor", eLOG_DEBUG);
    pLogger = &logger;

    logger.MirrorToStdOut();

    logger.log(eLOG_INFO, "%s v%d.%d", basename(argv[0]),
        EnvironmentMonitor_VERSION_MAJOR, EnvironmentMonitor_VERSION_MINOR);

    //-------------------------------------------------------------------------

    // Create DataStore that owns all data objects in the system.
    // This includes data that sensor tasks write to and the UI task reads.
    // This is a singleton so system objects can get access to it by calling getInstance.

    DataStore systemData;

    //-------------------------------------------------------------------------

    // Create all system task objects
    // General task parameters: name, id, *Logger

    UITask ui{"User Interface", UI_TASK_ID, &logger};
    DataLogTask dataLog{"Data Log", DATA_LOG_TASK_ID, &logger};
    LCDTask lcd{"LCD Display", LCD_TASK_ID, &logger};

    // Define sensor tasks (Type = flow, pressure, temperature, light, humidity, etc.)
    // @NOTE the Data ID (DID) doubles as a task ID.
    // Format:
    // TaskClass       taskVar     {NAME,                 DID,           LOG_PTR,  TYPE,  SAMPLE_FREQ, REPORT_PERIOD);
    //LightSensorTask   lightSensor  {"Ambient Light"     , LIGHT_SENSE   , &logger, "Light"  , 20.0, 2.0}; // 20Hz, update every 2 seconds
    //Pwr5VSensorTask   pwr5vSensor  {"5V Power Supply"   , PWR_5V_SENSE  , &logger, "Voltage", 20.0, 2.0}; // 20Hz, update every 2 seconds
    //Pwr3p3VSensorTask pwr3p3vSensor{"3.3V Power Supply" , PWR_3P3V_SENSE, &logger, "Voltage",  5.0, 1.0}; //  5Hz, update every 1 seconds
    ADCSensorsTask    adcSensors   {"ADC Sensors"       , ADC_BASE   , &logger, "Various", 20.0, 2.0}; // 20Hz, update every 2 seconds
    BME280SensorTask  bme280Sensor {"BME280 Env. Sensor", BME280_BASE, &logger, "Various", 10.0, 3.0}; // 10Hz, update every 3 seconds

    // The BME680 software automatically delays 3 seconds per reading.
    // Reporting is done on every sample period.
    BME680SensorTask  bme680Sensor {"BME680 Env. Sensor", BME680_BASE, &logger, "Various", 1.0, 1.0};

    // @NOTE: Sensor tasks have their own timers to read and average sensor data, then write to DataStore objects.

    // Timer to update internal HW stats
    StatTimer statTimer("IntStatTimer", (Timer::IntervalMs)2000, (Timer::IntervalMs)1000, &logger);

    //-------------------------------------------------------------------------

    // Create MasterTask object (which owns all other task objects)
    MasterTask masterTask("Master Task", &logger);
    // Don't start it yet...

    // Add tasks
    masterTask.AddAppTask(&ui);
    masterTask.AddAppTask(&dataLog);
    masterTask.AddAppTask(&lcd);
    //masterTask.AddAppTask(&lightSensor);
    //masterTask.AddAppTask(&pwr5vSensor);
    //masterTask.AddAppTask(&pwr3p3vSensor);
    masterTask.AddAppTask(&adcSensors);
    masterTask.AddAppTask(&bme280Sensor);
    masterTask.AddAppTask(&bme680Sensor);

    //-------------------------------------------------------------------------

    // @TODO Create CLI object

    //-------------------------------------------------------------------------

    logger.log(eLOG_INFO, "main() Current OS uptime: %.03f seconds", SystemClock::GetSeconds());

    // Start the master task, which will bring up all other tasks.
    masterTask.Start();
    // MasterTask will terminate if no tasks were added.
    if (masterTask.isTerminated()) {
        logger.log(eLOG_CRIT, "main() MasterTask has terminated.");
        return EXIT_FAILURE;
    }

    // Start internal HW stats-gathering timer
    statTimer.Start();

    //-------------------------------------------------------------------------

    Task::Sleep(1 * 1000);

    // This would be the application background in a normal system...
    while (!masterTask.isTerminated() && !gTerminate)
    {
        // @TODO - Background processing, maybe including CLI...

        // ------------------------------------------------

        Task::Sleep(1 * 1000);

        // Do something every 5 minutes
        if (++gStateCount >= (5*60))
        {
            logger.log(eLOG_DEBUG, "----------------------------------------");
            logger.log(eLOG_DEBUG, "main() Getting system info...");
            logger.log(eLOG_DEBUG, masterTask.GetSystemInfo());
            logger.log(eLOG_DEBUG, "----------------------------------------");
            gStateCount = 0;
        }

        // ------------------------------------------------
    } // end while()

    SaveSystemState();

    // Main loop terminated - terminate all subtasks.
    masterTask.TerminateSystem();
    Task::Sleep(1 * 1000);

    logger.log(eLOG_INFO, "----------------------------------------");
    logger.log(eLOG_INFO, masterTask.GetSystemInfo());
    logger.log(eLOG_INFO, "----------------------------------------");

    statTimer.Stop();

    //-------------------------------------------------------------------------

    logger.log(eLOG_INFO, "main() Ending");

    logger.WriteToFile();
    std::cout << "Log file '" << logger.name() << "' " << logger.Size() << " bytes" << std::endl;

    //-------------------------------------------------------------------------

    return EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------


// Save system device state
void SaveSystemState(void)
{
    // Make main loop exit
    gTerminate = 1;

    pLogger->log(eLOG_CRIT, "Saving system state");
    pLogger->WriteToFile();
    std::cerr << std::endl;
}

/*! \brief  Get fault description.
 *
 *  This function converts the signal/code combo into descriptive text.
 *
 *  \param signal   Signal ID
 *  \param code     signal subcode
 *  \return         string with description
 */
const char* FaultString(int signal, int code)
{
    if(SIGILL==signal)
    {
        switch(code)
        {
        case ILL_ILLOPC:
            return "illegal opcode";
        case ILL_ILLOPN:
            return "illegal operand";
        case ILL_ILLADR:
            return "illegal addressing mode";
        case ILL_ILLTRP:
            return "illegal trap";
        case ILL_PRVOPC:
            return "privileged register";
        case ILL_COPROC:
            return "coprocessor error";
        case ILL_BADSTK:
            return "internal stack error";
        case 0:
            return "SIGILL";
        default:
            return "unknown SIGILL code";
        }
    }

    if(SIGFPE==signal)
    {
        switch(code)
        {
        case FPE_INTDIV:
            return "integer divide by zero";
        case FPE_INTOVF:
            return "integer overflow";
        case FPE_FLTDIV:
            return "floating point divide by zero";
        case FPE_FLTOVF:
            return "floating point overflow";
        case FPE_FLTUND:
            return "floating point underflow";
        case FPE_FLTRES:
            return "floating point inexact result";
        case FPE_FLTINV:
            return "floating point invalid operation";
        case FPE_FLTSUB:
            return "subscript out of range";
        case 0:
            return "SIGFPE";
        default:
            return "unknown SIGFPE code";
        }
    }

    if(SIGSEGV==signal)
    {
        switch(code)
        {
        case SEGV_MAPERR:
            return "address not mapped to object";
        case SEGV_ACCERR:
            return "invalid permissions for mapped object";
        case 0:
            return "SIGSEGV";
        default:
            return "unknown SIGSEGV code";
        }
    }

    if(SIGBUS==signal)
    {
        switch(code)
        {
        case BUS_ADRALN:
            return "invalid address alignment";
        case BUS_ADRERR:
            return "non-existent physical address";
        case BUS_OBJERR:
            return "object specific hardware error";
        case 0:
            return "SIGBUS";
        default:
            return "unknown SIGBUS code";
        }
    }

    if (SIGINT==signal)
    {
        return "SIGINT; Terminated by user";
    }

    if (SIGABRT==signal)
    {
        if (code > 0) {
            return "Kernel-generated signal";
        }
#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
        switch(code)
        {
        case SI_TKILL:
            return "Self-abort; tkill()";

        case SI_SIGIO:
            return "IO";

        case SI_ASYNCIO:
            return "Async IO";

        case SI_MESGQ:
            return "Msg Queue state change";

        case SI_TIMER:
            return "Timer expiration";

        case SI_QUEUE:
            return "SIGQUEUE";

        case SI_USER:
            return "SIGABRT; User-generated kill, sigend, raise";

        default:
            return "External abort";
        }
#else
        switch(code)
        {
        case SI_USER:
            return "Self-abort";

        case SI_QUEUE:
            return "SIGQUEUE";

        case SI_TIMER:
            return "Timer expiration";

        case SI_ASYNCIO:
            return "Async IO";

        case SI_MESGQ:
            return "Msg Queue state change";

        default:
            return "External abort";
        }
#endif
    }

    return "Unhandled signal handler";
}

//-----------------------------------------------------------------------------

/*! \brief Process processor exception signals
 *
 *  This function collects the exception information and prints a standard
 *  diagnostic message via qDebug
 *
 *  @NOTE: Linux signal handling on some archs works by the kernel replacing the
 *  return address of the faulting function (within the stack) with the signla
 *  "unwinding" function, which would later restore it. What this means is that
 *  the backtrace we get is missing the single most important bit of information:
 *  the addres of the faulting function.  We can fix this by going (if necessary)
 *  to the signal structure and extracting the saved function address.
 *
 *  \param signal   The signal being trapped.
 *  \param siginfo  The signal information
 *  \param context  The process context at the time of the exception
 */
void FaultHandler(int signal, siginfo_t * siginfo, void *context)
{
    // ^C is not really a fault.
    // Catch it to update system state machine.
    if (signal == SIGINT)
    {
        std::cerr << "[interrupted by user]" << std::endl;
        // Make main loop exit
        gTerminate = 1;

//        // Re-register exception handlers to be able to re-catch ^C
//        SetupExceptionHandler();
        return;
    }

    // A system terminate signal also isn't a fault.
    // Catch it to gracefully shut down the system.
    if (signal == SIGTERM)
    {
        std::cerr << "[terminated by system]" << std::endl;
        // Make main loop exit
        gTerminate = 1;
       return;
    }

    // Real fault handling
    const int maxbtsize = 128; // display up to this many back calls
    const int max_line_size = 256; // max no. of chars in each backtrace line

    void*  stack[maxbtsize];
    ucontext_t * cntxt = (ucontext_t *)context;

    // Output buffers
    char  exAddr[max_line_size];
    char  exText[max_line_size];
    char  exError[max_line_size];
    char  exBackTrace[maxbtsize * max_line_size + 1];

    memset(stack, 0, sizeof(stack));

    //  Get exception address
    snprintf(exAddr, sizeof(exAddr), "0x%p", (void *)siginfo->si_addr);

    //  Get exception codes & description
    snprintf(exText, sizeof(exText), "{%d, %d} %s", signal, siginfo->si_code, FaultString(signal, siginfo->si_code));

    //  Get any error code associated with exception
    snprintf(exError, sizeof(exError), "%d", siginfo->si_errno);

    //  Get backtrace of stack.  For some architectures, replace missing address.
    int btSz  = backtrace((void**)stack, maxbtsize);
#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
    stack[3] = (void *)(cntxt->uc_mcontext.arm_ip);     // RaspberryPi
//    stack[3] = (void *)(cntxt->uc_mcontext.gregs[REG_RIP]);     // x86_64
//    stack[3] = (void *)(cntxt->uc_mcontext.gregs[REG_EIP]);   // i386
//    stack[3] = (void *)(cntxt->uc_mcontext.regs->nip);        // PPC
#else
//    stack[3] = (void *)(cntxt->uc_mcontext->__ss.__rip);        // x86_64 MacOs
    stack[3] = (void *)(cntxt->uc_mcontext->__ss.__pc);        // M1 Pro MacOs
#endif

    //  Get backtrace info
    if (btSz <= 0)
    {
        // None available
        strcpy(exBackTrace, "not available");
    }
    else
    {
        //  Try to get strings with symbolic info
        char ** symbols = backtrace_symbols((void**)stack, btSz);
        if (symbols)
        {
            char tmpStr[max_line_size];
            for (int t=0; t < btSz; t++)
            {
                snprintf(tmpStr, sizeof(tmpStr), "\n%2d [%p] %s", t, stack[t], symbols[t]);
                strncat(exBackTrace, tmpStr, (maxbtsize * max_line_size));
            }
        }
        else
        {
            // Send symbolic info to a text file.  (This backtrace funcion takes a file
            // descriptor, which we have to sneak out of the FILE structure via "_file".)
            std::string timestamp = pLogger->TimeStamp();
            strcpy(exBackTrace, (timestamp + std::string(".backtrace")).c_str());
            FILE* crashFile = fopen(exBackTrace, "w");
#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
            backtrace_symbols_fd(stack, btSz, crashFile->_fileno);
#else
            backtrace_symbols_fd(stack, btSz, crashFile->_file);
#endif
            fclose(crashFile);
        }
        free(symbols);
    }

    //  Build the message
    std::cerr << std::endl << "*** SYSTEM_EXCEPTION FAIL ::FaultHandler(): " << std::endl
            << "  Error: " << exError << std::endl
            << "  Addr: " << exAddr << std::endl
            << "  Text: " << exText << std::endl
            << "  Backtrace: " << exBackTrace
            << std::endl << std::endl;

    exit(signal);
}

//-----------------------------------------------------------------------------

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
    }

    if (-1 == sigaction (SIGILL, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGILL...)" << std::endl;
    }

    if (-1 == sigaction (SIGINT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGINT...)" << std::endl;
    }

    if (-1 == sigaction (SIGFPE, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGFPE...)" << std::endl;
    }

    if (-1 == sigaction (SIGBUS, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGBUS...)" << std::endl;
    }

    if (-1 == sigaction (SIGQUIT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGQUIT...)" << std::endl;
    }

    if (-1 == sigaction (SIGABRT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGABRT...)" << std::endl;
    }

    if (-1 == sigaction (SIGTERM, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGTERM...)" << std::endl;
    }

#if defined(_POSIX_C_SOURCE) && !defined(_DARWIN_C_SOURCE)
    if (-1 == sigaction (SIGSTKFLT, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGSTKFLT...)" << std::endl;
    }
#endif

    if (-1 == sigaction (SIGSYS, &action, NULL))
    {
        std::cerr << "SYSTEM_CALL_FAILED FAIL cExceptionHandler() sigaction(SIGSSYS...)" << std::endl;
    }
}

//-----------------------------------------------------------------------------
