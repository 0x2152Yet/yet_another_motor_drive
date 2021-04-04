////////////////////////////////////////////////////////////////////////////////
//
//  Yet Another Motor Drive
//
//  MIT License
//
//  Copyright (c) 2021 Michael F. Kaufman
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
//  Lowest priority application task
//
//  This task performs background activities that do not have any particular
//  timing requirement.  The only thing lower priority is the OS idle task.
//
////////////////////////////////////////////////////////////////////////////////
#include "background_task.h"

#include "data_logger.h"
#include "os_interface.h"
#include "uart_interface.h"

#include <stdio.h>

//
//  We create the class that manages the interface with the UART.
//
UARTInterface theUARTInterface;

//
//  We print these at startup.
//
extern volatile int buildCount;
extern volatile char absoluteBuildTime[];

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
BackgroundTask::BackgroundTask()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Performs initializations required before the scheduler is started
//
////////////////////////////////////////////////////////////////////////////////
void BackgroundTask::initBackgroundTask()
{
    //
    //  We register this task with the OS.
    //
    theOSInterface.createTask(
        theBackgroundTask.backgroundTaskEntryPoint,
        taskInfo::BackgroundTaskPriority,
        taskInfo::BackgroundTaskStackSizeLongwords,
        this,
        taskInfo::BackgroundTaskName);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Static entry point required by the OS
//
//  This should be provided a pointer to the actual instance of the
//  task class.  It will call the main loop by reference.
//
////////////////////////////////////////////////////////////////////////////////
void BackgroundTask::backgroundTaskEntryPoint(void * const thisPtr)
{
    if (thisPtr != nullptr)
    {
        //
        //  We call the main loop using the passed in "this" pointer.
        //
        BackgroundTask *const thisTaskPointer =
            reinterpret_cast<BackgroundTask *const>(thisPtr);
        thisTaskPointer->BackgroundTaskLoop();
    }

    //
    //  This call should not get here.  If it does, we'll loop with a delay
    //  to support debugging.
    //
    while (true)
    {
        OSInterface::delay(1000U);
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  Main loop for the Background task
//
//  This is invoked by the static task entry point.  It is scheduled by
//  the RTOS and never exits
//
////////////////////////////////////////////////////////////////////////////////
void BackgroundTask::BackgroundTaskLoop()
{

    //
    //  We initialize the test support UART.
    //
    theUARTInterface.initializeUARTInterface();

    //
    //  We display build date, time, etc.
    //
    printBuildInfo();

    //
    //  This is the main background loop.
    //
    while (true)
    {
        OSInterface::delay(100U);

        //
        //  We print the contents of the data log if there is data in it.
        //
        theDataLogger.printDataLogBuffer();

        //
        //  We print the low rate data if there is any to print.
        //
        theDataLogger.printLowRateItems();
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  Prints general build information
//
////////////////////////////////////////////////////////////////////////////////
void BackgroundTask::printBuildInfo()
{
    printf("\r\n\n");
    printf("          Yet Another Motor Drive         \r\n");
    printf("    Copyright (c) 2021, Michael F. Kaufman\r\n");
    printf("            0xEB90, 0x2152-yet.           \r\n\n");

    printf("Build time  : %s\r\n", absoluteBuildTime);
    printf("Build count : %d\r\n\n", static_cast<int>(buildCount));

    fflush(stdout);
}

