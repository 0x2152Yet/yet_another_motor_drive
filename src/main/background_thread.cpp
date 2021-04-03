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
//  Lowest priority application thread
//
//  This thread performs background activities that do not have any particular
//  timing requirement.  The only thing lower priority is the OS idle task.
//
////////////////////////////////////////////////////////////////////////////////
#include "background_thread.h"
#include "data_logger.h"
#include "os_interface.h"
#include "uart_interface.h"

#include <stdio.h>

//
//  We create the class that manages the interface with the UART.
//
UARTInterface theUARTInterface;


////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
BackgroundThread::BackgroundThread()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Performs initializations required before the scheduler is started
//
////////////////////////////////////////////////////////////////////////////////
void BackgroundThread::initBackgroundThread()
{
    //
    //  We register this thread with the OS.
    //
    theOSInterface.createThread(
        theBackgroundThread.backgroundThreadEntryPoint,
        threadInfo::BackgroundThreadPriority,
        threadInfo::BackgroundThreadStackSizeLongwords,
        this,
        threadInfo::BackgroundThreadName);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Static entry point required by the OS
//
//  This should be provided a pointer to the actual instance of the
//  thread class.  It will call the main loop by reference.
//
////////////////////////////////////////////////////////////////////////////////
void BackgroundThread::backgroundThreadEntryPoint(void * const thisPtr)
{
    if (thisPtr != nullptr)
    {
        //
        //  We call the main loop using the passed in "this" pointer.
        //
        BackgroundThread *const thisTaskPointer =
            reinterpret_cast<BackgroundThread *const>(thisPtr);
        thisTaskPointer->BackgroundThreadLoop();
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
//  Main loop for the Background Thread
//
//  This is invoked by the static thread entry point.  It is scheduled by
//  the RTOS and never exits
//
////////////////////////////////////////////////////////////////////////////////
void BackgroundThread::BackgroundThreadLoop()
{

    //
    //  We initialize the test support UART.
    //
    theUARTInterface.initializeUARTInterface();

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
