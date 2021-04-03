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
//  This class provides a very thin wrapper around the RTOS functions that we
//  use.  It also includes definitions for our tasking configuration and provides
//  some utility functions required by the RTOS.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "FreeRTOS.h"
#include "global_definitions.h"
#include "task.h"

//
//  This namespace defines parameters related to the various tasks.
//
namespace taskInfo
{
    //
    //  These define the configuration of each task.
    //
    const uint32_t MotorControlTaskStackSizeLongwords = 1024U;
    const uint32_t MotorControlTaskPriority           = configMAX_PRIORITIES - 4U;
    const char *const  MotorControlTaskName           = "MtrCtrl";

    const uint32_t BackgroundTaskStackSizeLongwords   = 1024U;
    const uint32_t BackgroundTaskPriority             = MotorControlTaskPriority - 1U;
    const char *const  BackgroundTaskName             = "BkGnd";
}


class OSInterface
{
public:
    OSInterface();
    ~OSInterface() {};

    //
    //  This should be called by each task to create itself before the OS
    //  scheduler is started.  This requires the following items:
    //    A function pointer to the task's static entry point
    //    The task's priority
    //    The task's stack size in longwords
    //    A parameter to pass the static entry point.  This is usually the
    //      task's class "this" pointer.
    //    A name to give the task
    //
    void createTask(
        TaskFunction_t     taskStaticEntryPointPtr,
        uint32_t           taskPriority,
        uint32_t           taskStackSize_longwords,
        void * const       taskParameter,
        const char * const taskName);

    //
    //  This starts the OS scheduler.  This method does not return to the
    //  caller.
    //
    static void startScheduler();

    //
    //  This performs a yielding delay.  The calling task sleeps for at least
    //  the provided time period.
    //
    static void delay(const time_ms delayTime);

private:


};

extern "C"
{
    //
    //  These may be called from within ISRs to mask and restore all system
    //  interrupts.  They should be used with great care and only very briefly.
    //  Note that if these are called from non-ISRs, the code will likely trap.
    //
    uint32_t maskAllInterruptsFromISR() __attribute__((naked));
    void restoreInterruptMaskFromISR(uint32_t originalMask) __attribute__((naked));

    //
    //  These are hooks and callbacks the RTOS requires.
    //
    void vApplicationMallocFailedHook(void);
    void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
    void vApplicationGetIdleTaskMemory(
        StaticTask_t **ppxIdleTaskTCBBuffer,
        StackType_t **ppxIdleTaskStackBuffer,
        uint32_t *pulIdleTaskStackSize);
}

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern OSInterface theOSInterface;
