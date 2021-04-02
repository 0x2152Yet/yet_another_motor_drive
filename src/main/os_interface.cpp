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
//  This class provides a very thin wrapper around the RTOS functions used in
//  this software along with some helper methods required by the RTOS.
//
////////////////////////////////////////////////////////////////////////////////
#include "motor_controller.h"
#include "FreeRTOS.h"
#include "global_constants.h"
#include "global_definitions.h"
#include "os_interface.h"
#include "task.h"

//
//  We provide memory for the RTOS heap.  We do this so that we can decide where
//  in memory it is linked (by default the RTOS sticks it into bss).
//
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((aligned(0x10), section(".rtos_heap")));

//
//  We provide memory for the RTOS idle task.
//
static StaticTask_t idleTaskBuffer;
static StackType_t idleStack[configMINIMAL_STACK_SIZE]
    __attribute__((aligned(configMINIMAL_STACK_SIZE))) = { 0 };


////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
OSInterface::OSInterface()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This creates a new OS thread.  Each thread should call this for itself
//  before the scheduler is started.
//
////////////////////////////////////////////////////////////////////////////////
void OSInterface::createThread(
        TaskFunction_t     threadStaticEntryPointPtr,
        uint32_t           threadPriority,
        uint32_t           threadStackSize_longwords,
        void * const       threadParameter,
        const char * const threadName)
{
    //
    //  We call the RTOS task create function.  For now we don't do anything
    //  with the handle.  We could use it later for task messaging, etc.
    //
    TaskHandle_t theHandle = nullptr;

    const BaseType_t creationResult = xTaskCreate(
        threadStaticEntryPointPtr,
        threadName,
        threadStackSize_longwords,
        threadParameter,
        threadPriority,
        &theHandle);

    if (creationResult != pdPASS)
    {
        while(true)
        {
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This starts the OS scheduler.  It never returns.
//
////////////////////////////////////////////////////////////////////////////////
void OSInterface::startScheduler()
{
    //
    //  We start the kernel.  This should not return.
    //
    vTaskStartScheduler();

    while (true)
    {
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This performs a yielding delay.  The calling thread will block for at least
//  the indicated time.
//
////////////////////////////////////////////////////////////////////////////////
void OSInterface::delay(const time_ms delayTime)
{
    //
    //  We convert the desired delay in milliseconds to OS ticks.  We will
    //  always delay at least 1 tick.
    //
    const float32_t OSTicksPerMillisecond =
        static_cast<float32_t>(configTICK_RATE_HZ) /
        GlobalConstants::millisecondsPerSecond;

    uint32_t ticksToDelay = static_cast<uint32_t>(
        (static_cast<float32_t>(delayTime) * OSTicksPerMillisecond) + 0.5);

    if (ticksToDelay < 1U)
    {
        ticksToDelay = 1U;
    }

    vTaskDelay(ticksToDelay);
}

////////////////////////////////////////////////////////////////////////////////
//
//  The OS calls this if a malloc call fails.
//
////////////////////////////////////////////////////////////////////////////////
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();

    shutOffMotor();

    while (true)
    {
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  The OS calls this if a stack overflow is detected.
//
////////////////////////////////////////////////////////////////////////////////
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    taskDISABLE_INTERRUPTS();

    shutOffMotor();

    while (true)
    {
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  The OS calls this at initialization to get the memory required for the
//  idle task.
//
////////////////////////////////////////////////////////////////////////////////
void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer,
    uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &idleTaskBuffer;
    *ppxIdleTaskStackBuffer = idleStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

