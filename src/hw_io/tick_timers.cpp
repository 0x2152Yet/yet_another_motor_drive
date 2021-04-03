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
//  Provides a number of timer utilities
//
//  Three different time bases are provided:  high, medium, and low time
//  bases.  The high resolution time base is started early in the
//  processor's initialization and may be used for delays, etc. during
//  initialization before the RTOS scheduler is running.  The lower
//  resolution time bases require the RTOS scheduler to operate.  All of
//  these times are relative to when the processor powered on.  Roll overs
//  of the high resolution timer are expected.  Roll overs of the lower
//  resolution timers are possible but would require much longer periods of
//  operation.
//
////////////////////////////////////////////////////////////////////////////////
#include "freeRTOSConfig.h"
#include "global_definitions.h"
#include "hw_definitions.h"
#include "processor.h"
#include "tick_timers.h"

//
//  These track our millisecond and second counters.
//  We increment them based on an RTOS hook.
//
static volatile float32_t partialMillisecondCounter = 0.0F;
static volatile uint32_t  millisecondCounter        = 0U;
static volatile uint32_t  partialSecondsCounter     = 0U;
static volatile uint32_t  secondCounter             = 0U;

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
TickTimers::TickTimers()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This prepares the timers for operation and starts the high resolution timer.
//
////////////////////////////////////////////////////////////////////////////////
void TickTimers::initTimers()
{
    partialMillisecondCounter = 0.0F;
    millisecondCounter        = 0U;
    partialSecondsCounter     = 0U;
    secondCounter             = 0U;

    //
    //  We get the present system clock frequencies.  This will include the
    //  input clock for our timer.  The timer source clock frequency is twice
    //  the input frequency, so we account for that.
    //
    LL_RCC_ClocksTypeDef clockData;
    LL_RCC_GetSystemClocksFreq(&clockData);
    const uint32_t timerInputClockFreq = clockData.PCLK1_Frequency * 2U;

    //
    //  We use the timer input clock to compute the divisor we need to get
    //  the desired tick rate.
    //
    const uint16_t timerPrescaler =
        static_cast<uint16_t>(timerInputClockFreq /
                              static_cast<uint32_t>(TimerDefs::highResTimerFrequency_Hz) - 1U);


    //
    //  Now we initialize the timer.  We set it up as a simple up-counter that
    //  rolls over at full scale.  We have to do this so our delta-time
    //  calculations act as expected.
    //
    LL_TIM_InitTypeDef timerInitData;
    LL_TIM_StructInit(&timerInitData);

    timerInitData.Prescaler         = timerPrescaler;
    timerInitData.CounterMode       = LL_TIM_COUNTERMODE_UP;
    timerInitData.Autoreload        = 0xFFFFFFFFU;
    timerInitData.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    timerInitData.RepetitionCounter = 0U;

    LL_TIM_Init(TimerDefinitions::highResolutionTickTimer, &timerInitData);

    //
    //  We start the high-res timer.  The other timers will start ticking when
    //  the OS starts.
    //
    LL_TIM_EnableCounter(TimerDefinitions::highResolutionTickTimer);
}


////////////////////////////////////////////////////////////////////////////////
//
//  These provide the present timer values.
//
////////////////////////////////////////////////////////////////////////////////
tickTime_us TickTimers::getHighResTime()
{
    return TimerDefinitions::highResolutionTickTimer->CNT;
}

tickTime_ms TickTimers::getMedResTime()
{
    return millisecondCounter;
}

tickTime_s TickTimers::getLowResTime()
{
    return secondCounter;
}

////////////////////////////////////////////////////////////////////////////////
//
//  These provide the elapsed time from the provided time to the present time.
//
////////////////////////////////////////////////////////////////////////////////
tickTime_us TickTimers::getHighResDelta(const tickTime_us fromTime)
{
    return (getHighResTime() - fromTime);
}

tickTime_ms TickTimers::getMedResDelta(const tickTime_ms fromTime)
{
    return (getMedResTime() - fromTime);
}

tickTime_s TickTimers::getLowResDelta(const tickTime_s fromTime)
{
    return (getLowResTime() - fromTime);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This performs a polling delay for at least the provided time (the actual
//  delay will always be somewhat longer than the requested time).  This is
//  a blocking delay so care must be taken where it is used.
//
////////////////////////////////////////////////////////////////////////////////
void TickTimers::performPollingDelay(const tickTime_us timeToDelay)
{
    tickTime_us adjustedTicksToDelay = timeToDelay;

    //
    //  We add one tick to the requested time as the very first tick will
    //  likely be less than one full time quanta.
    //
    if (timeToDelay < 0xFFFFFFFFU)
    {
        adjustedTicksToDelay = adjustedTicksToDelay + 1U;
    }

    const tickTime_us startTime = getHighResTime();
    tickTime_us deltaTime;
    do
    {
        deltaTime = getHighResDelta(startTime);
    } while (deltaTime < adjustedTicksToDelay);
}


extern "C"
{
    ////////////////////////////////////////////////////////////////////////////////
    //
    //  This is a hook for the RTOS system timer tick interrupt.  We use this
    //  to create our lower resolution time bases.
    //
    ////////////////////////////////////////////////////////////////////////////////
    void vApplicationTickHook(void)
    {
        const uint32_t millisecondsPerSecond = 1000U;

        //
        //  This is called by the RTOS sysTick ISR.  We simply increment our
        //  running millisecond counter. If a second has passed, we increment
        //  our second counter as well.  We do not assume that the OS tick
        //  rate is an integral number of milliseconds, but it most likely is.
        //
        const float32_t millisecondsPerOSTick =
            static_cast<float32_t>(configTICK_RATE_HZ) /
            static_cast<float32_t>(millisecondsPerSecond);

        partialMillisecondCounter = partialMillisecondCounter + millisecondsPerOSTick;

        if (partialMillisecondCounter >= 1.0F)
        {
            //
            //  We extract the integral number of milliseconds (WE DO NOT ROUND).
            //
            const uint32_t elapsedMilliseconds =
                static_cast<uint32_t>(partialMillisecondCounter);

            partialMillisecondCounter =
                partialMillisecondCounter - static_cast<float32_t>(elapsedMilliseconds);

            millisecondCounter = millisecondCounter + elapsedMilliseconds;
            partialSecondsCounter = partialSecondsCounter + elapsedMilliseconds;

            if (partialSecondsCounter >= millisecondsPerSecond)
            {
                const uint32_t newSeconds = partialSecondsCounter / millisecondsPerSecond;
                secondCounter = secondCounter + newSeconds;

                const uint32_t newMilliseconds = newSeconds * millisecondsPerSecond;
                if (partialSecondsCounter > newMilliseconds)
                {
                    partialSecondsCounter = partialSecondsCounter - newMilliseconds;
                }
                else
                {
                    partialSecondsCounter = 0U;
                }
            }
        }
    }
}
