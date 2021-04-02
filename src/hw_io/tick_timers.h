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
//  This class provides a number of time bases and utilities for using them.
//  The highest precision timer is implemented using a processor tick timer and
//  is always available.  The lower precision timers are based off of the
//  RTOS scheduler's timer and are only available when the RTOS is running.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"

namespace TimerDefs
{
    //
    //  These are the various timer frequencies and periods.
    //
    const float32_t highResTimerFrequency_Hz = 1000000.0F;
    const float32_t medResTimerFrequency_Hz  = 1000.0F;
    const float32_t lowResTimerFrequency_Hz  = 1.0F;
    const float32_t highResTimerPeriod_sec   = 1.0F / highResTimerFrequency_Hz;
    const float32_t medResTimerPeriod_sec    = 1.0F / medResTimerFrequency_Hz;
    const float32_t lowResTimerPeriod_sec    = 1.0F / lowResTimerFrequency_Hz;
}

class TickTimers
{
public:
    TickTimers();
    ~TickTimers() {};

    //
    //  This initializes the timers and starts the high precision timer.
    //
    void initTimers();

    //
    //  These get the available timers (high, medium, and low resolution).
    //  The RTOS scheduler must be running for the medium and low resolution
    //  timers to operate.
    //
    tickTime_us getHighResTime();
    tickTime_ms getMedResTime();
    tickTime_s  getLowResTime();

    //
    //  These provide delta-times at the available resolutions.
    //
    tickTime_us getHighResDelta(const tickTime_us fromTime);
    tickTime_ms getMedResDelta(const tickTime_ms fromTime);
    tickTime_s  getLowResDelta(const tickTime_s fromTime);

    //
    //  This performs a polling delay using the high resolution timer.  It does
    //  not yield the processor (nor does it require the RTOS scheduler to be
    //  operating).
    //
    void performPollingDelay(const tickTime_us timeToDelay);
};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern TickTimers theTimers;

extern "C"
{
    //
    //  Some of the time bases are created by hooking the RTOS tick timer callback.
    //
    void vApplicationTickHook(void);
}
