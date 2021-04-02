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
//  Defines the software configuration options.  Note this file does not
//  define motor specific constants (those may be found in motor_constants.h).
//
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "global_definitions.h"
#include "gpio_interface.h"

namespace SystemConfig
{
    //
    //  The software may be configured to "snoop" data provided by another
    //  motor controller.  In this configuration, the software will not output
    //  PWMs or motor enable signals.  While "snooping" another controller, the
    //  software will still sample and process all of it its inputs.  When snooping
    //  another controller, the software will utilize an external synchronization
    //  pulse to control when the ADC's sample their inputs (normally, the
    //  software configures the ADC synchronize its samples with the software's
    //  locally generated PWM signals).
    //
    const bool snoopRemoteMotorController = false;

    //
    //  The timing LED can be configured to blink around different events.
    //  These select which events should blink the LED.  Note that we use
    //  a bitmap so that multiple events may be selected simultaneously.
    //
    const uint32_t timeNoEvent          = 0x00000000U;
    const uint32_t timeADCInterrupt     = 0x00000001U;
    const uint32_t timeMotorControlTask = 0x00000002U;
    const uint32_t timeBackgroundTask   = 0x00000004U;
    const uint32_t timeStateChanges     = 0x00000008U;

    const uint32_t eventsToTimeWithLED = timeADCInterrupt;

    //
    //  These are helper methods for controlling the timing LED.  The
    //  code responsible for a given event should bracket the event
    //  with these calls while passing in the appropriate event identifier.
    //
    inline void turnTimingLEDOnForEvent (const uint32_t desiredEvent)
    {
        if ((desiredEvent & eventsToTimeWithLED) != 0U)
        {
            GPIOInterface::turnTimingLEDOn();
        }
    }

    inline void turnTimingLEDOffForEvent (const uint32_t desiredEvent)
    {
        if ((desiredEvent & eventsToTimeWithLED) != 0U)
        {
            GPIOInterface::turnTimingLEDOff();
        }
    }
}
