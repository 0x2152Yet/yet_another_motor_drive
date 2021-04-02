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
//  Provides an interface to the processor's physical control inputs
//
////////////////////////////////////////////////////////////////////////////////

#include "global_definitions.h"
#include "hw_definitions.h"
#include "gpio_interface.h"
#include "processor.h"
#include "physical_inputs.h"
#include "tick_timers.h"

namespace encoderConsts
{
    //
    //  These are the physical limits of the timer used to decode the speed
    //  command encoder.
    //
    const int32_t minEncoderTimerCounts      = 0;
    const int32_t maxEncoderTimerCounts      = 65535;
    const int32_t midPointEncoderTimerCounts =
        (maxEncoderTimerCounts - minEncoderTimerCounts) / 2;

    //
    //  These are the limits placed on the returned speed command.  These are
    //  somewhat smaller than the value that would fit in the numeric type so
    //  that we can keep it from overflowing.
    //
    const int32_t minEncoderCommandCounts = -0x80000000 + maxEncoderTimerCounts;
    const int32_t maxEncoderCommandCounts =  0x7FFFFFFF - maxEncoderTimerCounts;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
PhysicalInputs::PhysicalInputs() :
    onboardButtonState(),
    externalButtonState(),
    encoderCommandCounts(0),
    lastEncoderCounts(0)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method prepares the various physical inputs for operation
//
////////////////////////////////////////////////////////////////////////////////
void PhysicalInputs::initializePhysicalInputs()
{
    //
    //  We initialize the timer that is used to decode the speed command
    //  input.  It is set up as a basic timer except that it counts based
    //  on an external quadrature encoder input.
    //
    LL_TIM_InitTypeDef timerInitData;
    LL_TIM_StructInit(&timerInitData);

    timerInitData.Prescaler         = 0U;
    timerInitData.CounterMode       = LL_TIM_COUNTERMODE_UP;
    timerInitData.Autoreload        = 0xFFFFFFFFU;
    timerInitData.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    timerInitData.RepetitionCounter = 0U;

    LL_TIM_Init(TimerDefinitions::speedControlEncoderTimer, &timerInitData);

    LL_TIM_ENCODER_InitTypeDef encoderInitData;
    LL_TIM_ENCODER_StructInit(&encoderInitData);

    encoderInitData.EncoderMode    = LL_TIM_ENCODERMODE_X2_TI1;
    encoderInitData.IC1Polarity    = LL_TIM_IC_POLARITY_FALLING;
    encoderInitData.IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
    encoderInitData.IC1Prescaler   = LL_TIM_ICPSC_DIV1;
    encoderInitData.IC1Filter      = LL_TIM_IC_FILTER_FDIV1_N8;
    encoderInitData.IC2Polarity    = LL_TIM_IC_POLARITY_RISING;
    encoderInitData.IC2ActiveInput = LL_TIM_IC_POLARITY_FALLING;
    encoderInitData.IC2Prescaler   = LL_TIM_ICPSC_DIV1;
    encoderInitData.IC2Filter      = LL_TIM_IC_FILTER_FDIV1_N8;

    LL_TIM_ENCODER_Init(TimerDefinitions::speedControlEncoderTimer, &encoderInitData);

    //
    //  Now we "zero" the encoder timer and enable it.
    //
    resetSpeedCommand();
    LL_TIM_EnableCounter(TimerDefinitions::speedControlEncoderTimer);

}

////////////////////////////////////////////////////////////////////////////////
//
//  This method updates the states of the push buttons.
//
////////////////////////////////////////////////////////////////////////////////
void PhysicalInputs::updatePushButtons()
{
    onboardButtonState.updateDebouncer(GPIOInterface::onBoardButtonIsPressed());
    externalButtonState.updateDebouncer(GPIOInterface::externalButtonIsPressed());
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method updates the counter that decodes the speed command encoder so
//  that the next request for the speed command will return zero.
//
////////////////////////////////////////////////////////////////////////////////
void PhysicalInputs::resetSpeedCommand()
{
    //
    //  We reset the counters.
    //
    encoderCommandCounts = 0;
    LL_TIM_SetCounter(TimerDefinitions::speedControlEncoderTimer, 0U);
    lastEncoderCounts =
        static_cast<int32_t>(LL_TIM_GetCounter(TimerDefinitions::speedControlEncoderTimer));
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method provides the present speed command, in signed unit-less ticks.
//  It is up to the caller to convert ticks to a meaningful speed command.
//
////////////////////////////////////////////////////////////////////////////////
int32_t PhysicalInputs::getPresentSpeedCommandCounts()
{
    using namespace encoderConsts;
    //
    //  We get the number of encoder counts since the last time this was
    //  called.
    //
    int32_t encoderCounts =
        static_cast<int32_t>(LL_TIM_GetCounter(TimerDefinitions::speedControlEncoderTimer));

    //
    //  Now we compute the change since the last call.
    //
    int32_t deltaEncoderCounts = encoderCounts - lastEncoderCounts;

    lastEncoderCounts = encoderCounts;

    //
    //  The timer will roll over if the encoder reaches either of its limits.
    //  We detect this by looking for a very large delta.  This assumes that
    //  we are called quickly enough that such a delta cannot be "real".
    //
    if (deltaEncoderCounts > midPointEncoderTimerCounts)
    {
        deltaEncoderCounts = (deltaEncoderCounts - maxEncoderTimerCounts) - 1;
    }
    else if (deltaEncoderCounts < -midPointEncoderTimerCounts)
    {
        deltaEncoderCounts = (deltaEncoderCounts + maxEncoderTimerCounts) + 1;
    }

    //
    //  Now we add the new counts to the accumulator.  We clamp all of the
    //  values.  The clamps on the final value include some room so that it
    //  will not overflow before the clamps are applied.
    //
    if (deltaEncoderCounts > midPointEncoderTimerCounts)
    {
        deltaEncoderCounts = midPointEncoderTimerCounts;
    }
    else if (deltaEncoderCounts < -midPointEncoderTimerCounts)
    {
        deltaEncoderCounts = -midPointEncoderTimerCounts;
    }

    encoderCommandCounts = encoderCommandCounts + deltaEncoderCounts;

    if (encoderCommandCounts > maxEncoderCommandCounts)
    {
        encoderCommandCounts = maxEncoderCommandCounts;
    }
    else if (encoderCommandCounts < minEncoderCommandCounts)
    {
        encoderCommandCounts = minEncoderCommandCounts;
    }

    return encoderCommandCounts;
}
