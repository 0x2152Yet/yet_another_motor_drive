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
//  This class implements the midpoint clamp type zero sequence modulation 
//  function.  Zero sequence modulation adjusts duty cycle commands so that 
//  they can utilize the full capability of the DC bus voltage.
//
////////////////////////////////////////////////////////////////////////////////
#include "global_constants.h"
#include "global_definitions.h"
#include "math_util.h"
#include "motor_constants.h"
#include "zero_sequence_modulator.h"

///////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
zeroSequenceModulator::zeroSequenceModulator()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Performs the zero sequence modulation.
//
//  This implements a midpoint clamp zero sequence modulation.  This is a
//  straightforward way to implement a Conventional Space Vector PWM technique.
//  This particular algorithm is a good general purpose choice because it is 
//  fairly easy to compute.  Other types my be more desrable to reduce 
//  switching losses, for example, in higher voltage systems.
//
////////////////////////////////////////////////////////////////////////////////
void zeroSequenceModulator::performZeroSequenceModulation (
    const signedDutyCycle inputA,
    const signedDutyCycle inputB,
    const signedDutyCycle inputC,
    dutyCycle_pct &resultDutyCycleA,
    dutyCycle_pct &resultDutyCycleB,
    dutyCycle_pct &resultDutyCycleC)
{
    signedDutyCycle minDuty;
    signedDutyCycle maxDuty;

    //
    //  The input duty cycles range from nominally from -1.0 to 1.0 (that is
    //  the output range of the inverse Clarke transformation that produced
    //  them).  We need them to range from 0.0 to 1.0 so we re-scale them.
    //
    float32_t scaledInputA;
    float32_t scaledInputB;
    float32_t scaledInputC;
    scaleDutyCycles(
        inputA,
        inputB,
        inputC,
        scaledInputA,
        scaledInputB,
        scaledInputC);

    //
    //  We compute a normalization offset for the duty
    //  cycles.  This normalization groups them around their
    //  approximate midpoint between the enforced duty cycle limits.
    //
    minDuty = min(scaledInputA, scaledInputB);
    minDuty = min(minDuty, scaledInputC);

    maxDuty = max(scaledInputA, scaledInputB);
    maxDuty = max(maxDuty, scaledInputC);

    float32_t dutyNormalization = (minDuty + maxDuty) / 2.0F;

    //
    //  We adjust the normalization offset for the limits on the duty cycle.
    //  If the duty cycle limits are symmetrical around the duty cycle midpoint,
    //  this will offset the final duty cycles around that midpoint.  If the
    //  duty cycle limits are not symmetrical (for example 0.1 to 0.8) this 
    //  will offset the final duty cycles around the midpoint of the final
    //  allowed range (0.45 in this example).
    //
    const float32_t dutyCycleLimitAverage =
        (MotorConstants::minDutyCycleCommand +
         MotorConstants::maxDutyCycleCommand) / 2.0F;

    dutyNormalization = dutyNormalization - dutyCycleLimitAverage;

    //
    //  Now we apply the normalization offset to each duty cycle.
    //
    resultDutyCycleA = scaledInputA - dutyNormalization;
    resultDutyCycleB = scaledInputB - dutyNormalization;
    resultDutyCycleC = scaledInputC - dutyNormalization;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This function re-scales the duty cycles that are computed by the
//  inverse Clarke Transform to fit the input range needed by the zero 
//  sequence modulation functions.
//
////////////////////////////////////////////////////////////////////////////////
void zeroSequenceModulator::scaleDutyCycles (
    const signedDutyCycle inputA,
    const signedDutyCycle inputB,
    const signedDutyCycle inputC,
    signedDutyCycle &resultA,
    signedDutyCycle &resultB,
    signedDutyCycle &resultC)
{
    resultA = (inputA + 1.0F) / 2.0F;
    resultB = (inputB + 1.0F) / 2.0F;
    resultC = (inputC + 1.0F) / 2.0F;
}
