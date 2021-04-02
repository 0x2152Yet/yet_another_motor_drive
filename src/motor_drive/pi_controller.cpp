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
//  This class implements a Proportional-Integral (PI) control loop.
//
////////////////////////////////////////////////////////////////////////////////
#include "global_definitions.h"
#include "math_util.h"
#include "pi_controller.h"

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
PIController::PIController() :
    PGain(0.0F),
    IGain (0.0F),
    target (0.0F),
    outputLimit(0.0F),
    integratorLimit(0.0F),
    targetSlewLimit(noCommandSlewLimit),
    slewedTarget(0.0F),
    presentFeedback(0.0F),
    presentError(0.0F),
    integratedError(0.0L),
    lastOutput(0.0F),
    loopSaturatedPositive(false),
    loopSaturatedNegative(false)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method initializes the setting of the controller.  Other methods
//  may be called to update these settings.
//
////////////////////////////////////////////////////////////////////////////////
void PIController::initializePIController(
    const float32_t proportionalGain,
    const float32_t integralGain,
    const float32_t commandTarget,
    const float32_t maximumOutput,
    const float32_t maximumIntegrator,
    const float32_t commandSlewLimit)
{
    PGain           = proportionalGain;
    IGain           = integralGain;
    outputLimit     = maximumOutput;
    integratorLimit = static_cast<float64_t>(maximumIntegrator);
    targetSlewLimit = commandSlewLimit;

    lastOutput       = 0.0F;
    integratedError  = 0.0F;
    loopSaturatedPositive = false;
    loopSaturatedNegative = false;

    //
    //  We initialize the target and the slewed-target to the provided
    //  command.  We assume that for this initial case, we do not want
    //  the controller to have to slew its way to the target.
    //
    target       = commandTarget;
    slewedTarget = commandTarget;

}

////////////////////////////////////////////////////////////////////////////////
//
//  This method re-initializes the controller.  This may be called in cases
//  where the controller is "started-and-stopped."
//
////////////////////////////////////////////////////////////////////////////////
void PIController::reInitializePIController(
    const float32_t initialTarget,
    const float32_t initialIntegrator)
{
    target                = initialTarget;
    slewedTarget          = initialTarget;
    integratedError       = static_cast<float64_t>(initialIntegrator);
    loopSaturatedPositive = false;
    loopSaturatedNegative = false;
    lastOutput            = 0.0F;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method performs a control loop update.  The latest feedback input is used
//  to compute a new commandOutput.
//
////////////////////////////////////////////////////////////////////////////////
void PIController::updatePIController(
    const float32_t feedbackInput, float32_t &commandOutput)

{
    //
    //  If we've been commanded to apply a target slew-limit, we do so.
    //
    if (targetSlewLimit != noCommandSlewLimit)
    {
        slewLimit(slewedTarget, target, targetSlewLimit);
    }
    else
    {
        slewedTarget = target;
    }

    //
    //  First we compute the proportional error.
    //
    presentFeedback = feedbackInput;
    presentError = slewedTarget - presentFeedback;

    //
    //  We only update the integrator if the loop is not saturated OR if the new
    //  error would move the loop out of saturation.
    //
    bool okToIntegrate = !(loopSaturatedPositive || loopSaturatedNegative);

    if (((loopSaturatedPositive) && (presentError < 0.0F)) ||
        ((loopSaturatedNegative) && (presentError > 0.0F)))
    {
        okToIntegrate = true;
    }

    if (okToIntegrate)
    {
        //
        //  We update the integrator and if necessary, we clamp it.  Note
        //  that the integrator is tracked as a double precition value.  This
        //  allows the integrator to accumulate very small errors.
        //
        integratedError = integratedError + (static_cast<float64_t>(IGain * presentError));

        if (integratedError > integratorLimit)
        {
            integratedError = integratorLimit;
        }
        else if (integratedError < -integratorLimit)
        {
            integratedError = -integratorLimit;
        }
        else
        {
            //  The integrator is in range.  No action needed.
        }
    }

    //
    //  Now we update the loop output.
    //
    commandOutput = (presentError * PGain) + static_cast<float32_t>(integratedError);

    //
    //  We must now check for saturation of our output.  If it is
    //  saturated, we'll limit the output and note the saturation
    //  for the next pass through this loop.
    //
    if (commandOutput > outputLimit)
    {
        commandOutput = outputLimit;
        loopSaturatedPositive = true;
        loopSaturatedNegative = false;
    }
    else if (commandOutput < -outputLimit)
    {
        commandOutput = -outputLimit;
        loopSaturatedPositive = false;
        loopSaturatedNegative = true;
    }
    else
    {
        //
        //  The output is in limits.
        //
        loopSaturatedPositive = false;
        loopSaturatedNegative = false;
    }

    lastOutput = commandOutput;
}
