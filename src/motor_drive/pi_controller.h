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
//  This class defines a Proportional-Integral (PI) control loop.
//
//  This particular implementation is fairly standard and includes an
//  integrator that saturates.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"



class PIController
{
public:

    PIController();
    ~PIController() {};

    //
    //  This method initializes the control loop.  It should be called at least
    //  once before the loop is used.  The controller supports an optional slew
    //  limit on the target command.  If a slew-limit is desired, set the
    //  commandSlewLimit to a value other than noCommandSlewLimit.  Note that
    //  if the slew limit is enabled, it does not apply to the commandTarget
    //  that is provided to initializePIController.  In this case the loop's
    //  initial target will be the provided value -- it will not be slewed in.
    //
    static constexpr float32_t noCommandSlewLimit = 0.0F;

    void initializePIController(
        const float32_t proportionalGain,
        const float32_t integralGain,
        const float32_t commandTarget,
        const float32_t maximumOutput,
        const float32_t maximumIntegrator,
        const float32_t commandSlewLimit = noCommandSlewLimit);

    //
    //  This method re-initializes the controller.  An initial value for the
    //  loop target and the integrator may be provided to minimize startup
    //  transients.  Note that the loops target will be immediately set to
    //  the provided value.  It will not be slewed.
    //
    void reInitializePIController(
        const float32_t initialTarget = 0.0F,
        const float32_t initialIntegrator = 0.0F);

    //
    //  This method performs a control loop update.
    //
    void updatePIController(const float32_t feedbackInput, float32_t &commandOutput);

    //
    //  This method allows the caller to update the loop's target.  If a slew limit
    //  was provided when the controller was initialized, the loop's target
    //  will be slewed to this value as necessary.
    //
    void setPIControllerTarget(const float32_t commandTarget) { target = commandTarget; }

    //
    //  This allows the caller to update the loops gains and settings.
    //
    void updatePIControllerSettings(
        const float32_t proportionalGain,
        const float32_t integralGain,
        const float32_t maximumOutput,
        const float32_t integratorLimit);

    //
    //  These provide loop status etc. to support test and integration.
    //
    bool loopIsSaturated()           const
        { return (loopSaturatedPositive || loopSaturatedNegative); }
    float32_t getLastTarget()        const { return target; }
    float32_t getLastCommandOutput() const { return lastOutput; }
    float32_t getLastFeedbackInput() const { return presentFeedback; }
    float32_t getLoopError()         const { return presentError; }
    float32_t getLoopIntegrator()    const { return static_cast<float32_t>(integratedError); }


private:

    //
    //  These latch the loops settings.
    //
    float32_t PGain;
    float32_t IGain;
    float32_t target;
    float32_t outputLimit;
    float64_t integratorLimit;
    float32_t targetSlewLimit;

    //
    //  These track the loop's state.
    //
    float32_t slewedTarget;
    float32_t presentFeedback;
    float32_t presentError;
    float64_t integratedError;
    float32_t lastOutput;
    bool      loopSaturatedPositive;
    bool      loopSaturatedNegative;

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;

};
