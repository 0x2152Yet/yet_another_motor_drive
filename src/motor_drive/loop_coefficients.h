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
//  This file provides constants related to the motor's control loops.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "global_constants.h"
#include "global_definitions.h"

namespace LoopCoefficients
{
    //
    //  Our default motor target speed is fixed.
    //
    const electricalSpeed_RPM defaultTargetSpeed = 1000.0F;

    //
    //  These are the parameters for the motor speed control loop.
    //
    const float32_t speedPGain            = 0.003F;
    const float32_t speedIGain            = 0.000004F;
    const amperes maxSpeedCurrentCommand  = 1.5F;
    const amperes maxSpeedIntegrator      = 1.0F;

    //
    //  Changes to the speed command are slew-limited.
    //
    const float32_t targetSpeedSlewLimit = 0.5F;

    //
    //  This the filter constant used for motor speed calculations.
    //
    const frequency_Hz speedFilterCutoffFrequency = 5.0F;

    //
    //  These are the speeds uses to determine when the motor controller
    //  switches between open-loop and closed-loop speed control.  These
    //  only apply when an estimated shaft angle is being used for motor
    //  control.  Recall that at low speeds, the shaft angle estimate is
    //  not reliable so open-loop control is required.
    //
    const electricalSpeed_RPM openToClosedLoopSpeedControlThreshold = 600.0F;
    const electricalSpeed_RPM closedToOpenLoopSpeedControlThreshold = 500.0F;

    //
    //  These are the parameters for the sensor-less rotor alignment and
    //  open-loop ramp to speed.  During the open-loop start-up, we command
    //  a fixed current.  Note this command assume a positive motor direction.
    //  It will need to be adjusted if the desired direction is negative.
    //
    const amperes openLoopStartupQComponentCurrent = 1.0F;

    //
    //  We may slew-limit the current command during open-loop control.  This
    //  reduces transients that occur on certain mode changes.
    //
    const amperes OpenLoopIQCommandSlewLimit = 0.0001F;

    //
    //  To align the rotor we command a constant current at a fixed position
    //  for a period of time.  This will align the rotor to the position that
    //  the open-loop ramp-to-speed will start at.
    //
    const tickTime_ms timeForRotorAlignment          = 1000U;
    const electricalAngle_rad rotorAlignmentPosition = 0.0F;

    //
    //  After we align the rotor, we perform an open-loop ramp-up to an initial
    //  speed.  This makes sure that the sensor-less estimator has the motor
    //  data it requires to estimate the motor's shaft angle.
    //
    const acceleration_RotationsPerSec2  openLoopSpeedAcceleration = 2.0F;
    const electricalAngle_rad            openLoopStartingPosition  = 0.0F;
    const electricalSpeed_RPM            openLoopRampTargetSpeed   =
        openToClosedLoopSpeedControlThreshold + 50.0F;

    //
    //  When we transition from an open-loop startup to closed loop motor
    //  control, we slew the shaft angle from the simulated open-loop angle to
    //  the estimated closed-loop angle.  We slew the angle by adding a weighted
    //  offset where the weight is reduced each frame.  These are the parameters
    //  for slewing the offset weight.
    //
    const float32_t initialAngleOffsetWeight           = 1.0F;
    const float32_t angleOffsetWeightReductionPerFrame = 0.001F;

    //
    //  This is used when comparing speeds.
    //
    const electricalSpeed_RPM essentiallyEqualSpeed = 5.0F;

    //
    //  These are the settings for the current controllers.  For now, we will
    //  always control the non-torque producing D component current to a fixed
    //  value.  The PI gains for the current controllers were determined
    //  empirically (i.e. trial and error).
    //
    //  The current loop controllers output what portion of the DC bus should
    //  be utilized to achieved the desired motor current. In other words, the
    //  loops command duty cycles. While the DC bus cannot be commanded to more
    //  than 100% of its capability, the current loops have a maximum allowed
    //  output greater than 100%.  This is called Overmodulation.  Overmodulation
    //  allows for a slightly increased maximum speed and slightly faster
    //  current control.  A fine description of Overmodulation (including the
    //  derivation of the value used as a command limit) may be found at:
    //    https://microchipdeveloper.com/motor:overmodulation
    //
    const amperes targetDComponentCurrent = 0.0F;

    const float32_t currentLoopPGain                = 0.35F;
    const float32_t currentLoopIGain                = 0.00005F;
    const float32_t maxCurrentLoopIntegrator        = 0.6;

    const float32_t OvermodulationCommandLimit = 2.0F / GlobalConstants::sqrtOf3;
    const float32_t maxCurrentLoopOutputDutyCommand = OvermodulationCommandLimit;
}
