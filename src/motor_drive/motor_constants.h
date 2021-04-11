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
//  This file provides constants related to the motor and its
//  control.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "global_constants.h"
#include "global_definitions.h"

namespace MotorConstants
{
    //
    //  The software may be built to support different motors.  There are the
    //  available motors and the presently selected motor.
    //
    enum availableMotors
    {
        SegwayMotor
        //Anaheim_BLY172S_24V_4000_Motor  @TODO
    };

    const availableMotors selectedMotor = SegwayMotor;

    //
    //  We support several types of motor shaft angle encoders.
    //
    const bool computeAnalogHallBasedShaftAngle = true;
    const bool estimateShaftAngle               = true;

    //
    //  These items are used to determine which shaft angle is actually used
    //  to control the motor currents.
    //
    enum currentControlAngleTypes
    {
        controlWithMeasuredAnalogAngle,
        controlWithEstimatedAngle
    };

    const currentControlAngleTypes currentControlAngleSelection =
        controlWithEstimatedAngle;

    //
    //  There are the available algorithms available for estimating the
    //  motor shaft angle when estimateShaftAngle is true.
    //
    enum shaftAngleEstimatorTypes
    {
        slidingModeObserverAngleEstimator,
        phaseLockedLoopAngleEstimator
    };

    const shaftAngleEstimatorTypes selectedAngleEstimator =
        phaseLockedLoopAngleEstimator;

    //
    //  These provide the frequency and period at which the motor
    //  controller is updated.
    //
    const frequency_Hz controllerUpdateFrequency = 15000.0F;
    const time_s       controllerUpdatePeriod    =
        1.0F / controllerUpdateFrequency;

    //
    //  This is the rate at which the motor's speed is updated.
    //  It is best if the controllerUpdateFrequency is an integral
    //  multiple of this frequency.
    //
    const frequency_Hz motorSpeedUpdateFrequency = 3000.0F;

    //
    //  This is the system's nominal bus voltage.
    //
    const volts busVoltage = 24.0F;

    //
    //  This the nominal peak current.  This should most likely not be used
    //  a limit for current commands as it may be quite a bit higher than
    //  the power electronic's rated steady state current.
    //
    const amperes peakCurrent = 6.0F;

    //
    //  This defines the motor back EMF constants.  This defines the relationship
    //  between the motor voltage and its speed.  This is usually found in the
    //  motor's documentation.
    //
    const voltsPer1000MechanicalRPM SegwayMotorBackEMF = 8.7F;

    //
    //  The number of motor pole-pairs defines the relationship between electrical
    //  motor rotations and mechanical rotations.
    //
    const float32_t SegwayMotorPolePairs = 2.0F;

    //
    //  These are the motor's stator resistance and inductance.  These are
    //  also usually found in the motor's documentation.
    //
    const resistance_Ohms SegwayMotorStatorResistance = 0.275F;
    const inductance_H    SegwayMotorStatorInductance = 0.00087F;

    //
    //  This offset is used to align shaft angles computed with the
    //  motor analog hall sensors with the motor's back EMF.
    //
    const electricalAngle_rad SegwayMotorShaftAngleOffset =
        (60.0F * GlobalConstants::degreesToRadians);

    //
    //  These define the PWMs used to control the motor.
    //
    const frequency_Hz PWMCarrierFrequency = controllerUpdateFrequency;
    const time_s       PWMCarrierPeriod = 1.0F / PWMCarrierFrequency;

    //
    //  Our present hardware does not require a blanking period (AKA dead
    //  time) on our output PWMs as it handles this internally.  We do compute
    //  an actual blanking period based on the the bridge driver's output.
    //
    const time_s commandedPWMBlinkingPeriod = 0.0F;
    const time_s bridgeDriverBlankingPeriod = 0.0000000055F;
    const time_s actualBlankingPeriod       =
        commandedPWMBlinkingPeriod + bridgeDriverBlankingPeriod;

    //
    //  We apply limits to the PWM duty cycles.  These limits are coupled
    //  to our ADC's sampling of the motor feedback.  We must make sure that
    //  even with the PWMs operating at their limits, the ADC has time to sample
    //  all of its inputs.  When changing these limits, one must make sure that
    //  a PWM transition does not occur during the ADC sampling.  Right now,
    //  the same limit is applied to both very high duty cycles and very low
    //  duty cycles.
    //
    const dutyCycle_pct dutyCycleCommandLimit = 0.05F;
    const dutyCycle_pct minDutyCycleCommand   = 0.0F + dutyCycleCommandLimit;
    const dutyCycle_pct maxDutyCycleCommand   = 1.0F - dutyCycleCommandLimit;

    //
    //  If current sensor offsets are computed, they are computed during the
    //  PWM warmup time and then applied once the motor is running.
    //
    const bool computeCurrentSensorOffsets               = true;
    const frequency_Hz phaseCurrentOffsetFilterFrequency = 100.0F;

    //
    //  These are constants related to the start up of the motor.
    //  The first is the time we give the bridge driver to come out of
    //  reset.
    //
    const tickTime_ms bridgeDriverWarmupTime = 10U;

    //
    //  This the time we allow the PWMs to run at a fixed nominal duty
    //  cycle.  During this time, no current should flow through the
    //  motor.  This time may be used to compute an offset for the
    //  current sensors.
    //
    const tickTime_ms PWMWarmupTime                = 1000U;
    const tickTime_ms initialOffsetCalSettlingTime = 10U;

    //
    //  If we are using sensor-less control, we must perform an open
    //  loop startup.  The open-loop startup includes a rotor alignment phase
    //  and an open-loop ramp to an initial speed.  The definitions for the
    //  rotor alignment and ramp-to-speed are located in loop_coefficients.h.
    //

    //
    //  This is the time we allot for transitioning to closed loop speed control
    //  once the open-loop ramp to speed has completed.
    //
    const tickTime_ms openToClosedLoopTransitionTime = 1000U;

    //
    //  If a phase current exceeds this threshold, we shut down the motor.
    //
    const amperes phaseOverCurrentThreshold = 2.0F;

    //
    //  These are the setting for the gain tuning state.  In this state,
    //  the current controller command is toggled between two values
    //  at a fixed interval.  There is no speed loop active.
    //
    const bool enableGainTuning = false;
    const amperes IDCommand_1 =  0.0F;
    const amperes IDCommand_2 =  0.0F;
    const amperes IQCommand_1 =  0.5F;
    const amperes IQCommand_2 = -0.5F;
    const tickTime_ms gainTuningToggleTime = 100U;

    //
    //  This supports an auto-shutdown feature.  If this is a non-zero value,
    //  the motor controller will only operate for this many frames before
    //  shutting itself down.  This exists to minimize the likelihood of damaging
    //  the motor electronics when testing new software features.
    //
    const uint32_t dontAutoShutdown   = 0U;
    const uint32_t autoShutdownFrames = dontAutoShutdown;

    //
    //  We use an external encoder and button to command motor speed.
    //  Taps of the external button command the processor to collect log
    //  data.  If the button is held, the speed's sign is toggled (i.e.
    //  the motor will change direction).  If we do change the direction,
    //  we delay a number of frames after logging is enabled so we can capture
    //  the transition in the log.
    //
    const electricalSpeed_RPM maxSpeedCommand           = 6000.0F;
    const electricalSpeed_RPM speedCommandGain          = 10.0F;
    const electricalSpeed_RPM speedCommandDeadband      = 15.0F;
    const bool changeDirectionOnButtonHold              = true;
    const bool latchLogOnDirectionChange                = false;
    const tickTime_ms timeToDelayBeforeSpeedToggle      = 100U;
    const tickTime_ms speedCommandUpdateTime            = 100U;

    //
    //  The external speed command encoder includes a ring of LEDs.  Just for
    //  fun, we light the LED that corresponds to the motor's shaft angle.
    //  We actually compute an angle based on a reduced version of the motor
    //  speed because at most motor speeds, the display update is too slow
    //  to keep up with the motor (it does provide a fine example of aliasing).
    //  Note that if the particular encoder is not used in the system, it is
    //  best if this feature is disabled as the failed attempts to communicate
    //  with the display hardware can be time consuming.
    //
    const bool displayShaftAngleOnLEDDisplay = true;
    const float32_t speedReductionDivider    = 0.03F;
}
