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
//  This class manages the motor's shaft angle and speed calculations.  The
//  shaft angle may be determined using an external encoder or it may be
//  estimated using a sensor-less angle estimator algorithm.  The change-in-angle
//  from whichever method is selected is used to compute the motor's speed.
//
//  This class also supports generating a simulated shaft angle and speed for
//  use in performing the open-loop startup required when a sensor-less shaft
//  angle estimator is selected.
//
////////////////////////////////////////////////////////////////////////////////

#include "angle_estimator_coefficients.h"
#include "global_constants.h"
#include "global_definitions.h"
#include "low_pass_filter.h"
#include "loop_coefficients.h"
#include "math_util.h"
#include "motor_angle_n_speed.h"
#include "motor_constants.h"

#include <math.h>
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
motorAngleAndSpeed::motorAngleAndSpeed():
    presentRampAngle(0.0F),
    presentRampSpeed (0.0F),
    presentAnalogHallAngle (0.0F),
    presentEstimatedAngle (0.0F),
    presentSpeed(0U),
    filteredSpeed(),
    analogHallX(0.0F),
    analogHallY(0.0F),
    angleStep (0.0F),
    finalAngleStep(0.0F),
    rampTargetSpeed(0.0F),
    rampAngleStepChangePerUpdate(0.0F),
    rampAngleStepToSpeedRPMGain(0.0F),
    rampComplete(false),
    firstSpeedUpdate(false),
    speedUpdateDownsample(0U),
    speedUpdateCounter(0U),
    deltaAngleToSpeedRPSGain(0.0F),
    lastSpeedAngle(0.0F),
    smoAngleEstimator(),
    pllAngleEstimator(),
    voltageTransforms(),
    estimatedPhaseAVoltage(0.0F),
    estimatedPhaseBVoltage(0.0F),
    estimatedPhaseCVoltage(0.0F),
    estimatedPhaseVoltageAlpha(0.0F),
    estimatedPhaseVoltageBeta(0.0F)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes the calculation of motor speed.
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::initializeSpeedCalculation(
    const frequency_Hz shaftAngleUpdateFreqeuncy,
    const frequency_Hz speedUpdateFreqeuncy)
{
    //
    //  We compute the number of frames we need to downsample the speed
    //  calculation.  We do not use delta time as that can lead to hysteresis
    //  problems if the speed shaftAngleUpdateFreqeuncy is an integral
    //  multiple of the speedUpdateFreqeuncy -- the most likely case.  Note
    //  that we do not round.  If the angle update frequency is not an integral
    //  multiple of the speed update frequency, this will cause us to choose the
    //  next highest update rate.
    //
    speedUpdateDownsample =
        static_cast<uint32_t>(shaftAngleUpdateFreqeuncy / speedUpdateFreqeuncy);

    //
    //  Now we compute the coefficients associated with the speed calculation.
    //
    firstSpeedUpdate   = true;
    speedUpdateCounter = 0;

    //
    //  The conversion from change-in-angle to speed is based on how fast we
    //  do the speed calculation.  We want a speed in RPM.
    //
    const frequency_Hz speedUpdatesPerSecond =
        shaftAngleUpdateFreqeuncy / static_cast<float32_t>(speedUpdateDownsample);
    deltaAngleToSpeedRPSGain = speedUpdatesPerSecond / GlobalConstants::twoPi;

    //
    //  We set the coefficient for the speed filter.
    //
    filteredSpeed.computeFilterCoefficient(
        LoopCoefficients::speedFilterCutoffFrequency,
        speedUpdatesPerSecond);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This updates the motor speed using whichever angle has been selected
//  as the control angle.  This will handle and commanded downsample of
//  the speed udpate.
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::updateMotorSpeed()
{
    using namespace MotorConstants;

    bool updateSpeed;

    speedUpdateCounter++;
    if (speedUpdateCounter >= speedUpdateDownsample)
    {
        updateSpeed = true;
        speedUpdateCounter = 0U;
    }
    else
    {
        updateSpeed = false;
    }

    if (updateSpeed)
    {
        electricalAngle_rad updatedAngle = 0.0F;

        //
        //  We select the angle to use in the calculation.
        //
        if (currentControlAngleSelection == controlWithMeasuredAnalogAngle)
        {
            if (computeAnalogHallBasedShaftAngle)
            {
                updatedAngle = presentAnalogHallAngle;
            }
        }
        else if (currentControlAngleSelection == controlWithEstimatedAngle)
        {
            if (estimateShaftAngle)
            {
                //
                //  Note that for the speed calculation we do not use the
                //  speed adjusted estimated angle.  That would cause a
                //  potential "chicken-and-the-egg" feedback problem.
                //  In any case, either angle should yield the same speed.
                //
                updatedAngle = presentEstimatedAngle;
            }
        }
        else
        {
            // No angle selected.
        }

        if (firstSpeedUpdate)
        {
            //
            //  We set up for the next pass.  We do this to avoid a large
            //  transient on the very first angle calculation.
            //
            firstSpeedUpdate = false;
            lastSpeedAngle = updatedAngle;
            presentSpeed = 0.0F;
            filteredSpeed.setFilterState(0.0F);
        }
        else
        {
            //
            //  We compute the speed by looking at how the angle has changed
            //  since the last update.
            //
            electricalAngle_rad angleChange = updatedAngle - lastSpeedAngle;
            lastSpeedAngle = updatedAngle;

            //
            //  We have to handle the angle rolling over across zero.  Note that
            //  this is not the same as the calculation performed by offsetAngle!
            //
            if (angleChange > GlobalConstants::Pi)
            {
                angleChange = angleChange - GlobalConstants::twoPi;
            }
            else if (angleChange < -GlobalConstants::Pi)
            {
                angleChange = angleChange + GlobalConstants::twoPi;
            }

            //
            //  We compute the new speed and filter it.
            //
            presentSpeed = angleChange * deltaAngleToSpeedRPSGain;
            presentSpeed = presentSpeed * GlobalConstants::secondsPerMinute;

            float32_t tempFilteredSpeed;
            filteredSpeed.updateFilter(presentSpeed, tempFilteredSpeed);
        }
    }

}

////////////////////////////////////////////////////////////////////////////////
//
//  This updates the motor angle and speed when analog hall sensors are
//  used for the shaft encoder.
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::updateAnalogHallEncoderBasedAngle(
        const float32_t hallSensorA,
        const float32_t hallSensorB,
        const float32_t hallSensorC)
{
    //
    //  We transform the three hall sensor inputs into an X/Y representation.
    //
    analogHallX = ((2.0F * hallSensorA) - hallSensorB - hallSensorC) / 4.0F;
    analogHallY = (GlobalConstants::sqrt3Over2 * (hallSensorB - hallSensorC)) / 2.0F;

    //
    //  We now get our shaft angle using an arc tangent.
    //
    presentAnalogHallAngle = approxAtan2(analogHallX, analogHallY);

    //
    //  Now we have to adjust the angle for the angle offset.  This aligns
    //  the angle computed with the hall sensors to the actual magnetics
    //  within the motor.  Note that adding the offset could make the angle
    //  wrap around zero.
    //
    presentAnalogHallAngle = offsetShaftAngle(
        presentAnalogHallAngle,
        MotorConstants::SegwayMotorShaftAngleOffset);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This initializes the sensor-less motor angle and speed
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::computeSensorlessEstimatorCoefficients(
        const resistance_Ohms statorResistance,
        const inductance_H statorInductance,
        const voltsPer1000MechanicalRPM motorBackEMFConstant,
        const float32_t motorPolePairs,
        const frequency_Hz sampleFrequency)
{
    if (MotorConstants::selectedAngleEstimator ==
        MotorConstants::slidingModeObserverAngleEstimator)
    {
        smoAngleEstimator.initializeSMO(
            statorResistance,
            statorInductance,
            sampleFrequency);
    }
    else if (MotorConstants::selectedAngleEstimator ==
             MotorConstants::phaseLockedLoopAngleEstimator)
    {
        pllAngleEstimator.initializePLLEstimtor(
            statorResistance,
            statorInductance,
            motorBackEMFConstant,
            motorPolePairs,
            sampleFrequency);
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This updates the estimated value for the motor shaft angle using a sliding
//  mode observer technique.
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::updateSensorlessAngle(
    const volts busVoltage,
    const float32_t feedbackCurrentAlpha,
    const float32_t feedbackCurrentBeta,
    const dutyCycle_pct phaseACommandedDutyCycle,
    const dutyCycle_pct phaseBCommandedDutyCycle,
    const dutyCycle_pct phaseCCommandedDutyCycle)
{
    //
    //  We start by estimating the motor's phase voltages.
    //
    estimatePhaseVoltages(
        busVoltage,
        phaseACommandedDutyCycle,
        phaseBCommandedDutyCycle,
        phaseCCommandedDutyCycle);

    //
    //  Now we can update the estimated shaft angle.
    //
    if (MotorConstants::selectedAngleEstimator ==
        MotorConstants::slidingModeObserverAngleEstimator)
    {
        smoAngleEstimator.updateSMOAngleEstimate(
            feedbackCurrentAlpha,
            feedbackCurrentBeta,
            estimatedPhaseVoltageAlpha,
            estimatedPhaseVoltageBeta,
            getFilteredSpeed());

        presentEstimatedAngle = smoAngleEstimator.getSlidingModeAngleEstimate();
    }
    else if (MotorConstants::selectedAngleEstimator ==
             MotorConstants::phaseLockedLoopAngleEstimator)
    {
        pllAngleEstimator.updatePLLAngleEstimate(
            feedbackCurrentAlpha,
            feedbackCurrentBeta,
            estimatedPhaseVoltageAlpha,
            estimatedPhaseVoltageBeta,
            busVoltage);

        presentEstimatedAngle = pllAngleEstimator.getPLLAngleEstimate();

    }

}

////////////////////////////////////////////////////////////////////////////////
//
//  Starts the generation of a new speed ramp.
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::startSpeedRamp(
    const electricalAngle_rad initialAngle,
    const electricalSpeed_RPM initialSpeed,
    const electricalSpeed_RPM finalSpeed,
    const acceleration_RotationsPerSec2 rampSpeedAcceleration,
    const frequency_Hz callingFrequency)
{
    if (callingFrequency > 0.0F)
    {
        //
        //  We setup for the new ramp.
        //
        rampComplete      = false;
        presentRampAngle  = initialAngle;
        presentRampSpeed  = initialSpeed;
        rampTargetSpeed   = finalSpeed;

        //
        //  We compute the change-in-angle at the start of the ramp and the
        //  change-in-angle at the end of the ramp.
        //
        const float32_t initialSpeedRPS  = initialSpeed / GlobalConstants::secondsPerMinute;
        const float32_t finalSpeedRPS    = finalSpeed   / GlobalConstants::secondsPerMinute;
        angleStep       = (initialSpeedRPS * GlobalConstants::twoPi) / callingFrequency;
        finalAngleStep  = (finalSpeedRPS   * GlobalConstants::twoPi) / callingFrequency;

        //
        //  We compute how much to "accelerate" the change in angle each update.
        //  @TODO:  This is probably not right.  It is getting reasonable behavior
        //          but the units are probably wrong.
        //
        rampAngleStepChangePerUpdate =
           rampSpeedAcceleration /
           (GlobalConstants::twoPi * callingFrequency * GlobalConstants::secondsPerMinute);

        if (finalSpeed < initialSpeed)
        {
            rampAngleStepChangePerUpdate = -rampAngleStepChangePerUpdate;
        }

        //
        //  We compute how to convert the change in angle to speed.
        //
        rampAngleStepToSpeedRPMGain =
           (callingFrequency / GlobalConstants::twoPi) * GlobalConstants::secondsPerMinute;
    }
    else
    {
        //
        //  We have an invalid ramp update frequency.
        //
        rampComplete                 = true;
        presentRampAngle             = 0.0F;
        presentRampSpeed             = 0.0F;
        rampTargetSpeed              = 0.0F;
        angleStep                    = 0.0F;
        finalAngleStep               = 0.0F;
        rampAngleStepChangePerUpdate = 0.0F;
        rampAngleStepToSpeedRPMGain  = 0.0F;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  Updates the speed ramp.
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::updateSpeedRamp()
{
    //
    //  We update the angle and handle the wrap.
    //
    presentRampAngle = offsetShaftAngle(presentRampAngle, angleStep);

    presentRampSpeed = angleStep * rampAngleStepToSpeedRPMGain;

    //
    //  If we are still ramping, we update the amount we change the returned
    //  angle each call.
    //
    if (!rampComplete)
    {
        angleStep = angleStep + rampAngleStepChangePerUpdate;

        //
        //  We check to see if we are done ramping.  We must be careful of
        //  the direction of the ramp.
        //
        if (rampAngleStepChangePerUpdate > 0.0F)
        {
            if (angleStep > finalAngleStep)
            {
                rampComplete = true;
                angleStep = finalAngleStep;
            }
        }
        else if (rampAngleStepChangePerUpdate < 0.0F)
        {
            if (angleStep < finalAngleStep)
            {
                rampComplete = true;
                angleStep = finalAngleStep;
            }
        }
        else
        {
            //
            //  We handle the case where the ramp's initial speed and final
            //  speed were the same.
            //
            rampComplete = true;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This is a helper method that adds an offset to an angle.  The rollover
//  across zero is handled.
//
////////////////////////////////////////////////////////////////////////////////
electricalAngle_rad motorAngleAndSpeed::offsetShaftAngle(
        const electricalAngle_rad originalAngle,
        const electricalAngle_rad angleOffset)
{
    electricalAngle_rad newAngle = originalAngle + angleOffset;

    if (newAngle > GlobalConstants::twoPi)
    {
        newAngle = newAngle - GlobalConstants::twoPi;
    }
    else if (newAngle < 0.0F)
    {
        newAngle = newAngle + GlobalConstants::twoPi;
    }

    return newAngle;

}


////////////////////////////////////////////////////////////////////////////////
//
//  This estimates the voltage for each motor phase.  The measured bus voltage
//  and the most recent phase duty cycle commands are required for this operation.
//
////////////////////////////////////////////////////////////////////////////////
void motorAngleAndSpeed::estimatePhaseVoltages(
    const volts busVoltage,
    const dutyCycle_pct phaseACommandedDutyCycle,
    const dutyCycle_pct phaseBCommandedDutyCycle,
    const dutyCycle_pct phaseCCommandedDutyCycle)
{
    //
    //  To start, the bus voltage is scaled by each phase's duty cycle
    //  command.  This calculation assumes that the duty cycles are expressed
    //  in a range of -1.0 to 1.0, rather than the 0.0 to 1.0 range of the
    //  passed in values.
    //
    signedDutyCycle adjustedPhaseADutyCycle = (phaseACommandedDutyCycle - 0.5F) * 2.0F;
    signedDutyCycle adjustedPhaseBDutyCycle = (phaseBCommandedDutyCycle - 0.5F) * 2.0F;
    signedDutyCycle adjustedPhaseCDutyCycle = (phaseCCommandedDutyCycle - 0.5F) * 2.0F;

    //
    //  We reduce the duty cycle by the blanking period enforced in the PWM
    //  and driver hardware.  Recall that the blanking period only impacts the
    //  rising edge, so we only have to take it out once per period.
    //
    const float32_t blankingPeriodAdjustment_pct =
        1.0F - (MotorConstants::actualBlankingPeriod / MotorConstants::PWMCarrierPeriod);

    adjustedPhaseADutyCycle = adjustedPhaseADutyCycle * blankingPeriodAdjustment_pct;
    adjustedPhaseBDutyCycle = adjustedPhaseBDutyCycle * blankingPeriodAdjustment_pct;
    adjustedPhaseCDutyCycle = adjustedPhaseCDutyCycle * blankingPeriodAdjustment_pct;

    //
    //  Now we scale the bus voltage.
    //
    const volts scaledPhaseAVoltage = busVoltage * adjustedPhaseADutyCycle;
    const volts scaledPhaseBVoltage = busVoltage * adjustedPhaseBDutyCycle;
    const volts scaledPhaseCVoltage = busVoltage * adjustedPhaseCDutyCycle;

    //
    //  Now we can estimate each phase voltage.  The estimated voltage is
    //  a combination of all three scaled inputs.
    //
    estimatedPhaseAVoltage =
        (scaledPhaseAVoltage * GlobalConstants::twoThirds) -
        (scaledPhaseBVoltage * GlobalConstants::oneThird) -
        (scaledPhaseCVoltage * GlobalConstants::oneThird);

    estimatedPhaseBVoltage =
        (scaledPhaseBVoltage * GlobalConstants::twoThirds) -
        (scaledPhaseAVoltage * GlobalConstants::oneThird) -
        (scaledPhaseCVoltage * GlobalConstants::oneThird);

    estimatedPhaseCVoltage =
        (scaledPhaseCVoltage * GlobalConstants::twoThirds) -
        (scaledPhaseAVoltage * GlobalConstants::oneThird) -
        (scaledPhaseBVoltage * GlobalConstants::oneThird);

    //
    //  The angle estimators require the voltages to be transformed into
    //  an alpha-beta form so we use our reduced Clarke transform.
    //
    voltageTransforms.performReducedClarkeTransformation(
        estimatedPhaseAVoltage,
        estimatedPhaseBVoltage,
        estimatedPhaseVoltageAlpha,
        estimatedPhaseVoltageBeta);
}

