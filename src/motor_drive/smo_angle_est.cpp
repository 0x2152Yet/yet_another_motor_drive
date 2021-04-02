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
//  This file implements the Sliding Mode Observer (SMO) motor shaft angle
//  estimator.
//
//  At present, it is assumed that the SMO is initialized when the system
//  starts up and is then run continually.  It is assumed that when the
//  motor is spinning too slowly for the SMO's angle to be trustworthy some
//  other angle source is used to commutate the motor and compute its speed.
//
//  This implementation was based on documentation and example software from both
//  Texas Instruments and Microchip.  The following application notes may be
//  referenced for more detail:
//
//    "AN1078: Sensorless Field Oriented Control of a PMSM"
//    Copyright 2010 Jorge Zambada and Debraj Deb, Microchip Technology Inc.
//
//    "8015.smopos.pdf: Sliding-Mode Rotor Position Observer of PMSM"
//    Digital Control Systems (DCS) Group, Texas Instruments.
//
////////////////////////////////////////////////////////////////////////////////
#include "angle_estimator_coefficients.h"
#include "data_logger.h"
#include "global_definitions.h"
#include "low_pass_filter.h"
#include "math_util.h"
#include "motor_angle_n_speed.h"
#include "smo_angle_est.h"

#include <math.h>


////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
slidingModeObserverAngleEstimator::slidingModeObserverAngleEstimator() :
    estimatedAngle(0.0F),
    motorControlGain(0.0F),
    plantGain(0.0F),
    alphaAxisSlidingControl(0.0F),
    betaAxisSlidingControl(0.0F),
    estimatedCurrentAlpha(0.0F),
    estimatedCurrentBeta(0.0F),
    estimatedBackEMFAlpha(0.0F),
    estimatedBackEMFBeta(0.0F),
    filteredBackEMFAlpha(),
    filteredBackEMFBeta(),
    filteredBackEMFAlphaSecondPole(),
    filteredBackEMFBetaSecondPole(),
    finalEstimatedBackEMFAlpha(0.0F),
    finalEstimatedBackEMFBeta(0.0F)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This prepares the observer for operation.  It must be called at least
//  once before the angle estimate is updated.
//
////////////////////////////////////////////////////////////////////////////////
void slidingModeObserverAngleEstimator::initializeSMO(
    const resistance_Ohms statorResistance,
    const inductance_H statorInductance,
    const frequency_Hz sampleFrequency)
{
    //
    //  We compute the motor specific parameters used by the SMO.
    //
    const time_s samplePeriod = 1.0F / sampleFrequency;
    plantGain = 1.0F - ((samplePeriod * statorResistance) / statorInductance);
    motorControlGain = samplePeriod / statorInductance;

    //
    //  We initialize the filters used in the SMO.
    //
    filteredBackEMFAlpha.computeFilterCoefficient(
        angleEstimatorCoefficients::smoBackEMFFilterCutoffFrequency,
        sampleFrequency);
    filteredBackEMFBeta.computeFilterCoefficient(
        angleEstimatorCoefficients::smoBackEMFFilterCutoffFrequency,
        sampleFrequency);
    filteredBackEMFAlphaSecondPole.computeFilterCoefficient(
        angleEstimatorCoefficients::smoBackEMFFilterCutoffFrequency,
        sampleFrequency);
    filteredBackEMFBetaSecondPole.computeFilterCoefficient(
        angleEstimatorCoefficients::smoBackEMFFilterCutoffFrequency,
        sampleFrequency);

    //
    //  We re-initialize all of the values that carry across updates.
    //
    alphaAxisSlidingControl = 0.0F;
    betaAxisSlidingControl  = 0.0F;
    estimatedBackEMFAlpha   = 0.0F;
    estimatedBackEMFBeta    = 0.0F;

    filteredBackEMFAlpha.setFilterState(0.0F);
    filteredBackEMFBeta.setFilterState(0.0F);
    filteredBackEMFAlphaSecondPole.setFilterState(0.0F);
    filteredBackEMFBetaSecondPole.setFilterState(0.0F);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This performs an update of the sliding mode observer angle estimate.  The
//  SMO is an iterative estimator that must be called repeatedly when the motor
//  is spinning above a minimum speed to compute a usable angle.
//
//  The caller is responsible for determining the source of the inputs.  Values
//  may be sampled or estimated.  The provided speed may be computed using
//  the angle provided by the SMO.  Note that both the currents and the voltages
//  are expected as signed alpha-beta values that are zeroed.  For example, the
//  voltage values should range from (-VBus/2.0) to (VBus/2.0).
//
////////////////////////////////////////////////////////////////////////////////
void slidingModeObserverAngleEstimator::updateSMOAngleEstimate(
    const amperes feedbackCurrentAlpha,
    const amperes feedbackCurrentBeta,
    const volts phaseVoltageAlpha,
    const volts phaseVoltageBeta,
    const electricalSpeed_RPM motorSpeed)
{
        using namespace angleEstimatorCoefficients;

    //
    //  We start by updating the estimated current feedback.
    //
    estimatedCurrentAlpha =
        (plantGain * estimatedCurrentAlpha) +
        (motorControlGain *
            (phaseVoltageAlpha - estimatedBackEMFAlpha - alphaAxisSlidingControl));

    estimatedCurrentBeta =
        (plantGain * estimatedCurrentBeta) +
        (motorControlGain *
            (phaseVoltageBeta - estimatedBackEMFBeta - betaAxisSlidingControl));

    //
    //  Now we compute the error between our actual and estimated current feedback.
    //
    float32_t feedbackCurrentAlphaError = estimatedCurrentAlpha - feedbackCurrentAlpha;
    float32_t feedbackCurrentBetaError  = estimatedCurrentBeta - feedbackCurrentBeta;

    //
    //  Now we compute the sliding mode control terms used in the back EMF
    //  estimation.  The updated sliding mode control terms are based on
    //  the errors just computed.  If the error is to large, we use default
    //  control terms.
    //
    if (fabs(feedbackCurrentAlphaError) < slidingControlCurrentErrorLimit)
    {
        alphaAxisSlidingControl =
            (feedbackCurrentAlphaError * angleEstimatorCoefficients::slidingControlGain) /
            slidingControlCurrentErrorLimit;
    }
    else if (feedbackCurrentAlphaError > 0.0F)
    {
        alphaAxisSlidingControl = angleEstimatorCoefficients::slidingControlGain;
    }
    else
    {
        alphaAxisSlidingControl = -angleEstimatorCoefficients::slidingControlGain;
    }

    if (fabs(feedbackCurrentBetaError) < slidingControlCurrentErrorLimit)
    {
        betaAxisSlidingControl =
            (feedbackCurrentBetaError * angleEstimatorCoefficients::slidingControlGain) /
            slidingControlCurrentErrorLimit;
    }
    else if (feedbackCurrentBetaError > 0.0F)
    {
        betaAxisSlidingControl = angleEstimatorCoefficients::slidingControlGain;
    }
    else
    {
       betaAxisSlidingControl = -angleEstimatorCoefficients::slidingControlGain;
    }

    //
    //  Our estimated back EMF terms are computed by filtering the control
    //  terms just calculated.
    //
    filteredBackEMFAlpha.updateFilter(alphaAxisSlidingControl, estimatedBackEMFAlpha);
    filteredBackEMFBeta.updateFilter(betaAxisSlidingControl, estimatedBackEMFBeta);

    //
    //  We filter the back EMF terms again before using them to compute
    //  the estimated angle.
    //
    filteredBackEMFAlphaSecondPole.updateFilter(
        estimatedBackEMFAlpha,
        finalEstimatedBackEMFAlpha);
    filteredBackEMFBetaSecondPole.updateFilter(
        estimatedBackEMFBeta,
        finalEstimatedBackEMFBeta);

    //
    //  We can now use the estimated back EMF alpha and beta terms to compute
    //  the estimated shaft angle.  The order of the Alpha and Beta in the
    //  arc-tangent was selected so that resulting angle would change with
    //  the expected sign for a given motor direction.
    //
    estimatedAngle = approxAtan2(finalEstimatedBackEMFBeta, finalEstimatedBackEMFAlpha);

    //
    //  The angle just computed will run from -Pi to Pi radians.  We offset
    //  it to get it to the expected 0..2 Pi radians range.
    //
    estimatedAngle = estimatedAngle + GlobalConstants::Pi;

    //
    //  The estimated angle will lag the true angle due to the filtering
    //  in the observer.  The lag will vary based on the speed of the
    //  motor.  We adjust the angle using the speed that we've been
    //  computing.
    //
    adjustAngleEstimateForSpeedLag(motorSpeed);
}


////////////////////////////////////////////////////////////////////////////////
//
//  This method adjusts the sensor-less angle estimate for the speed
//  dependent filter delay.
//
////////////////////////////////////////////////////////////////////////////////
void slidingModeObserverAngleEstimator::adjustAngleEstimateForSpeedLag(
    const electricalSpeed_RPM motorSpeed)
{
    using namespace angleEstimatorCoefficients;

    if (motorSpeed > 0.0F)
    {
        float32_t angleOffset = motorSpeed * posSpeedAngleAdjustmentK1;
        angleOffset = (angleOffset + posSpeedAngleAdjustmentK2) * motorSpeed;
        angleOffset = (angleOffset + posSpeedAngleAdjustmentK3) * motorSpeed;
        angleOffset =  angleOffset + posSpeedAngleAdjustmentK4;

        estimatedAngle =
            motorAngleAndSpeed::offsetShaftAngle(estimatedAngle, angleOffset);
    }
    else
    {
        float32_t angleOffset = motorSpeed * negSpeedAngleAdjustmentK1;
        angleOffset = (angleOffset + negSpeedAngleAdjustmentK2) * motorSpeed;
        angleOffset = (angleOffset + negSpeedAngleAdjustmentK3) * motorSpeed;
        angleOffset =  angleOffset + negSpeedAngleAdjustmentK4;
        estimatedAngle =
            motorAngleAndSpeed::offsetShaftAngle(estimatedAngle, -angleOffset);
    }
}
