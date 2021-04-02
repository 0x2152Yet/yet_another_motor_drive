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
//  This file implements a Phased Lock Loop (PLL) motor shaft angle
//  estimator.
//
//  At present, it is assumed that the PLL is initialized when the system
//  starts up and is then run continually.  It is assumed that when the
//  motor is spinning too slowly for the PLL estimtor's angle to be trustworthy
//  some other angle source is used to commutate the motor and compute its speed.
//
//  The software implemented in this class is based on the algorithm described
//  in the following Microchip application note:
//
//    "DSO1292A: Sensorless Field Oriented Control (FOC) for a Permanent Magnet
//    Synchronous Motor (PMSM) Using a PLL Estimator and Field Weakening (FW)"
//    Copyright 2009 Mihai Cheles, Microchip Technology
//
////////////////////////////////////////////////////////////////////////////////
#include "angle_estimator_coefficients.h"
#include "data_logger.h"
#include "global_definitions.h"
#include "low_pass_filter.h"
#include "math_util.h"
#include "motor_angle_n_speed.h"
#include "pll_angle_est.h"

#include <math.h>


////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
PLLAngleEstimator::PLLAngleEstimator() :
    estimatedAngle(0.0F),
    pllEstimatorStatorResistance(0.0F),
    pllEstimatorInductancePerUpdate(0.0F),
    pllEstimatorBackEMFToSpeedConversion(0.0F),
    pllSpeedToAngleConversion(0.0F),
    rawEstimatedAngle(0.0F),
    pllSpeedEstimate(0.0F),
    filteredBackEMF_D(),
    filteredBackEMF_Q(),
    lastFeedbackCurrentAlpha(0.0F),
    lastFeedbackCurrentBeta(0.0F),
    estimatorTransforms(),
    pllEstimatorGoodnessMetric(0.0F)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This prepares the observer for operation.  It must be called at least
//  once before the angle estimate is updated.
//
////////////////////////////////////////////////////////////////////////////////
void PLLAngleEstimator::initializePLLEstimtor(
    const resistance_Ohms statorResistance,
    const inductance_H statorInductance,
    const voltsPer1000MechanicalRPM motorBackEMFConstant,
    const float32_t motorPolePairs,
    const frequency_Hz sampleFrequency)
{
    using namespace angleEstimatorCoefficients;
    using namespace GlobalConstants;

    const float32_t samplePeriod = 1.0F / sampleFrequency;

    //
    //  The motor's resistance and inductance are used in the estimator.
    //  The inductance term must be adjusted to take the algorithm's update
    //  sample rate into account.
    //
    pllEstimatorStatorResistance    = statorResistance;
    pllEstimatorInductancePerUpdate = statorInductance / samplePeriod;

    //
    //  We initialize the filters used to filter the computed back EMF values.
    //
    filteredBackEMF_D.computeFilterCoefficient(
        pllBackEMFFilterCutoffFrequency,
        sampleFrequency);

    filteredBackEMF_Q.computeFilterCoefficient(
        pllBackEMFFilterCutoffFrequency,
        sampleFrequency);

    filteredBackEMF_D.setFilterState(0.0F);
    filteredBackEMF_Q.setFilterState(0.0F);

    //
    //  The estimator converts back EMF to estimated speed using the
    //  motor's back EMF constant.  We want a speed in electrical RPM, so
    //  we adjust the conversion factor for the number of motor pole-pairs.
    //
    pllEstimatorBackEMFToSpeedConversion =
        (revolutionsPerRPM * motorPolePairs) / motorBackEMFConstant;

    //
    //  Speed is integrated to get angle.  Our speed is in RPM so we need
    //  a conversion factor that yields angular change per update.
    //
    pllSpeedToAngleConversion =
        samplePeriod * (twoPi / secondsPerMinute);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This estimates the motor shaft angle and speed using a Phased Lock Loop
//  algorithm.
//
////////////////////////////////////////////////////////////////////////////////
void PLLAngleEstimator::updatePLLAngleEstimate(
    const float32_t feedbackCurrentAlpha,
    const float32_t feedbackCurrentBeta,
    const amperes phaseVoltageAlpha,
    const amperes phaseVoltageBeta,
    const volts busVoltage)
{

    //
    //  We start by computing the motor's estimated back EMF in a stationary
    //  alpha/beta coordinate frame.  This calculation requires two terms:
    //  one that adjusts the provided current feedback by the motor's stator
    //  resistance and one that adjusts the change in current feedback by the
    //  motor's inductance.  The inductance coefficient has been previously
    //  adjusted to take the elapsed time between the present and the last
    //  current feedback values into account.
    //
    const float32_t alphaCurrentResistanceTerm =
        pllEstimatorStatorResistance * feedbackCurrentAlpha;
    const float32_t betaCurrentResistanceTerm  =
        pllEstimatorStatorResistance * feedbackCurrentBeta;

    const float32_t changeInCurrentAlpha = feedbackCurrentAlpha - lastFeedbackCurrentAlpha;
    const float32_t changeInCurrentBeta  = feedbackCurrentBeta - lastFeedbackCurrentBeta;

    const float32_t changeInAlphaCurrentInductanceTerm =
        pllEstimatorInductancePerUpdate * changeInCurrentAlpha;
    const float32_t changeInBetaCurrentInductanceTerm  =
        pllEstimatorInductancePerUpdate * changeInCurrentBeta;

    //
    //  With our terms ready, we an compute the back EMF alpha and beta values.
    //
    const float32_t backEMFAlpha =
        phaseVoltageAlpha -
        alphaCurrentResistanceTerm -
        changeInAlphaCurrentInductanceTerm;

    const float32_t backEMFBeta =
        phaseVoltageBeta -
        betaCurrentResistanceTerm -
        changeInBetaCurrentInductanceTerm;

    //
    //  We save off the new current feedback for use in the next estimator
    //  update.
    //
    lastFeedbackCurrentAlpha = feedbackCurrentAlpha;
    lastFeedbackCurrentBeta  = feedbackCurrentBeta;

    //
    //  Next, we transform the alpha and beta back EMF values into the rotating
    //  DQ coordiate frame using a Park transform.  There is a very key point here:
    //  the Park transform must use the angle estimate computed by this PLL
    //  estimtor for its rotating frame reference angle.  If we are controlling
    //  the motor with some other angle source (for example, a motor encoder),
    //  we cannot use that angle source here. Using this PLL estimator's angle in
    //  the transform creates the feedback path that allows the estimator to
    //  "lock" its angle to the motor's actual back EMF angle.  If we use some
    //  other "ground truth" angle, we will likely compute the proper motor speed
    //  (see below) but the resulting angle will not be properly aligned.
    //
    //  One further note:  you will note below that we adjust the angle estimate
    //  for any offset.  The resulting adjusted angle MUST NOT be used here.
    //  Doing so corrupts the speed estimate that is computed below.
    //
    float32_t backEMF_D;
    float32_t backEMF_Q;
    estimatorTransforms.provideRotatingFrameAngle(rawEstimatedAngle);
    estimatorTransforms.performParkTransformation(
        backEMFAlpha,
        backEMFBeta,
        backEMF_D,
        backEMF_Q);

    //
    //  We filter our resulting D and Q back EMF values.
    //
    float32_t newFilteredBackEMF_D;
    float32_t newFilteredBackEMF_Q;
    filteredBackEMF_D.updateFilter(backEMF_D, newFilteredBackEMF_D);
    filteredBackEMF_Q.updateFilter(backEMF_Q, newFilteredBackEMF_Q);

    //
    //  We adjust the back EMF Q component for any control error.  In a perfectly
    //  controlled motor, we would expect the D component to be close to zero.
    //  In actuality, we expect some D component error so we adjust the Q
    //  component accordingly.
    //
    float32_t adjustedBackEMF_Q;

    if (newFilteredBackEMF_Q > 0)
    {
        adjustedBackEMF_Q = newFilteredBackEMF_Q - newFilteredBackEMF_D;
    }
    else
    {
        adjustedBackEMF_Q = newFilteredBackEMF_Q + newFilteredBackEMF_D;
    }

    //
    //  We can now use our Q component back EMF to compute an estimated motor
    //  speed using the motor's back EMF constant.  We have previously adjusted
    //  the conversion factor to yield a speed in electrical RPM.  At present
    //  we do not us this speed in our motor controller.  While this speed
    //  should be fine for our controller, this software supports multiple
    //  sources for its controller speed.  By using this classes output angle
    //  to compute a controller speed we ensure that all of the various speed-
    //  sources are treated the same (same filter constants, etc.)
    //
    pllSpeedEstimate = (pllEstimatorBackEMFToSpeedConversion * adjustedBackEMF_Q);

    //
    //  We integrate the speed estimate to create the angle estimate.
    //
    rawEstimatedAngle = motorAngleAndSpeed::offsetShaftAngle(
        rawEstimatedAngle,
        (pllSpeedEstimate * pllSpeedToAngleConversion));

    //
    //  We adjust our angle estimate with any required offset to yield the
    //  an angle that is properly aligned with the motor's back EMF for use
    //  in controlling the motor currents.  Note that WE MUST NOT use this
    //  angle in the previously performed Park transform.  The need for this
    //  offset and its adverse impact on the previous speed calculation is not
    //  fully understood by the author.
    //
    if (pllSpeedEstimate > 0.0F)
    {
        estimatedAngle = motorAngleAndSpeed::offsetShaftAngle(
            rawEstimatedAngle,
            angleEstimatorCoefficients::pllPositiveSpeedShaftAngleOffset);
    }
    else
    {
        estimatedAngle = motorAngleAndSpeed::offsetShaftAngle(
            rawEstimatedAngle,
            angleEstimatorCoefficients::pllNegativeSpeedShaftAngleOffset);
    }

    //
    //  We compute a "goodness" metric for use in determine the angle and speed
    //  data computed by this estimator is adequate for motor control.
    //
    //  @TODO -- WIP.
    //
    float32_t voltageGoodness = fabs(newFilteredBackEMF_Q / busVoltage);
    float32_t ratioGoodness   = 1.0F - fabs(newFilteredBackEMF_D / newFilteredBackEMF_Q);

    if (voltageGoodness > 0.01F)
    {
        pllEstimatorGoodnessMetric = ratioGoodness;
    }
    else
    {
        pllEstimatorGoodnessMetric = 0.0F;
    }
}

