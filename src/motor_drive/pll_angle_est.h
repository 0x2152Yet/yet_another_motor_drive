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
//  This file describes the interface to a Phased Locked Loop (PLL) motor
//  shaft angle estimator.
//
//  The PLL estimator is one of several algorithms used in sensor-less motor
//  drives to estimate the motor's shaft angle. This estimator does use measured
//  motor currents and bus voltage.  This is combined with the commanded voltage
//  and a motor model to estimate the motor's back EMF, and from that, the shaft
//  angle.
//
//  The PLL estimator requires a minimum motor speed to work and does not allow
//  for motor torque control at lower speeds.  For this reason, the estimator
//  cannot be used to start a stationary motor.  Some other technique must be
//  used to control the motor a low speed.
//
//  The software implemented in this class is based on the algorithm described
//  in the following Microchip application note:
//
//    "DSO1292A: Sensorless Field Oriented Control (FOC) for a Permanent Magnet
//    Synchronous Motor (PMSM) Using a PLL Estimator and Field Weakening (FW)"
//    Copyright 2009 Mihai Cheles, Microchip Technology
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "data_logger.h"
#include "global_definitions.h"
#include "low_pass_filter.h"
#include "motor_transforms.h"

class PLLAngleEstimator
{
public:

    PLLAngleEstimator();
    ~PLLAngleEstimator() {};

    //
    //  This prepares the estimator for operation.  It requires several motor
    //  specific parameters.
    //
    void initializePLLEstimtor(
        const resistance_Ohms statorResistance,
        const inductance_H statorInductance,
        const voltsPer1000MechanicalRPM motorBackEMFConstant,
        const float32_t motorPolePairs,
        const frequency_Hz sampleFrequency);

    //
    //  This computes an updated angle estimate.  The feedback current data should
    //  be the same values used in the motor's current control loops.  The phase
    //  voltage data may either be sampled or estimated using prior motor commands.
    //  Note that the voltage alpha and beta values should be signed such that
    //  zero is mid-scale.
    //
    void updatePLLAngleEstimate(
        const float32_t feedbackCurrentAlpha,
        const float32_t feedbackCurrentBeta,
        const amperes phaseVoltageAlpha,
        const amperes phaseVoltageBeta,
        const volts busVoltage);

    electricalAngle_rad getPLLAngleEstimate() const { return estimatedAngle; }



private:

    //
    //  This is the result from the angle estimation.
    //
    electricalAngle_rad estimatedAngle;

    //
    //  These are the motor specific parameters.
    //
    float32_t pllEstimatorStatorResistance;
    float32_t pllEstimatorInductancePerUpdate;
    float32_t pllEstimatorBackEMFToSpeedConversion;
    float32_t pllSpeedToAngleConversion;

    //
    //  These are the estimator's intermediate terms.
    //
    electricalAngle_rad rawEstimatedAngle;
    electricalSpeed_RPM pllSpeedEstimate;
    lowPassFilter       filteredBackEMF_D;
    lowPassFilter       filteredBackEMF_Q;
    float32_t           lastFeedbackCurrentAlpha;
    float32_t           lastFeedbackCurrentBeta;

    //
    //  The PLL estimator requires its own set of coordinate transforms.  It
    //  may not necessarily be able to use the same reference angle as that
    //  used to control the motor.
    //
    motorTransforms estimatorTransforms;

    //
    //  This is a WIP -- we try do determine with the angle estimate is
    //  useable.
    //
    float32_t pllEstimatorGoodnessMetric;

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;

};

