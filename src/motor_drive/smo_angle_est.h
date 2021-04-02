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
//  This file describes the interface to a Sliding Mode Observer (SMO) motor
//  shaft angle estimator.  Some articles also refer to this as a Sliding Mode
//  Controller.
//
//  The SMO is one of several algorithms used in sensor-less motor drives to
//  estimate the motor's shaft angle. The SMO does use measured motor currents
//  and bus voltage.  This is combined with the commanded voltage and a motor
//  model to estimate the motor's back EMF, and from that, the shaft angle.
//
//  The SMO requires a minimum motor speed to work and does not allow for motor
//  torque control at lower speeds.  For this reason, the SMO cannot be used
//  to start a stationary motor.  Some other technique must be used to control
//  the motor a low speed.
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
#pragma once

#include "data_logger.h"
#include "global_definitions.h"
#include "low_pass_filter.h"

class slidingModeObserverAngleEstimator
{
public:

    slidingModeObserverAngleEstimator();
    ~slidingModeObserverAngleEstimator() {};

    //
    //  This prepares the SMO for operation.  It requires several motor specific
    //  parameters.
    //
    void initializeSMO(
        const resistance_Ohms statorResistance,
        const inductance_H statorInductance,
        const frequency_Hz sampleFrequency);

    //
    //  This computes an updated angle estimate.  The feedback current data should
    //  be the same values used in the motor's current control loops.  The phase
    //  voltage data may either be sampled or estimated using prior motor commands.
    //  Note that the voltage alpha and beta values should be signed such that
    //  zero is mid-scale.  The motor speed is used to adjust the angle estimate
    //  for changes that occur as the motor's speed changes.  This speed may
    //  derived by using the angle values previously computed with the SMO.
    //
    void updateSMOAngleEstimate(
        const float32_t feedbackCurrentAlpha,
        const float32_t feedbackCurrentBeta,
        const amperes phaseVoltageAlpha,
        const amperes phaseVoltageBeta,
        const electricalSpeed_RPM motorSpeed);

    electricalAngle_rad getSlidingModeAngleEstimate() const { return estimatedAngle; }



private:

    //
    //  This is the result from the SMO calculation.
    //
    electricalAngle_rad estimatedAngle;

    //
    //  These are the motor specific gains.
    //
    float32_t motorControlGain;
    float32_t plantGain;

    //
    //  These are the intermediate terms in the SMO calculation.
    //
    float32_t alphaAxisSlidingControl;
    float32_t betaAxisSlidingControl;
    float32_t estimatedCurrentAlpha;
    float32_t estimatedCurrentBeta;
    float32_t estimatedBackEMFAlpha;
    float32_t estimatedBackEMFBeta;
    lowPassFilter filteredBackEMFAlpha;
    lowPassFilter filteredBackEMFBeta;
    lowPassFilter filteredBackEMFAlphaSecondPole;
    lowPassFilter filteredBackEMFBetaSecondPole;
    float32_t finalEstimatedBackEMFAlpha;
    float32_t finalEstimatedBackEMFBeta;

    //
    //  The angle estimate has a speed-dependent offset from the motor's
    //  true shaft angle.
    //
    void adjustAngleEstimateForSpeedLag(const electricalSpeed_RPM motorSpeed);

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;

};

