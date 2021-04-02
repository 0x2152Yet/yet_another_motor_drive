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
//  This file provides constants related to sensor-less motor shaft angle
//  estimators.
//
//  A note on how these were determined:  This is a work-in-progress.  At
//  present, both estimators will determine a useable shaft angle across a
//  range of motor speeds.  So far, all of this has been done without any
//  motor load.  It is expected that these values may change based on the
//  motor's actual operating environment.
//
//  All of the tuning was performed while controlling the motor with an external
//  shaft encoder.  The "ground truth" angle from the shaft encoder was
//  compared with the output from the estimators to determine the appropriate
//  coefficients and "adjustment factors" (shaft angle offsets, polynomial
//  correction coefficients, etc.)
//
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "global_definitions.h"
#include "motor_constants.h"

namespace angleEstimatorCoefficients
{
    //
    //  These are constants specific to the sliding mode observer.
    //
    //  These are the observers controller gain and filter coefficient.
    //
    const float32_t slidingControlCurrentErrorLimit    = 0.08F;
    const float32_t slidingControlGain                 = 6.0F;
    const frequency_Hz smoBackEMFFilterCutoffFrequency = 100.0F;

    //
    //  The filtering in observer will cause the estimated angle to
    //  lag the true angle.  This varies with the motor's speed.
    //  We use a polynomial correction to adjust the angle.  The polynomial
    //  coefficients were determined by comparing the motor's true angle to
    //  the estimated angle at a number of different speeds.
    //
    const float32_t posSpeedAngleAdjustmentK1 =  1.455E-11F;
    const float32_t posSpeedAngleAdjustmentK2 = -4.923E-8F;
    const float32_t posSpeedAngleAdjustmentK3 =  0.0002F;
    const float32_t posSpeedAngleAdjustmentK4 =  3.6179F;

    const float32_t negSpeedAngleAdjustmentK1 = -6.51E-12F;
    const float32_t negSpeedAngleAdjustmentK2 =  2.843E-9F;
    const float32_t negSpeedAngleAdjustmentK3 = -0.0001F;
    const float32_t negSpeedAngleAdjustmentK4 = -0.3621F;

    //
    //  These are coefficients associated with the Phase Locked Loop (PLL)
    //  angle estimator.
    //
    //  This the filter constant used to filter the back EMF estimates.
    //
    const frequency_Hz pllBackEMFFilterCutoffFrequency = 1000.0F;

    //
    //  This is the offset used to adjust the PLL estimator's output angle
    //  so that is properly aligned to the motor's back EMF.  This was determined
    //  by comparing the angle computed by the estimator with an angle measured
    //  with a motor's encoder.
    //
    const electricalAngle_rad pllPositiveSpeedShaftAngleOffset = 0.3F;
    const electricalAngle_rad pllNegativeSpeedShaftAngleOffset =
        pllPositiveSpeedShaftAngleOffset * 2.0F;
}

