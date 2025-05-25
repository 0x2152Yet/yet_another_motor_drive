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
//  This file implements several transformations that are used in the motor
//  control process.
//
////////////////////////////////////////////////////////////////////////////////
#include "arm_math.h"
#include "global_constants.h"
#include "global_definitions.h"
#include "motor_transforms.h"

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
motorTransforms::motorTransforms() :
    referenceSine(0.0F),
    referenceCosine(0.0F)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Computes the trigonometric values based on a rotating coordinate frame's
//  present angle.
//
////////////////////////////////////////////////////////////////////////////////
void motorTransforms::provideRotatingFrameAngle (
    const electricalAngle_rad rotatingFrameAngle)
{
    //
    //  We make sure the angle falls in the expected range.
    //
    float32_t adjustedAngle;

    if (rotatingFrameAngle < 0.0)
    {
        adjustedAngle = 0.0F;
    }
    else if (rotatingFrameAngle > GlobalConstants::twoPi)
    {
        adjustedAngle = GlobalConstants::twoPi;
    }
    else
    {
        adjustedAngle = rotatingFrameAngle;
    }

    //
    //  We compute the sines and cosines that are required by the
    //  transformations between rotating and stationary frames.
    //
    referenceSine   = arm_sin_f32(adjustedAngle);
    referenceCosine = arm_cos_f32(adjustedAngle);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Computes the reduced Clarke Transformation.  This transforms 3-phase
//  ABC input data into two-phase AlphaBeta data, both in the stationary
//  reference frame.  This particular implementation assumes one is providing
//  A and B phase inputs.  Note that the A, B, and C inputs must sum to zero
//  for the proper operation of this transform.
//
////////////////////////////////////////////////////////////////////////////////
void motorTransforms::performReducedClarkeTransformation (
    const float32_t inputA,
    const float32_t inputB,
    float32_t &resultAlpha,
    float32_t &resultBeta) const
{
    resultAlpha = inputA;
    resultBeta  = (inputA + (2.0F * inputB)) * GlobalConstants::oneOverSqrt3;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Computes the Park Transformation.  The Park transform transforms data from
//  two-phase AlphaBeta form in the stationary frame to two components DQ of
//  a vector in the rotating frame.
//
////////////////////////////////////////////////////////////////////////////////
void motorTransforms::performParkTransformation (
    const float32_t inputAlpha,
    const float32_t inputBeta,
    float32_t &result_D,
    float32_t &result_Q) const
{
    //
    //  This transform requires the values based on the reference angle
    //  provided earlier.
    //
    result_D = (inputAlpha * referenceSine) - (inputBeta * referenceCosine);
    result_Q = (inputAlpha * referenceCosine) + (inputBeta * referenceSine);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Computes the Inverse Park Transformation.  It transforms the DQ
//  vector components in the rotating frame to a two-phase AlphaBeta 
//  representation in the stationary frame.
//
////////////////////////////////////////////////////////////////////////////////
void motorTransforms::performInverseParkTransformation (
    const float32_t inputD,
    const float32_t inputQ,
    float32_t &resultAlpha,
    float32_t &resultBeta) const
{
    resultAlpha = (inputQ * referenceCosine) + (inputD * referenceSine);
    resultBeta  = (inputQ * referenceSine) - (inputD * referenceCosine);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Computes the inverse Clarke Transformation.  This transforms
//  two-phase AlphaBeta form data in the stationary frame back to the
//  three phase ABC form, also in the stationary frame.
//
////////////////////////////////////////////////////////////////////////////////
void motorTransforms::performInverseClarkeTransformation (
    const float32_t inputAlpha,
    const float32_t inputBeta,
    float32_t &resultA,
    float32_t &resultB,
    float32_t &resultC) const
{
    resultA = inputAlpha;
    resultB = (-0.5F * inputAlpha) + (inputBeta * GlobalConstants::sqrt3Over2);
    resultC = (-0.5F * inputAlpha) - (inputBeta * GlobalConstants::sqrt3Over2);
}

