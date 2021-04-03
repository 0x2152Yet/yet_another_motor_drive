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
//  This class provides tranforms used to convert the motor data and commands
//  between various reference frames.
//
//  Sensor feedback from the motor and the final voltage commands to the motor
//  come-from/go-to sensors and windings that are fixed relative to the housing
//  of the motor -- they are always in the same place.  However, the motor control
//  loops use feedback that is fixed relative to the spinning portion of the
//  motor (the motor's rotor).  For example, the feedback Phase Current A is
//  measuring the current flowing into the motor's winding A.  The windings do
//  not move.  However, the Q component current feedback that is used to control
//  the Q current feedback loop moves with the motor's rotor.  Ideally, the Q
//  component current is moving 90 degrees ahead of the rotor electrically, 
//  as it spins.
//
//  Items such as the phase current feedback are said to be in the stationary
//  reference frame while items such as the Q component of the current vector 
//  are said to be in a rotating reference frame.
//
//  Items in the stationary frame come-into and go-out-of the processor in a
//  three phase ABC form.  The three phase ABC values are usually sinusoids
//  offset by 120 degrees.  For some of the system's purposes these three
//  phase values are transformed into two phase Alpha Beta values.  The Alpha
//  Beta values are also usually sinusoids, but they are separated by 90
//  degrees.
//
//  Rotating frame values are expressed as the two components of a rotating 
//  vector: Direct (D) and Quadrature (Q).  In terms of motor currents, the D 
//  vector component refers to the current that is aligned with the magnetic 
//  poles of the motor's rotor while the Q vector component refers to the 
//  current that is 90 degrees "ahead" of motor's rotor poles.  The D 
//  component current is not torque producing while the Q component is 
//  (the rotor "chases" the Q component current).  Note that in a well 
//  controlled motor in steady state, the D and Q values will usually appear 
//  to be DC values -- not sinusoids!
//
//  These are the transformations that are used to moved between the stationary
//  and rotating frame and back again:
//
//      Clarke Transform:  Transforms stationary frame 3-phase ABC data separated
//        in space by 120 degrees to AlphaBeta data, two phases separated in
//        space by 90 degrees still in the stationary frame.
//      Reduced Clarke Transform:  Alternative to the Clark Transform when only 
//        two phases of the 3-phase ABC data are available.  In order to use this
//        form of the Clarke transform the three ABC values must sum to zero.
//      Park Transform:  Transforms stationary frame AlphaBeta data (sinusoidal 
//        in space) to DQ vector components in the rotating frame (mostly 
//        constant in steady-state).
//      Inverse Park Transform:  Transforms rotating frame DQ vector command
//        components into two-phase AlphaBeta commands in the stationary frame.
//      Inverse Clarke Transform:  Transforms the stationary frame AlphaBeta
//        commands back to 3 phase ABC data, also in the stationary frame.
//
//  The Park and inverse Park transformations that move between the stationary
//  and rotating frames require a reference angle to perform their actions.
//  This angle is usually the motor's shaft angle but it is important that it
//  is properly aligned with the motor's back EMF.  It is generally not used 
//  directly in these transforms, rather it is represented in the form of the 
//  sine and cosine of the angle because that is what is needed for these 
//  trigonometric transforms.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "data_logger.h"
#include "global_definitions.h"

class motorTransforms
{
public:

    motorTransforms();
    ~motorTransforms() {};

    //
    //  The transforms that bridge the stationary and rotating reference 
    //  frames require the present angle of the rotating frame.  In the
    //  case of the motor controller, this angle is usually the motor
    //  shaft angle that has been properly aligned with the motor's back
    //  EMF (AKA Theta). This should  be called each computational
    //  frame before the other transforms are utilized.  Note that this
    //  will perform some processor intensive calculations, so it will be
    //  somewhat time consuming.
    //
    void provideRotatingFrameAngle (const electricalAngle_rad rotatingFrameAngle);

    //
    //  This computes the Clarke Transform.  This particular implementation is
    //  a reduced form that assumes one is sampling the A and B phase values and
    //  that the A, B, and C phase values sum to zero.
    //
    void performReducedClarkeTransformation (
        const float32_t inputA,
        const float32_t inputB,
        float32_t &resultAlpha,
        float32_t &resultBeta) const;

    //
    //  This computes the Park Transformation.  It transforms data from
    //  two-phase Alpha Beta form in the stationary frame to two components 
    //  DQ of a vector in the rotating frame.
    //
    void performParkTransformation (
        const float32_t inputAlpha,
        const float32_t inputBeta,
        float32_t &resultD,
        float32_t &resultQ) const;

    //
    //  This performs an inverse Park Transformation.  It transforms the DQ
    //  vector components in the rotating frame to a two-phase Alpha Beta
    //  representation in the stationary frame.
    //
    void performInverseParkTransformation (
        const float32_t inputD,
        const float32_t inputQ,
        float32_t &resultAlpha,
        float32_t &resultBeta) const;

    //
    //  This performs an inverse Clarke Transformation.  It transforms stationary
    //  frame Alpha/Beta data to stationary frame ABC data.
    //
    void performInverseClarkeTransformation (
        const float32_t inputAlpha,
        const float32_t inputBeta,
        float32_t &resultA,
        float32_t &resultB,
        float32_t &resultC) const;

private:

    //
    //  These latch intermediate terms used in multiple transforms.
    //
    electricalAngle_rad referenceSine;
    electricalAngle_rad referenceCosine;

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;

};

