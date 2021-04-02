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
//  Provides constants common to the entire application
//
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "global_definitions.h"

namespace GlobalConstants
{
    //
    //  These items are general constants.
    //
    const float32_t Pi      = 3.1415926F;
    const float32_t twoPi   = 2.0F * Pi;
    const float32_t PiOver2 = Pi / 2.0F;
    const float32_t radiansToDegrees = (360.0F / twoPi);
    const float32_t degreesToRadians = (twoPi / 360.0F);


    const float32_t sqrtOf3      = 1.732051F;
    const float32_t oneOverSqrt3 = 1.0F / sqrtOf3;
    const float32_t sqrt3Over2   = sqrtOf3 / 2.0F;
    const float32_t sqrt3Over3   = sqrtOf3 / 3.0F;

    const float32_t oneThird  = 1.0F / 3.0F;
    const float32_t twoThirds = 2.0F / 3.0F;

    const time_s    secondsPerMinute      = 60.0F;
    const float32_t millisecondsPerSecond = 1000.0F;

    const float32_t revolutionsPerRPM = 1000.0F;
}

