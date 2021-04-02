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
//  This file provides number of general purpose math utilities.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "global_constants.h"
#include "global_definitions.h"

#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//
//  This function provides the sign of the parameter.
//
////////////////////////////////////////////////////////////////////////////////
template <typename T> int32_t sgn(T val)
{
    uint32_t returnSign;

    if (val < T(0))
    {
        returnSign = -1;
    }
    else if (val > T(0))
    {
        returnSign = 1;
    }
    else
    {
        returnSign = 0;
    }

    return returnSign;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This function returns the minimum of the two inputs.
//
////////////////////////////////////////////////////////////////////////////////
template <typename T> T min(T val1, T val2)
{
    T returnMin;

    if (val1 > val2)
    {
        returnMin = val2;
    }
    else
    {
        returnMin = val1;
    }

    return returnMin;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This function returns the maximum of the two inputs.
//
////////////////////////////////////////////////////////////////////////////////
template <typename T> T max(T val1, T val2)
{
    T returnMax;

    if (val1 < val2)
    {
        returnMax = val2;
    }
    else
    {
        returnMax = val1;
    }

    return returnMax;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This function enforces a slew limit on a value.  The slewed value is
//  provided by, and returned by, reference.
//
////////////////////////////////////////////////////////////////////////////////
template <typename T> void slewLimit(T &slewedValue, T finalValue, T slewLimit)
{
    const T deltaValue = finalValue - slewedValue;

    if (deltaValue > slewLimit)
    {
        slewedValue = slewedValue + slewLimit;
    }
    else if (deltaValue < -slewLimit)
    {
        slewedValue = slewedValue - slewLimit;
    }
    else
    {
        slewedValue = finalValue;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This approximates an arc tangent.  It is used by the atan2 approximation,
//  below.  It is a polynomial approximating arc tangent on the range -1,1.
//  It has a maximum error less than 0.005 radians.  See the comments for
//  the approxAtan2, below, for more detail.
//
////////////////////////////////////////////////////////////////////////////////
template <typename T> T approxAtan(const T z)
{
    const T n1 = 0.97239411F;
    const T n2 = -0.19194795F;
    return (n1 + n2 * z * z) * z;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This approximates a 2 parameter arc tangent ("atan2").  In most places in
//  this software we strive to use the normal library calls for such items.
//  However, in the case of the atan2 the library call is very throughput
//  intensive.  Using this approximation cuts the time to compute an angle
//  significantly.
//
//  This algorithm was authored by Nic Taylor and is described in his article,
//  "How to Find a Fast Floating-Point atan2 Approximation".  The article may
//  be found at the following URL:
//      https://www.dsprelated.com/showarticle/1052.php
//
////////////////////////////////////////////////////////////////////////////////
template <typename T> T approxAtan2(const T y, const T x)
{
    T returnAtan2;

    if (x != 0.0F)
    {
        if (fabsf(x) > fabsf(y))
        {
            const T z = y / x;

            if (x > 0.0F)
            {
                //
                //  atan2(y,x) = atan(y/x) if x > 0
                //
                returnAtan2 = approxAtan(z);
            }
            else if (y >= 0.0F)
            {
                //
                //  atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
                //
                returnAtan2 = (approxAtan(z) + GlobalConstants::Pi);
            }
            else
            {
                //
                //  atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
                //
                returnAtan2 =  approxAtan(z) - GlobalConstants::Pi;
            }
        }
        else
        {
            //
            //  Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
            //
            const T z = x / y;

            if (y > 0.0F)
            {
                //
                //  atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
                //
                returnAtan2 = (-approxAtan(z)) + GlobalConstants::PiOver2;
            }
            else
            {
                //
                //  atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
                //
                returnAtan2 =  (-approxAtan(z)) - GlobalConstants::PiOver2;
            }
        }
    }
    else
    {
        if (y > 0.0F)
        {
            //
            //  x = 0, y > 0
            //
            returnAtan2 = GlobalConstants::PiOver2;
        }
        else if (y < 0.0f)
        {
            //
            //  x = 0, y < 0
            //
            returnAtan2 = -GlobalConstants::PiOver2;
        }
        else
        {
            //
            //  This is actually an illegal case.  We'll return zero.
            //
            returnAtan2 = 0.0F;
        }
    }

    return returnAtan2;
}

