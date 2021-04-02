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
//  This class implements a simple single-pole low pass filter.
//
////////////////////////////////////////////////////////////////////////////////

#include "global_constants.h"
#include "global_definitions.h"
#include "low_pass_filter.h"

#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
lowPassFilter::lowPassFilter(const float32_t initFilterCoefficient) :
    filteredValue(0.0F),
    filterCoefficient (initFilterCoefficient)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This computes the filter coefficient based on the passed in cutoff
//  frequency and filter update frequency.
//
////////////////////////////////////////////////////////////////////////////////
void lowPassFilter::computeFilterCoefficient(
        const frequency_Hz cutoffFrequency,
        const frequency_Hz updateFrequency)
{
    //
    //  We compute our filter coefficient.
    //
    filterCoefficient =
        expf((-1.0F * (cutoffFrequency * GlobalConstants::twoPi)) * (1.0F / updateFrequency));
}

////////////////////////////////////////////////////////////////////////////////
//
//  This updates the filter.
//
////////////////////////////////////////////////////////////////////////////////
void lowPassFilter::updateFilter(const float32_t newSample, float32_t &filteredSample)
{
    //
    //  This an optimized version of a 1-alpha filter.  With some algebra,
    //  one can hoist out the filter coefficient.
    //
    filteredValue = newSample + (filterCoefficient * (filteredValue - newSample));
    filteredSample = filteredValue;
}
