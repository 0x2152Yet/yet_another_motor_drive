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
//  This class provides a simple single-pole low pass filter.  In addition
//  to the filter, a method is provided to compute the filter coefficient.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"

class lowPassFilter
{
public:
    //
    //  A filter coefficient may be optionally provided to the constructor.
    //
    lowPassFilter(const float32_t initFilterCoefficient = 0.0F);
    ~lowPassFilter() {};

    //
    //  This allows the caller to provide a predefined filter coefficient.
    //
    void setFilterCoefficient (const float32_t newFilterCoefficient) { filterCoefficient = newFilterCoefficient; }

    //
    //  This computes the filter coefficient based on the provided parameters.
    //
    void computeFilterCoefficient (
        const frequency_Hz cutoffFrequency,
        const frequency_Hz updateFrequency);

    //
    //  This updates the filter with the provided sample.
    //
    void updateFilter (const float32_t newSample, float32_t &filteredSample);

    //
    //  This gets the present filter state.
    //
    float32_t getFilterState() const { return filteredValue; }

    //
    //  This sets the filter's state to a particular value.  This is useful for
    //  initializing the filter on system state transitions.
    //
    void setFilterState(const float32_t newState) { filteredValue = newState; }

private:
    //
    //  These store the filter state and the filter coefficient.
    //
    float32_t filteredValue;
    float32_t filterCoefficient;

};
