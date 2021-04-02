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
//  Defines the interface to the processor's Digital-to-Analog Converter (DAC)
//
//  The DAC is used to output analog data at a high rate.  It uses may include
//  algorithm development.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"
#include "processor.h"

namespace DACDefinitions
{
    //
    //  These items describe the relationship between DAC counts and voltages at
    //  its outputs.
    //
    const volts DACReferenceVoltage = 3.3F;
    const float32_t DACFullScale_counts = 4096.0F;
    const float32_t DACCountsPerVolt = DACFullScale_counts / DACReferenceVoltage;
    const float32_t DACVoltsPerCount = DACReferenceVoltage / DACFullScale_counts;
}

class DACInterface
{
public:
    DACInterface();
    ~DACInterface() {};

    //
    //  This prepares the DAC for operation.
    //
    void initializeDACInterface();

    //
    //  These set the conversion factors for each DAC channel.  These may be
    //  updated at any time.  These are used to convert the values sent to
    //  the DAC to the appropriate counts range for the DAC.  The minimum
    //  and maximum values should be set to the expected range of the values
    //  provided to sentToDACn.  The DAC output will be scaled such that
    //  DAC1MinValue will produce zero volts at the output and DAC1MaxValue
    //  will produce dacReferenceVoltage volts at the output.
    //
    void setDAC1ConversionFactors(const float32_t DAC1MinValue, const float32_t DAC1MaxValue);
    void setDAC2ConversionFactors(const float32_t DAC2MinValue, const float32_t DAC2MaxValue);

    //
    //  These update the values sent out the DAC.  The appropriate gain and
    //  offset are applied to the provided value to convert the value to
    //  counts for the DAC.
    //
    void sendToDAC1(const float32_t outputValue);
    void sendToDAC2(const float32_t outputValue);

private:

    //
    //  These latch the conversion factors for each DAC channel.
    //
    float32_t dac1Gain;
    float32_t dac1Offset;
    float32_t dac2Gain;
    float32_t dac2Offset;
};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern DACInterface theDACInterface;

