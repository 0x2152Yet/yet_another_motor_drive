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
//  Defines constants and functions for converting data from ADC counts to
//  engineering units.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"

//
//  These are types specific to ADC value conversions.
//
typedef uint32_t  rawADCCounts;
typedef float32_t ADCCounts;
typedef float32_t ADCVolts;
typedef float32_t ADCCountsPerADCVolt;
typedef float32_t ADCVoltsPerADCCount;
typedef float32_t amperesPerADCVolt;
typedef float32_t voltsPerADCVolt;


namespace ADCConversions
{
    //
    //  These items describe the basic relationship between ADC counts and voltages at
    //  its inputs.
    //
    const bits ADCResolution           = 12U;
    const rawADCCounts rawADCFullScale = (1U << ADCResolution) - 1U;
    const ADCCounts ADCFullScale       = static_cast<ADCCounts>(rawADCFullScale);
    const volts ADCReference           = 3.3F;
    const ADCCountsPerADCVolt countsPerVolt = static_cast<float32_t>(ADCFullScale) / ADCReference;
    const ADCVoltsPerADCCount voltsPerCount = ADCReference / static_cast<float32_t>(ADCFullScale);

    //
    //  These define the relationship between volts at the ADC's input and
    //  the equivalent engineering unit.  For the present hardware, the derivation
    //  of these may be found in document "ti_drv8312_evm_scale_factors.pdf", which
    //  is included with this software.  We start with the phase currents.
    //
    const amperesPerADCVolt phaseCurrentConversion = 5.24F;
    const amperesPerADCVolt busCurrentConversion   = 5.22F;
    const voltsPerADCVolt   phaseVoltageConversion = 19.7F;
    const voltsPerADCVolt   busVoltageConversion   = 19.7F;

    //
    //  These define other factors required to convert ADC values to units.  These
    //  are used to account for fixed offsets and gains in the circuits.  These
    //  should not be confused with offsets and gains that may vary from implementation
    //  to implementation.  Those are not handled here.
    //
    const int32_t makeCurrentSignedOffset = static_cast<int32_t>(rawADCFullScale) / 2;
    const amperesPerADCVolt correctCurrentInversionGain = -1.0F;

    //
    //  These methods perform the unit conversions.  All accept raw ADC counts
    //  as their inputs and return the appropriate  engineering unit.
    //
    amperes convertPhaseCurrent(const rawADCCounts ADCinput);
    amperes convertBusCurrent(const rawADCCounts ADCinput);
    volts convertPhaseVoltage(const rawADCCounts ADCinput);
    volts convertBusVoltage(const rawADCCounts ADCinput);
}

