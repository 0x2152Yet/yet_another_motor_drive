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
//  This file provides the methods for converting values from raw ADC counts
//  to actual engineering units.
//
////////////////////////////////////////////////////////////////////////////////
#include "adc_conversions.h"
#include "global_definitions.h"

namespace ADCConversions
{

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  This converts a phase current value from raw ADC counts to amperes.
    //
    ////////////////////////////////////////////////////////////////////////////////
    amperes convertPhaseCurrent(const rawADCCounts ADCinput)
    {
        //
        //  First, we make sure the input value actually falls within the range
        //  of an expected ADC input.
        //
        int32_t adjustedADCInputs;
        if (ADCinput > rawADCFullScale)
        {
            adjustedADCInputs = static_cast<int32_t>(rawADCFullScale);
        }
        else
        {
            adjustedADCInputs = static_cast<int32_t>(ADCinput);
        }

        //
        //  Phase current is actually signed so we need to offset the input.
        //
        adjustedADCInputs = adjustedADCInputs - makeCurrentSignedOffset;

        //
        //  Now we can convert.
        //
        const float32_t phaseCurrentADCCountsToAmperes = voltsPerCount * phaseCurrentConversion;
        amperes phaseCurrent = static_cast<float32_t>(adjustedADCInputs) * phaseCurrentADCCountsToAmperes;

        //
        //  We are not quite done.  We need to invert the sign of the phase current
        //  due to the hardware current sense implementation.
        //
        phaseCurrent = phaseCurrent * correctCurrentInversionGain;

        return phaseCurrent;
    }

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  This converts a bus current value from raw ADC counts to amperes.
    //
    ////////////////////////////////////////////////////////////////////////////////
    amperes convertBusCurrent(const rawADCCounts ADCinput)
    {
        int32_t adjustedADCInputs;
        if (ADCinput > rawADCFullScale)
        {
            adjustedADCInputs = static_cast<int32_t>(rawADCFullScale);
        }
        else
        {
            adjustedADCInputs = static_cast<int32_t>(ADCinput);
        }

        adjustedADCInputs = adjustedADCInputs - makeCurrentSignedOffset;

        const float32_t busCurrentADCCountsToAmperes = voltsPerCount * busCurrentConversion;
        amperes busCurrent = static_cast<float32_t>(adjustedADCInputs) * busCurrentADCCountsToAmperes;

        busCurrent = busCurrent * correctCurrentInversionGain;

        return busCurrent;
    }

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  This converts a phase voltage value from raw ADC counts to volts.
    //
    ////////////////////////////////////////////////////////////////////////////////
    volts convertPhaseVoltage(const rawADCCounts ADCinput)
    {
        int32_t adjustedADCInputs;
        if (ADCinput > rawADCFullScale)
        {
            adjustedADCInputs = static_cast<int32_t>(rawADCFullScale);
        }
        else
        {
            adjustedADCInputs = static_cast<int32_t>(ADCinput);
        }

        const float32_t phaseVoltageADCCountsToVolts = voltsPerCount * phaseVoltageConversion;
        volts phaseVoltage = static_cast<float32_t>(adjustedADCInputs) * phaseVoltageADCCountsToVolts;

        return phaseVoltage;
    }

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  This converts a bus voltage value from raw ADC counts to volts.
    //
    ////////////////////////////////////////////////////////////////////////////////
    volts convertBusVoltage(const rawADCCounts ADCinput)
    {
        int32_t adjustedADCInputs;
        if (ADCinput > rawADCFullScale)
        {
            adjustedADCInputs = static_cast<int32_t>(rawADCFullScale);
        }
        else
        {
            adjustedADCInputs = static_cast<int32_t>(ADCinput);
        }

        const float32_t busVoltageADCCountsToVolts = voltsPerCount * busVoltageConversion;
        volts busVoltage = static_cast<float32_t>(adjustedADCInputs) * busVoltageADCCountsToVolts;

        return busVoltage;
    }
}

