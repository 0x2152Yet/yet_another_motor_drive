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
//  Provides an interface to the processor's DAC
//
//  The DAC allows the processor to output analog values at a high rate.
//  This may be used during algorithm development to examine values on an
//  oscilloscope.
//
////////////////////////////////////////////////////////////////////////////////
#include "dac_interface.h"
#include "global_definitions.h"
#include "hw_definitions.h"
#include "processor.h"

namespace DACConsts
{
    //
    //  These are the minimum and maximum count values that can be sent to the
    //  DAC.
    //
    const float32_t minDACOutput = 0.0F;
    const float32_t maxDACOutput = 4095.0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
DACInterface::DACInterface() :
    dac1Gain(1.0F),
    dac1Offset(0.0F),
    dac2Gain(1.0F),
    dac2Offset(0.0F)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes the DAC interface.
//
////////////////////////////////////////////////////////////////////////////////
void DACInterface::initializeDACInterface()
{
    //
    //  We use the DAC in a very simple fashion -- no DMA, automatic triggers,
    //  etc.
    //
    LL_DAC_InitTypeDef dacSettings;
    LL_DAC_StructInit(&dacSettings);
    dacSettings.TriggerSource            = LL_DAC_TRIG_SOFTWARE;
    dacSettings.WaveAutoGeneration       = LL_DAC_WAVE_AUTO_GENERATION_NONE;
    dacSettings.WaveAutoGenerationConfig = LL_DAC_NOISE_LFSR_UNMASK_BIT0;
    dacSettings.OutputBuffer             = LL_DAC_OUTPUT_BUFFER_ENABLE;

    LL_DAC_Init(DACDefinitions::theDAC, LL_DAC_CHANNEL_1, &dacSettings);
    LL_DAC_Init(DACDefinitions::theDAC, LL_DAC_CHANNEL_2, &dacSettings);

    //
    //  This is a touch counter-intuitive:  When the trigger is disabled,
    //  the DAC will update whenever the S/W updates the output value.
    //  This is how we are presently using the DAC (i.e. no trigger).
    //
    LL_DAC_DisableTrigger(DACDefinitions::theDAC, LL_DAC_CHANNEL_1);
    LL_DAC_DisableTrigger(DACDefinitions::theDAC, LL_DAC_CHANNEL_2);

    sendToDAC1(0.0F);
    sendToDAC2(0.0F);

    LL_DAC_Enable(DACDefinitions::theDAC, LL_DAC_CHANNEL_1);
    LL_DAC_Enable(DACDefinitions::theDAC, LL_DAC_CHANNEL_2);
}

////////////////////////////////////////////////////////////////////////////////
//
//  These update the conversion factors used when sending data to the DAC.
//
////////////////////////////////////////////////////////////////////////////////
void DACInterface::setDAC1ConversionFactors(
    const float32_t DAC1MinValue, const float32_t DAC1MaxValue)
{
    //
    //  We compute the gain and offset used to convert the inputs to
    //  DAC counts.  Note that we will apply the offset before we apply
    //  the gain when these values are used.
    //
    dac1Offset = -DAC1MinValue;
    dac1Gain = DACDefinitions::DACFullScale_counts / (DAC1MaxValue - DAC1MinValue);
}

void DACInterface::setDAC2ConversionFactors(
    const float32_t DAC2MinValue, const float32_t DAC2MaxValue)
{
    dac2Offset = -DAC2MinValue;
    dac2Gain = DACDefinitions::DACFullScale_counts / (DAC2MaxValue - DAC2MinValue);
}

////////////////////////////////////////////////////////////////////////////////
//
//  These write values to the DAC.  The DAC is updated as soon as these are
//  called.
//
////////////////////////////////////////////////////////////////////////////////
void DACInterface::sendToDAC1(const float32_t outputValue)
{
    //
    //  We use the gain and offset to convert the value to counts.
    //
    float32_t floatCounts = (outputValue + dac1Offset) * dac1Gain;

    //
    //  We clamp it to the DAC's limits before we send it out.
    //
    uint32_t outputCounts;
    if (floatCounts < DACConsts::minDACOutput)
    {
        outputCounts = static_cast<uint32_t>(DACConsts::minDACOutput);
    }
    else if (floatCounts > DACConsts::maxDACOutput)
    {
        outputCounts = static_cast<uint32_t>(DACConsts::maxDACOutput);
    }
    else
    {
        outputCounts = static_cast<uint32_t>(floatCounts);
    }

    //
    //  Now we send the data to the DAC.
    //
    LL_DAC_ConvertData12RightAligned(DACDefinitions::theDAC, LL_DAC_CHANNEL_1, outputCounts);
}

void DACInterface::sendToDAC2(const float32_t outputValue)
{
    float32_t floatCounts = (outputValue + dac2Offset) * dac2Gain;

    uint32_t outputCounts;
    if (floatCounts < DACConsts::minDACOutput)
    {
        outputCounts = static_cast<uint32_t>(DACConsts::minDACOutput);
    }
    else if (floatCounts > DACConsts::maxDACOutput)
    {
        outputCounts = static_cast<uint32_t>(DACConsts::maxDACOutput);
    }
    else
    {
        outputCounts = static_cast<uint32_t>(floatCounts);
    }

    LL_DAC_ConvertData12RightAligned(DACDefinitions::theDAC, LL_DAC_CHANNEL_2, outputCounts);
}
