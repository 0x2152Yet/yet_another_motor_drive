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
//  Defines the interface to the processor's Analog-to-Digital (ADC) interface
//
//  The ADC collects the analog input data required to control the motor.  It
//  is configured to collect that data near the center of the PWM cycle to
//  avoid noise issues that can occur when the PWMs change state.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "adc_conversions.h"
#include "global_definitions.h"
#include "processor.h"

class ADCInterface
{
public:
    ADCInterface();
    ~ADCInterface() {};

    //
    //  This prepares the ADC interface for operation.  It sets everything
    //  up but it does not start actual data collection and the
    //  associated interrupts.
    //
    void initializeADCInterface();

    //
    //  This starts ADC data collection.  As a result, ADC data-ready interrupts
    //  will be processed and the motor controller will be invoked.   This would
    //  normally be called once during system start-up.
    //
    void startADCCollections();

    //
    //  This processes a new frame of ADC data.  It is expected that this is
    //  called from an ISR.  In addition to managing the ADC, this invokes the
    //  motor controller.  It is expected that the motor controller will call
    //  commandADCDataCollection once it has retrieved the data it requires.
    //
    void processADCDataReady();

    //
    //  This is the data structure used to provide basic motor control data.
    //
    typedef struct
    {
        rawADCCounts currentA;
        rawADCCounts currentB;
        rawADCCounts currentC;
        rawADCCounts voltageA;
        rawADCCounts voltageB;
        rawADCCounts voltageC;
        rawADCCounts busCurrent;
        rawADCCounts busVoltage;
    } rawMotorData;

    //
    //  These provide the most recently collected ADC data in ADC counts.
    //  Note that some of these items may not exist in certain hardware
    //  configurations.  It is important that these be called soon after
    //  processADCDataReady has been called to start a new frame's processing.
    //  However, these should also be called before commandADCDataCollection is
    //  called to start the next ADC update.  Calling any of these after
    //  commandADCDataCollection could lead to a data collision with the DMA
    //  controllers that move data from the ADC to the local storage buffers.
    //
    void getRawMotorData(rawMotorData &newMotorData);
    void getRawAnalogHallInputs(
        rawADCCounts &hallA,
        rawADCCounts &hallB,
        rawADCCounts &hallC);

    //
    //  This commands the next data collection.
    //
    void commandADCDataCollection();

private:

};

#ifdef __cplusplus
extern "C" {
#endif

    //
    //  This is the ISR for the DMA's new-ADC-data-is-ready interrupt.
    //
    void DMA2_Stream0_IRQHandler(void);

#ifdef __cplusplus
}
#endif

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern ADCInterface theADCInterface;
