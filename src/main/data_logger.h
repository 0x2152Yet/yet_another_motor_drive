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
//  This class logs and outputs data for test support.  At present, two
//  forms of logging are provided:  The first log is a data buffer of N data
//  items are stored at a high rate and then printed to the console once
//  the buffer is full.  The second "log" comprises two DAC outputs that
//  are updated after each motor drive state machine update.
//
//  Note that data is always sent to the DAC every update.
//
//  In addition to the above logs, a number of general status items are printed
//  to the console at a low rate whenever high speed log buffer data is not
//  being printed.
//
//  This data log class is added as a "friend" to each other system class.
//  This allows this class to access private data for logging purposes.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"
#include "motor_constants.h"

class dataLogger
{
public:
    dataLogger();
    ~dataLogger() {};

    //
    //  This prepares the data log for operation.
    //
    void initializeDataLog();

    //
    //  This should be called at the end of each motor state machine update
    //  so that logging data may be processed.
    //
    void updateDataLog();

    //
    //  This is used to start the buffering of log data.  Once the buffer is
    //  full its contents will be printed.  This needs to be called every
    //  time a buffer of data is required.
    //
    void startBufferingData() { saveLogData = true; }

    //
    //  This prints the contents of the data log buffer to the console.  This
    //  should be called periodically by a low priority task.  Note that this
    //  may block the caller for an extended period of time if there is a buffer
    //  of data to print.
    //
    void printDataLogBuffer();

    //
    //  This prints low rate items.  These are printed to the console periodically
    //  whenever there is not something else going out.
    //
    void printLowRateItems();

private:

    //
    //  These are used to track the contents of the data log buffer.
    //
    struct logDataStruct
    {
        tickTime_us time;
        float32_t item1;
        float32_t item2;
        float32_t item3;
        float32_t item4;
        float32_t item5;
        float32_t item6;
        float32_t item7;
        float32_t item8;
    };

    static const uint32_t itemsToBuffer = 100U;
    logDataStruct logData[itemsToBuffer];
    uint32_t dataLogIndex;
    bool     saveLogData;
    bool     dataInLog;

    //
    //  These support downsampling from the maximum update rate.
    //
    static const uint32_t downsampleFrames = 0U;
    uint32_t downsampleCounter;

    //
    //  These define periodic items that are printed at a low
    //  rate whenever there is not something else going to the
    //  console.
    //
    static const uint32_t lowRateItemDownsample = 15000U;
    uint32_t lowRateItemDownsampleCounter;
    uint32_t lowRateItemUpdateCounter;
    uint32_t lastLowRateItemUpdateCounter;
    tickTime_us lowRateUpdateTime;
    float32_t   lowRateItem1;
    float32_t   lowRateItem2;
    float32_t   lowRateItem3;
    float32_t   lowRateItem4;
    float32_t   lowRateItem5;

    //
    //  These are used to display the motor shaft angle on
    //  an LED display.
    //
    void displayMotorAngle();
    electricalAngle_rad angleForDisplay;
    electricalSpeed_RPM speedForDisplay;
    electricalAngle_rad reducedAngle;
    tickTime_ms         lastDisplayUpdateTime;
    uint8_t             lastPositionLED;

};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern dataLogger theDataLogger;
