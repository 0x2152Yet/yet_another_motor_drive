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
//  This file defines the layouts of the various messages the software will
//  send to and accept from an external test support tool via its test
//  support UART.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"

//
//  The items in this namespace describe the overall message layout.
//
namespace messageDefinitions
{
    //
    //  Each message sent on the serial port starts with a sync word.
    //
    const bytes    syncWordOffset  = 0U;
    const uint16_t messageSyncWord = 0xEB90U;

    //
    //  After the sync word, there is a message identifier.
    //
    const bytes idOffset = 2U;

    //
    //  There is always a payload length and CRC.  If there is no payload,
    //  these items will be the end of the message.
    //
    const bytes    payloadLengthOffset = 4U;
    const uint16_t CRCOffset           = 6U;

    const bytes PayloadStartOffset   = 8U;

    //
    //  Not all messages have a payload.
    //
    const bytes minimumMessageLength = PayloadStartOffset;

    //
    //  These are the message identifiers.
    //
    const uint16_t periodicStatus_Id = 0x2152U;
}

//
//  This class defines status that is sent to the external tool at a
//  periodic rate.
//
class periodicStatusMessage
{
public:

    periodicStatusMessage() :
        sampleTime(0),
        feedbackCurrent_D(0.0F),
        feedbackCurrent_Q(0.0F),
        feedbackShaftAngle(0.0F),
        feedbackMotorSpeed(0.0F)
    {
    }

    ~periodicStatusMessage() {};

    uint32_t            sampleTime;
    amperes             feedbackCurrent_D;
    amperes             feedbackCurrent_Q;
    electricalAngle_rad feedbackShaftAngle;
    electricalSpeed_RPM feedbackMotorSpeed;
};


