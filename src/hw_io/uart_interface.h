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
//  Defines the interface to the processor's test support UART (serial interface)
//
//  The UART is used to send commands to the processor and for it to output
//  status and data.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"
#include "processor.h"

class UARTInterface
{
public:
    UARTInterface();
    ~UARTInterface() {};

    //
    //  This prepares the UART interface for operation.  Once this is called
    //  serial data may be to to/from the UART.
    //
    void initializeUARTInterface();

    //
    //  This sends data out the UART.  This will return before the data is
    //  actually transferred.  The data transfer is managed internally.
    //
    void sendToUART(uint8_t *const dataToSend, uint32_t numberBytesToSend);

private:

    //
    //  This will poll until a previously commanded transfer has
    //  completed.
    //
    void waitForLastTransferToComplete();

};

#ifdef __cplusplus
extern "C" {
#endif

    //
    //  This queues data for output via the UART.  This exists to support
    //  printf character output.
    //
    void sendCharsToUart(char *const dataToSend, uint32_t numberCharsToSend);

#ifdef __cplusplus
}
#endif

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern UARTInterface theUARTInterface;
