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
//  Defines the interface to the processor's I2C bus
//
//  At present, this is a very simply interface that only supports writing
//  to devices on the bus.
//
//  This is a polling interface.  Some of the calls may take a while to return.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"
#include "processor.h"

class I2CInterface
{
public:
    I2CInterface();
    ~I2CInterface() {};

    //
    //  This prepares the I2C interface for operation.
    //
    void initializeI2CInterface();

    //
    //  This writes a buffer of bytes to an I2C device.  The device address
    //  should be in an 8 bit form.  In other words, the LSB should not be
    //  part of the device address.
    //
    void writeToI2CDevice(
        const uint8_t  deviceAddress,
        const uint8_t  *const dataToSend,
        const uint32_t numBytesToSend);

private:

    //
    //  These manage the portions of the transfer.
    //
    bool sendAddress(const uint8_t deviceAddress, const bool isWrite);
    bool sendData(const uint8_t *const dataToSend, const uint8_t numBytesToSend);
    bool waitForIdleBus();
    bool waitForStartSent();
    bool waitForByteSent();

};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern I2CInterface theI2CInterface;
