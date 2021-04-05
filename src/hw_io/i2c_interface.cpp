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
//  Provides an interface to the processor's I2C peripheral
//
//  At present, this is a very simple write-only driver.
//
////////////////////////////////////////////////////////////////////////////////
#include "config.h"
#include "global_definitions.h"
#include "hw_definitions.h"
#include "i2c_interface.h"
#include "processor.h"
#include "tick_timers.h"

#include <string.h>
#include <stdio.h>

//
//  These are physical constants related to the I2C interface.
//
namespace I2CConstants
{
    //
    //  This is the maximum time we will wait for a transfer to complete.
    //
    const tickTime_us maxTimeForI2CTransfer = 5000U;

    //
    //  This bit being cleared in a device address indicates we are writing to
    //  it.
    //
    const uint8_t transferIsWriteBit = 0x01U;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
I2CInterface::I2CInterface()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes the ADC interface.  This configures the ADC and its converter.
//  It does not start the actual sampling of data.
//
////////////////////////////////////////////////////////////////////////////////
void I2CInterface::initializeI2CInterface()
{
    using namespace I2CDefinitions;
    using namespace I2CConstants;

    //
    //  We initialize the I2C peripheral.  We are the bus master.
    //
    LL_I2C_InitTypeDef I2CSettings;
    LL_I2C_StructInit(&I2CSettings);
    I2CSettings.PeripheralMode  = LL_I2C_MODE_I2C;
    I2CSettings.ClockSpeed      = I2CClockSpeed;
    I2CSettings.DutyCycle       = LL_I2C_DUTYCYCLE_2;
    I2CSettings.AnalogFilter    = LL_I2C_ANALOGFILTER_ENABLE;
    I2CSettings.DigitalFilter   = 0U;
    I2CSettings.OwnAddress1     = 0U;
    I2CSettings.TypeAcknowledge = LL_I2C_ACK;
    I2CSettings.OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;

    LL_I2C_Init(theI2C, &I2CSettings);

}

////////////////////////////////////////////////////////////////////////////////
//
//  Writes a buffer of data to an I2C device.  The address should be an 8
//  bit address.  This method will fill in direction bit.
//
////////////////////////////////////////////////////////////////////////////////
void I2CInterface::writeToI2CDevice(
    const uint8_t  deviceAddress,
    const uint8_t  *const dataToSend,
    const uint32_t numBytesToSend)
{
    using namespace I2CDefinitions;

    bool okSoFar = false;

    //
    //  We make sure we actually have data to send.  If so,
    //  we enable the bus and make sure its idle.
    //
    if ((dataToSend != nullptr) && (numBytesToSend > 0U))
    {
        LL_I2C_Enable(theI2C);
        okSoFar = waitForIdleBus();
    }

    if (okSoFar)
    {
        //
        //  We address the device.  This will also verify
        //  the device's ack.
        //
        LL_I2C_GenerateStartCondition(theI2C);
        okSoFar = sendAddress(deviceAddress, true);
    }

    //
    //  Now we loop and send the data.
    //
    if (okSoFar)
    {
        uint32_t theByte = 0U;
        do
        {
            //
            //  Note that we have to make sure we clear any prior ack
            //  failure before we do any transfer that expects an ack.
            //  If we let the wait-for-ack method do this, we would have
            //  a race condition if we were swapped out by the OS between
            //  the write and checking for the ack.
            //
            LL_I2C_ClearFlag_AF(theI2C);
            LL_I2C_TransmitData8(theI2C, dataToSend[theByte]);
            okSoFar = waitForAck();

            theByte++;

        } while ((okSoFar) && (theByte < numBytesToSend));
    }

    //
    //  We put a stop condition on the bus and disable the peripheral
    //  regardless of how the above went.
    //
    LL_I2C_GenerateStopCondition(theI2C);
    LL_I2C_Disable(theI2C);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Addresses a device on the I2C bus.
//
////////////////////////////////////////////////////////////////////////////////
bool I2CInterface::sendAddress(const uint8_t deviceAddress, const bool isWrite)
{
    bool didAddress;
    using namespace I2CDefinitions;

    uint8_t addressToSend = deviceAddress;

    if (isWrite)
    {
        //
        //  We note that we are writing to the device.
        //
        addressToSend = addressToSend & ~(I2CConstants::transferIsWriteBit);
    }

    LL_I2C_ClearFlag_AF(theI2C);
    LL_I2C_TransmitData8(theI2C, addressToSend);
    didAddress = waitForAck();

    return didAddress;

}

////////////////////////////////////////////////////////////////////////////////
//
//  Waits for the bus to be idle.
//
////////////////////////////////////////////////////////////////////////////////
bool I2CInterface::waitForIdleBus()
{
    return false;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Waits for a device to acknowledge a transfer.
//
////////////////////////////////////////////////////////////////////////////////
bool I2CInterface::waitForAck()
{
    return false;
}

