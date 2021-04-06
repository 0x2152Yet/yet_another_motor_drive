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
    const tickTime_us maxTimeForI2CTransfer = 1000U;

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

    //LL_I2C_Enable(theI2C);
    //LL_I2C_GenerateStopCondition(theI2C);

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

    bool okSoFar;

    //
    //  We make sure we actually have data to send.  If so,
    //  we enable the bus, clear any prior data or status
    //  and make sure its idle.
    //
    if ((dataToSend != nullptr) && (numBytesToSend > 0U))
    {
        LL_I2C_Enable(theI2C);

        (void)LL_I2C_ReceiveData8(theI2C);
        (void)LL_I2C_ReceiveData8(theI2C);
        (void)theI2C->SR1;
        (void)theI2C->SR2;

        okSoFar = waitForIdleBus();
    }
    else
    {
        okSoFar = false;
    }

    if (okSoFar)
    {
        //
        //  We send a start condition.
        //
        LL_I2C_GenerateStartCondition(theI2C);
        okSoFar = waitForStartSent();
    }

    if (okSoFar)
    {
        //
        //  We address the device.  This will also verify
        //  the device's ack.
        //
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
            LL_I2C_TransmitData8(theI2C, dataToSend[theByte]);
            okSoFar = waitForByteSent();

            theByte++;

        } while ((okSoFar) && (theByte < numBytesToSend));
    }

    //
    //  We put out a stop condition on the bus and disable the peripheral
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
    theI2C->DR = addressToSend;

    //
    //  Now we wait for the device to ack its address.  This is a little
    //  different than the ack for a normal transfer.
    //
    bool addressAckReceived = false;
    tickTime_us startTime = theTimers.getHighResTime();
    tickTime_us timeWaiting;
    do
    {
        if (LL_I2C_IsActiveFlag_ADDR(theI2C) != 0U)
        {
            addressAckReceived = true;
        }

        timeWaiting = theTimers.getHighResDelta(startTime);

    } while ((!addressAckReceived) &&
             (timeWaiting < I2CConstants::maxTimeForI2CTransfer));

    //
    //  We have a special case:  if the calling task was swapped out, the above
    //  check could have timed out.  We do one more check just to be sure things
    //  really failed.
    //
    if (!addressAckReceived)
    {
        if (LL_I2C_IsActiveFlag_ADDR(theI2C) != 0U)
        {
            addressAckReceived = true;
        }
    }

    //
    //  We must clear SR2 after sending an address or the I2C peripheral will
    //  hold the clock low.
    //
    (void)theI2C->SR2;

    return addressAckReceived;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Waits for the I2C bus to become idle
//
////////////////////////////////////////////////////////////////////////////////
bool I2CInterface::waitForIdleBus()
{
    bool busIdle = false;
    tickTime_us startTime = theTimers.getHighResTime();
    tickTime_us timeWaiting;
    do
    {
        if (LL_I2C_IsActiveFlag_BUSY(I2CDefinitions::theI2C) == 0U)
        {
            busIdle = true;
        }

        timeWaiting = theTimers.getHighResDelta(startTime);

    } while ((!busIdle) &&
             (timeWaiting < I2CConstants::maxTimeForI2CTransfer));

    //
    //  We have a special case:  if the calling task was swapped out, the above
    //  check could have timed out.  We do one more check just to be sure things
    //  really failed.
    //
    if (!busIdle)
    {
        if (LL_I2C_IsActiveFlag_BUSY(I2CDefinitions::theI2C) == 0U)
        {
            busIdle = true;
        }
    }

    return busIdle;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Waits for the a start condition to complete
//
////////////////////////////////////////////////////////////////////////////////
bool I2CInterface::waitForStartSent()
{
    bool startSent = false;
    tickTime_us startTime = theTimers.getHighResTime();
    tickTime_us timeWaiting;
    do
    {
        if (LL_I2C_IsActiveFlag_SB(I2CDefinitions::theI2C) != 0U)
        {
            startSent = true;
        }

        timeWaiting = theTimers.getHighResDelta(startTime);

    } while ((!startSent) &&
             (timeWaiting < I2CConstants::maxTimeForI2CTransfer));

    if (!startSent)
    {
        if (LL_I2C_IsActiveFlag_SB(I2CDefinitions::theI2C) != 0U)
        {
            startSent = true;
        }
    }

    return startSent;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Waits for a byte write to complete
//
////////////////////////////////////////////////////////////////////////////////
bool I2CInterface::waitForByteSent()
{
    bool byteSent = false;
    tickTime_us startTime = theTimers.getHighResTime();
    tickTime_us timeWaiting;
    do
    {
        if (LL_I2C_IsActiveFlag_BTF(I2CDefinitions::theI2C) != 0U)
        {
            byteSent = true;
        }

        timeWaiting = theTimers.getHighResDelta(startTime);

    } while ((!byteSent) &&
             (timeWaiting < I2CConstants::maxTimeForI2CTransfer));

    if (!byteSent)
    {
        if (LL_I2C_IsActiveFlag_BTF(I2CDefinitions::theI2C) != 0U)
        {
            byteSent = true;
        }
    }

    return byteSent;
}

