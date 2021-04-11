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
//  This class manages data logging for test support purposes.
//
//  When commanded, high speed data is saved to a buffer.  Once the buffer is
//  full, its contents are printed to the console.
//
//  In addition, data is sent to the two test support Digital-to-Analog
//  Converters (DACs) each update.
//
//  At present, the software must be edited and re-built to change the items
//  logged.
//
//  Any class with data to log must include this class as a "friend class" in
//  its definition.
//
////////////////////////////////////////////////////////////////////////////////
#include "adc_interface.h"
#include "dac_interface.h"
#include "data_logger.h"
#include "global_constants.h"
#include "global_definitions.h"
#include "i2c_interface.h"
#include "motor_angle_n_speed.h"
#include "motor_controller.h"
#include "motor_state_machine.h"
#include "os_interface.h"
#include "pwm_interface.h"
#include "task.h"
#include "tick_timers.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

//
//  This namespace describes the interface to the LED position display.
//
namespace positionDisplayConsts
{
    //
    //  This is the address of the LED controller on the I2C bus.
    //
    const uint8_t LEDControllerAddress = 0x2AU;

    //
    //  Each LED in the ring is controlled by setting a brightness value in
    //  a register.
    //
    const uint8_t numberOfLEDs                 = 24U;
    const uint8_t maxLEDOffset                 = numberOfLEDs - 1U;
    const uint8_t LED0BrightnessControlAddress = 0x0AU;
    const uint8_t LED0CurrentControlAddress    = 0x22U;
    const uint8_t LEDOffDutyCycle              = 0U;
    const uint8_t DefaultDutyCycle             = 255U;
    const uint8_t DefaultCurrentGain           = 128U;

    const uint8_t bytesInLEDCommand = 2U;
    const uint8_t registerIndex     = 0U;
    const uint8_t valueIndex        = 1U;

    //
    //  At startup, we delay a while before we start updating the device.
    //
    const tickTime_s startupDelay = 5U;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
dataLogger::dataLogger() :
    dataLogIndex(0U),
    saveLogData(),
    dataInLog(false),
    downsampleCounter(0U),
    lowRateItemDownsampleCounter(0U),
    lowRateItemUpdateCounter(0U),
    lastLowRateItemUpdateCounter(0U),
    lowRateUpdateTime(0U),
    lowRateItem1(0.0F),
    lowRateItem2(0.0F),
    lowRateItem3(0.0F),
    lowRateItem4(0.0F),
    lowRateItem5(0.0F),
    angleForDisplay(0.0F),
    speedForDisplay(0.0F),
    reducedAngle(0.0F),
    lastDisplayUpdateTime(0U),
    lastPositionLED(positionDisplayConsts::LED0BrightnessControlAddress)
{
    memset(logData, 0U, sizeof(logData));
}

////////////////////////////////////////////////////////////////////////////////
//
//  This prepares the data log for update.  At present this includes setting
//  the DAC output scale factors.
//
////////////////////////////////////////////////////////////////////////////////
void dataLogger::initializeDataLog()
{
    using namespace positionDisplayConsts;
    //
    //  We prepare the DAC hardware.
    //
    theDACInterface.initializeDACInterface();

    //
    //  We prepare the I2C interface for the LED position display.
    //
    theI2CInterface.initializeI2CInterface();


    //
    //  We configure the DAC for the data we will be viewing.  This must be
    //  updated to match the expected range of the values that are sent to the
    //  DAC.
    //
    //theDACInterface.setDAC1ConversionFactors(0.0, 4095.0F); // Raw value
    //theDACInterface.setDAC1ConversionFactors(-1.0F, 1.0F);
    //theDACInterface.setDAC2ConversionFactors(-1.0F, 1.0F);
    theDACInterface.setDAC1ConversionFactors(0.0, 2500.0);  // Speed
    theDACInterface.setDAC2ConversionFactors(0.0, 2500.0);  // Speed
    //theDACInterface.setDAC1ConversionFactors(0.0, GlobalConstants::twoPi);  // Shaft angle
    //theDACInterface.setDAC2ConversionFactors(0.0, GlobalConstants::twoPi);  // Shaft angle

    //
    //  We set the default current gain for each position LED.
    //
    uint8_t LEDCurrentCommand[bytesInLEDCommand] =
        {LED0CurrentControlAddress, DefaultCurrentGain};

    for (uint8_t theLED = 0; theLED < numberOfLEDs; theLED++)
    {
        theI2CInterface.writeToI2CDevice(
            LEDControllerAddress, LEDCurrentCommand, bytesInLEDCommand);

        (LEDCurrentCommand[registerIndex])++;
    }

}

////////////////////////////////////////////////////////////////////////////////
//
//  This updates the logging of data.  This includes sending data to the DACs.
//  This should be called at the end of each motor drive update.
//
////////////////////////////////////////////////////////////////////////////////
void dataLogger::updateDataLog()
{
    //
    //  We save data to the buffer if we've been commanded to do so.
    //
    if (saveLogData)
    {
        if (dataLogIndex < itemsToBuffer)
        {
            if (downsampleCounter <= 1U)
            {
                downsampleCounter = downsampleFrames;

                logData[dataLogIndex].time = theTimers.getHighResTime();
                logData[dataLogIndex].item1 =
                    theMotorController.theMotorStateMachine.feedbackCurrent_Alpha;
                logData[dataLogIndex].item2 =
                    theMotorController.theMotorStateMachine.feedbackCurrent_Beta;
                logData[dataLogIndex].item3 =
                    theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.estimatedPhaseVoltageAlpha;
                logData[dataLogIndex].item4 =
                    theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.estimatedPhaseVoltageBeta;
                logData[dataLogIndex].item5 =
                    theMotorController.theMotorStateMachine.feedbackCurrent_Alpha;
                logData[dataLogIndex].item6 =
                    theMotorController.theMotorStateMachine.busVoltage;
                logData[dataLogIndex].item7 =
                    theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentAnalogHallAngle;
                logData[dataLogIndex].item8 =
                    theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentEstimatedAngle;

                dataLogIndex++;
            }
            else
            {
                downsampleCounter--;
            }
        }
        else
        {
            saveLogData       = false;
            dataLogIndex      = 0U;
            dataInLog         = true;
            downsampleCounter = 0U;
        }
    }

    //
    //  We latch data for the motor angle display.
    //
    angleForDisplay =
        theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentAnalogHallAngle;
    speedForDisplay =
        theMotorController.theMotorStateMachine.feedbackMotorSpeed;

    //
    //  If it is time, we update the low rate items.
    //
    if (lowRateItemDownsampleCounter >= lowRateItemDownsample)
    {
        if (lowRateItemUpdateCounter == lastLowRateItemUpdateCounter)
        {
            lowRateUpdateTime = theTimers.getHighResTime();
            lowRateItem1 =
                theMotorController.theMotorStateMachine.feedbackMotorSpeed;
            lowRateItem2 =
                theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentAnalogHallAngle;
            lowRateItem3 =
                theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentEstimatedAngle;
            lowRateItem4 =
                theMotorController.theMotorStateMachine.feedbackCurrent_D;
            lowRateItem5 =
                theMotorController.theMotorStateMachine.feedbackCurrent_Q;

            lowRateItemUpdateCounter++;
            lowRateItemDownsampleCounter = 0U;

        }
    }
    else
    {
        lowRateItemDownsampleCounter++;
    }

    //
    //  For now, the DAC outputs are hard-coded.  Make sure you update the
    //  DAC scale factors in initializeDataLog for the data you are sending.
    //
    theDACInterface.sendToDAC1(
        //theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.bemf_Q_Filtered);
        //theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentAnalogHallAngle);
        theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.filteredSpeed.getFilterState());
        //theMotorController.theMotorStateMachine.feedbackMotorSpeed);
        //theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.getFilteredSpeed());
        //theMotorController.theMotorStateMachine.speedController.slewedTarget);
    theDACInterface.sendToDAC2(
        //theMotorController.theMotorStateMachine.feedbackCurrent_Q);
        //theMotorController.theMotorStateMachine.speedController.presentFeedback);
        //theMotorController.theMotorStateMachine.IQController.presentFeedback);
        //theMotorController.theMotorStateMachine.feedbackShaftAngle);
        //theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentEstimatedAngle);
        theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.presentRampSpeed);
        //theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.adjustedEstimatedAngle);
        //theMotorController.theMotorStateMachine.motorAngleAndSpeedManager.estimatedCurrentAlpha);

}

////////////////////////////////////////////////////////////////////////////////
//
//  This prints the contents of the data buffer to the console if it contains
//  data.  This will block the caller for the time required to print the
//  entire buffer.  This could be a significant amount of time so the caller
//  should be a low priority task.
//
////////////////////////////////////////////////////////////////////////////////
void dataLogger::printDataLogBuffer()
{
    //
    //  When there is data in the high speed log buffer, we print it to
    //  the console.
    //
    if (dataInLog)
    {
        for (uint32_t item = 0; item < itemsToBuffer; item++)
        {
            printf ("%d %d ", static_cast<int>(item), static_cast<int>(logData[item].time));
            printf ("%f ", logData[item].item1);
            printf ("%f ", logData[item].item2);
            printf ("%f ", logData[item].item3);
            printf ("%f ", logData[item].item4);
            printf ("%f ", logData[item].item5);
            printf ("%f ", logData[item].item6);
            printf ("%f ", logData[item].item7);
            printf ("%f ", logData[item].item8);

            printf ("\r\n");

            fflush(stdout);

            //
            //  We delay a bit for the line to make it out to the console.
            //
            OSInterface::delay(10U);
        }

        dataInLog = false;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This prints the low rate items.  These are sent to the console whenever there
//  is not something else going on.
//
////////////////////////////////////////////////////////////////////////////////
void dataLogger::printLowRateItems()
{
    using namespace positionDisplayConsts;

    //
    //  When there is new low rate data, we print it.  Note that by printing
    //  a carriage return with no line feed, we get a cool updating display
    //  effect (if I do say so myself).
    //
    if (lowRateItemUpdateCounter != lastLowRateItemUpdateCounter)
    {
        printf ("Time:  %10d", static_cast<int>(lowRateUpdateTime));
        printf ("  Speed (RPM Elec.): %8.2f", lowRateItem1);
        printf ("  Hall Angle (rad): %6.2f", lowRateItem2);
        printf ("  Est Angle (rad): %6.2f", lowRateItem3);
        printf ("  ID (amp): %8.4f", lowRateItem4);
        printf ("  IQ (amp): %8.4f", lowRateItem5);
        printf ("\r");

        lastLowRateItemUpdateCounter = lowRateItemUpdateCounter;

        fflush(stdout);
    }
    else
    {
        if (MotorConstants::displayShaftAngleOnLEDDisplay)
        {
            displayMotorAngle();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This lights an external LED that corresponds to the motor shaft angle.
//  This exists simply to take advantage of a feature included with the
//  external speed command encoder.
//
////////////////////////////////////////////////////////////////////////////////
void dataLogger::displayMotorAngle()
{
    using namespace positionDisplayConsts;

    //
    //  If the motor is disabled, we use the actual shaft angle for our
    //  display if we have an encoder.
    //
    if (!(theMotorController.theMotorStateMachine.motorEnabled()))
    {
        reducedAngle = angleForDisplay;
        lastDisplayUpdateTime = theTimers.getMedResTime();
    }
    else
    {
        //
        //  When the motor is running, we create an angle based on a reduced
        //  motor speed.  If we use the actual motor speed. the angle changes
        //  to quickly to produce a meaningful display.
        //
        const float32_t reducedSpeed =
            speedForDisplay * MotorConstants::speedReductionDivider;

        const time_s deltaSeconds =
            (theTimers.getMedResDelta(lastDisplayUpdateTime) *
            GlobalConstants::secondsPerMillisecone);
        lastDisplayUpdateTime = theTimers.getMedResTime();
        const electricalAngle_rad reducedAngleChange =
            reducedSpeed * deltaSeconds;
        reducedAngle =
            motorAngleAndSpeed::offsetShaftAngle(reducedAngle, reducedAngleChange);
    }

    //
    //  We don't start to update the display until the processor has been
    //  running for a while.  If we try to talk to the display too quickly
    //  after reset, it sometimes upsets the display controller.
    //
    if (theTimers.getLowResTime() > startupDelay)
    {
        //
        //  We light an LED based on the motor's position.
        //
        const float32_t radiansPerLED = GlobalConstants::twoPi / numberOfLEDs;
        uint8_t thisLED =
            static_cast<uint8_t>(reducedAngle / radiansPerLED);

        if (thisLED > maxLEDOffset)
        {
            thisLED = maxLEDOffset;
        }

        thisLED = thisLED + LED0BrightnessControlAddress;

        if (thisLED != lastPositionLED)
        {
            uint8_t lightLEDCommand[bytesInLEDCommand] = {thisLED, DefaultDutyCycle};

            theI2CInterface.writeToI2CDevice(
                LEDControllerAddress, lightLEDCommand, bytesInLEDCommand);

            //
            //  We shut off the last LED.
            //
            lightLEDCommand[registerIndex] = lastPositionLED;
            lightLEDCommand[valueIndex]    = LEDOffDutyCycle;
            theI2CInterface.writeToI2CDevice(
                LEDControllerAddress, lightLEDCommand, bytesInLEDCommand);

            lastPositionLED = thisLED;
        }
    }
}



