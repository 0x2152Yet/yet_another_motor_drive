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
//  This class performs the actual motor control
//
//  The motor control is handled by two "threads":  The high speed control
//  is performed by code that executes each time a new set of ADC motor data
//  is available.  The high speed control executes during the high priority ISR
//  that occurs each time a new set of data is collected.  The lower speed
//  control executes during a task that is scheduled by the RTOS.
//
//  The high speed control commutates the motor and closes its loops (e.g. the
//  motor current control loop).  It has fairly strict timing requirements
//  dictated by the physics of the motor and the frequency of the PWM signals that
//  are used to energize the motor phases.  The lower speed controller manages
//  the overall state of the motor (disabled, enabled, running, etc.) based on
//  commands to this class.  It also manages the data interface between the high
//  speed controller and the rest of the processor.
//
//  Care must be taken when engaging with the software in this class.  Physical
//  damage to the power electronics and/or motor may occur if the controllers do
//  not operate as intended.  Debugging with breakpoints is an area of particular
//  concern as stopping the processor can cause the motor's commutation to freeze
//  and allow excessive current to flow though the motor's windings.
//
////////////////////////////////////////////////////////////////////////////////
#include "adc_interface.h"
#include "config.h"
#include "dac_interface.h"
#include "data_logger.h"
#include "global_constants.h"
#include "global_definitions.h"
#include "loop_coefficients.h"
#include "motor_constants.h"
#include "motor_controller.h"
#include "motor_state_machine.h"
#include "os_interface.h"
#include "physical_inputs.h"
#include "pwm_interface.h"
#include "tick_timers.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

//
//  These are the global instances of the classes that manage the physical
//  hardware interfaces.
//
ADCInterface theADCInterface;
DACInterface theDACInterface;
PWMInterface thePWMInterface;

//
//  This manages the test support data log.
//
dataLogger theDataLogger;

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
MotorController::MotorController() :
    theMotorStateMachine(),
    desiredSpeed(0.0F),
    lastSpeedCommand(0.0F),
    speedCommandOffset(0.0F),
    speedToggleCommandTime(0U),
    lastSpeedCommandUpdateTime(0U),
    commandBuffer(),
    commandReadIndex(0U),
    commandWriteIndex(0U),
    isMotorError(false),
    motorIsEnabled(false),
    motorIsRunning(false)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Prepares the motor controller for operation.  This must be called before
//  the scheduler is started.
//
////////////////////////////////////////////////////////////////////////////////
void MotorController::initializeMotorController()
{
    //
    //  We register the controller task with the OS.
    //
    theOSInterface.createTask(
        theMotorController.motorControlTaskEntryPoint,
        taskInfo::MotorControlTaskPriority,
        taskInfo::MotorControlTaskStackSizeLongwords,
        this,
        taskInfo::MotorControlTaskName);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This is the high speed entry point for the motor controller.
//
//  This is called by an ISR and should operate accordingly:  The time-to-execute
//  should be monitored carefully.  Care must be taken to avoid read-modify-write
//  conflicts with other activities in the processor.  Anything invoked by this
//  function should not use RTOS services (delays, semaphores, etc.)
//
////////////////////////////////////////////////////////////////////////////////
void MotorController::updateMotorController()
{
    //
    //  The very first thing we do is invoke the state machine.  The state
    //  machine will kick off the next ADC data collection so we want to do
    //  that early in the process.
    //
    theMotorStateMachine.updateMotorStateMachine();

    //
    //  We see if we have a new command in our command buffer.
    //
    controllerCommands newCommand;
    float32_t commandParameter;
    getNextCommandFromBuffer(newCommand, commandParameter);

    switch (newCommand)
    {
        case noCommand:
            break;

        case startCommand:
            theMotorStateMachine.enableMotor();
            break;

        case stopCommand:
            theMotorStateMachine.stopMotor();
            break;

        case speedCommand:
            theMotorStateMachine.setMotorSpeed(commandParameter);
            break;

        case startLoggingCommand:
            theDataLogger.startBufferingData();
            break;

        default:
            break;
    }

    //
    //  We latch state data for the low speed controller.
    //
    isMotorError   = theMotorStateMachine.isMotorError();
    motorIsEnabled = theMotorStateMachine.motorEnabled();
    motorIsRunning = theMotorStateMachine.running();

    //
    //  We update the data log.
    //
    theDataLogger.updateDataLog();
}

////////////////////////////////////////////////////////////////////////////////
//
//  Static entry point required by the OS
//
//  This should be provided a pointer to the actual instance of the
//  motor controller class.  It will call the main loop by reference.
//
////////////////////////////////////////////////////////////////////////////////
void MotorController::motorControlTaskEntryPoint(void * const thisPtr)
{
    if (thisPtr != nullptr)
    {
        //
        //  We call the main loop using the passed in "this" pointer.
        //
        MotorController *const thisTaskPointer =
            reinterpret_cast<MotorController *const>(thisPtr);
        thisTaskPointer->motorControlTaskLoop();
    }

    //
    //  This call should not get here.  If it does, we'll loop with a delay
    //  to support debugging.
    //
    while (true)
    {
        OSInterface::delay(1000U);
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  Main loop for the motor control lower-speed task
//
//  This is invoked by the static task entry point.  It is scheduled by
//  the RTOS and never exits.
//
////////////////////////////////////////////////////////////////////////////////
void MotorController::motorControlTaskLoop()
{
    uint32_t statusBlinkCounter = 0U;

    //
    //  We initialize the data logger.
    //
    theDataLogger.initializeDataLog();

    //
    //  We prepare the motor and start the data flowing.
    //
    theMotorStateMachine.initializeMotorControlStateMachine();

    //
    //  This is the main loop.
    //
    while (true)
    {
        OSInterface::delay(10U);

        //
        //  We use the push button to control the motor state.
        //
        thePhysicalInputs.updatePushButtons();

        if (thePhysicalInputs.onboardPushButtonIsPressed())
        {
            if (isMotorError || motorIsEnabled)
            {
                //
                //  If the motor is running, we stop it.
                //
                addCommandToBuffer(stopCommand);
            }
            else
            {
                //
                //  We command the motor to start.  For now we always
                //  start at a nominal speed.
                //
                thePhysicalInputs.resetSpeedCommand();
                desiredSpeed = LoopCoefficients::defaultTargetSpeed;
                lastSpeedCommand = desiredSpeed;
                addCommandToBuffer(speedCommand, desiredSpeed);
                addCommandToBuffer(startCommand);
            }
        }

        //
        //  We check for a new speed comand.
        //
        determineSpeedCommand();

        //
        //  We check for a button press.  If it is pressed, we start logging high
        //  speed motor data.
        //
        if (thePhysicalInputs.externalPushButtonIsPressed())
        {
            addCommandToBuffer(startLoggingCommand);
        }

        //
        //  We blink the status LED based on the motor state.
        //
        const uint32_t motorIdleBlinkRate    = 200U;
        const uint32_t motorEnabledBlinkRate =  25U;
        const uint32_t motorRunningBlinkRate =  10U;
        uint32_t blinkRate = 0U;
        if (motorIsRunning)
        {
            blinkRate = motorRunningBlinkRate;
        }
        else if (motorIsEnabled)
        {
            blinkRate = motorEnabledBlinkRate;
        }
        else
        {
            blinkRate = motorIdleBlinkRate;
        }

        statusBlinkCounter++;

        if (statusBlinkCounter > blinkRate)
        {
            GPIOInterface::toggleStatusLED();
            statusBlinkCounter = 0U;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method reads external control inputs and determines the desired
//  motor speed.
//
////////////////////////////////////////////////////////////////////////////////
void MotorController::determineSpeedCommand()
{
    //
    //  We toggle the direction of the requested speed if the external button
    //  is held down.
    //
    if (thePhysicalInputs.externalPushButtonIsHeld())
    {

        if (MotorConstants::changeDirectionOnButtonHold)
        {
            if (MotorConstants::latchLogOnDirectionChange)
            {
                addCommandToBuffer(startLoggingCommand);
            }
            speedToggleCommandTime = theTimers.getMedResTime();
        }
        else
        {
            speedToggleCommandTime = 0U;
        }
    }

    if (speedToggleCommandTime != 0U)
    {
        if (theTimers.getMedResDelta(speedToggleCommandTime) >
            MotorConstants::timeToDelayBeforeSpeedToggle)
        {
            desiredSpeed = -(desiredSpeed + speedCommandOffset);
            thePhysicalInputs.resetSpeedCommand();
            speedToggleCommandTime = 0U;
        }
    }

    //
    //  We adjust the speed command based on the speed command input.  We
    //  read the encoder somewhat slowly so we don't pester the controller
    //  with too many speed command changes.
    //
    if (theTimers.getMedResDelta(lastSpeedCommandUpdateTime) >
        MotorConstants::speedCommandUpdateTime)
    {
        lastSpeedCommandUpdateTime = theTimers.getMedResTime();

        int32_t speedCommandCounts = thePhysicalInputs.getPresentSpeedCommandCounts();
        speedCommandOffset =
            static_cast<electricalSpeed_RPM>(speedCommandCounts) *
            MotorConstants::speedCommandGain;
    }

    //
    //  We limit the speed command and we enforce a deadband.
    //
    electricalSpeed_RPM finalSpeedCommand = desiredSpeed + speedCommandOffset;

    if (finalSpeedCommand > MotorConstants::maxSpeedCommand)
    {
        finalSpeedCommand = MotorConstants::maxSpeedCommand;
    }
    else if (finalSpeedCommand < -MotorConstants::maxSpeedCommand)
    {
        finalSpeedCommand = -MotorConstants::maxSpeedCommand;
    }

    if (fabs(finalSpeedCommand) < MotorConstants::speedCommandDeadband)
    {
        finalSpeedCommand = 0.0F;
    }

    if (finalSpeedCommand != lastSpeedCommand)
    {
        addCommandToBuffer(speedCommand, finalSpeedCommand);
        lastSpeedCommand = finalSpeedCommand;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This adds a new command to the command buffer.  This should only be called
//  by the low-priority controller task.
//
////////////////////////////////////////////////////////////////////////////////
void MotorController::addCommandToBuffer(
    const controllerCommands command, const float32_t parameter)
{
    //
    //  We start by computing the NEXT place to write.
    //
    volatile uint32_t nextWriteLocation = commandWriteIndex + 1U;
    if (nextWriteLocation >= commandsInBuffer)
    {
        nextWriteLocation = 0U;
    }

    //
    //  Now we see if there is room in the buffer.
    //
    if (nextWriteLocation != commandReadIndex)
    {
        //
        //  We can add the command.
        //
        commandBuffer[commandWriteIndex].desiredCommand   = command;
        commandBuffer[commandWriteIndex].commandParameter = parameter;
        commandWriteIndex = nextWriteLocation;
    }
    else
    {
        //
        //  The buffer is full.  In a critical application, this would
        //  probably be an error.
        //
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This retrieves the next command from the buffer.  This should only be
//  called by the high speed motor controller.
//
////////////////////////////////////////////////////////////////////////////////
void MotorController::getNextCommandFromBuffer(
        controllerCommands &command, float32_t &parameter)
{
    command   = noCommand;
    parameter = 0.0F;

    //
    //  We see if we have a new command.
    //
    if (commandWriteIndex != commandReadIndex)
    {
        //
        //  We get the new command.
        //
        command   = commandBuffer[commandReadIndex].desiredCommand;
        parameter = commandBuffer[commandReadIndex].commandParameter;
        commandBuffer[commandReadIndex].clearCommand();

        //
        // We update the location of the next command we will read.
        //
        volatile uint32_t nextReadLocation = commandReadIndex + 1U;
        if (nextReadLocation >= commandsInBuffer)
        {
            nextReadLocation = 0U;
        }

        commandReadIndex = nextReadLocation;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This shuts off the motor power electronics.  It may be called during error
//  or trap conditions.
//
////////////////////////////////////////////////////////////////////////////////
void shutOffMotor(void)
{
    GPIOInterface::disableBridgeOutput();
    thePWMInterface.disablePWMOutputs();
}

