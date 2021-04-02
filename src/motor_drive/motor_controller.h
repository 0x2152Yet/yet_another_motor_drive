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
//  This defines the "outer" motor controller.  This contains the entry point
//  and external interfaces for the motor controller.
//
//  The motor controller has two threads-of-control.  There is the high speed
//  code that commutates the motor and closes its various loops.  There is a lower
//  speed thread that manages the motor's high-level state machine and also manages
//  the interface to the "outside world".  The high speed code executes during
//  the interrupt that occurs whenever new motor data is available from the ADC.
//  The low speed thread operates as an endless loop that is scheduled by the
//  operating system.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "data_logger.h"
#include "global_definitions.h"
#include "motor_state_machine.h"


class MotorController
{
public:
    MotorController();
    ~MotorController() {};

    //
    //  This prepares the motor controller for operation.  This must be called
    //  before the RTOS scheduler is started.
    //
    void initializeMotorController();

    //
    //  This is the motor controller's high-speed entry point.  It is invoked
    //  by the ISR that services the new-ADC-data-is-available interrupt.
    //  Most of the actual closed-loop motor control software is invoked by this
    //  function.
    //
    void updateMotorController();

    //
    //  This is a static function intended to be provided to the OS at
    //  initialization.  It invokes the motor controller's lower speed thread.
    //
    static void motorControlThreadEntryPoint(void * const thisPtr);

private:

    //
    //  This is the state machine that performs most of the motor control.
    //  It runs under the context of an interrupt service routine.
    //
    motorStateMachine theMotorStateMachine;

    //
    //  This is the motor controller lower-speed, low priority control thread.
    //
    void motorControlThreadLoop();

    //
    //  These are used to change the speed command based on physical command
    //  inputs.
    //
    void determineSpeedCommand();
    electricalSpeed_RPM desiredSpeed;
    electricalSpeed_RPM lastSpeedCommand;
    electricalSpeed_RPM speedCommandOffset;
    tickTime_ms         speedToggleCommandTime;
    tickTime_ms         lastSpeedCommandUpdateTime;

    //
    //  These are used to communicate commands between the controller thread
    //  and the high-speed ISR motor controller.  We use a simple circular
    //  buffer of commands.
    //
    enum controllerCommands
    {
        noCommand,
        startCommand,
        stopCommand,
        speedCommand,
        startLoggingCommand
    };

    struct motorCommandStruct
    {
        motorCommandStruct() :
            desiredCommand(noCommand),
            commandParameter(0.0F)
        {
        }

        void clearCommand()
        {
            desiredCommand = noCommand;
            commandParameter = 0.0F;
        }

        controllerCommands desiredCommand;
        float32_t commandParameter;
    };

    void addCommandToBuffer(
        const controllerCommands command, const float32_t parameter = 0.0F);
    void getNextCommandFromBuffer(
        controllerCommands &command, float32_t &parameter);

    static const uint32_t commandsInBuffer = 10U;
    motorCommandStruct commandBuffer[commandsInBuffer];
    volatile uint32_t commandReadIndex;
    volatile uint32_t commandWriteIndex;

    //
    //  These are status items latched by the high speed controller
    //  that are monitored by the low-speed controller.
    //
    volatile bool isMotorError;
    volatile bool motorIsEnabled;
    volatile bool motorIsRunning;

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;
};

extern "C"
{
    //
    //  This shuts off the motor power electronics.  This may be called when
    //  ever an error or fault condition is detected.
    //
    void shutOffMotor(void);
}

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern MotorController theMotorController;


