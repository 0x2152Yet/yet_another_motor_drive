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
//  This class computes manages the motor control state machine.  The motor
//  state machine manages the states and transitions required to enable the
//  motor and bring it up to speed.  Most of the motor control is managed by
//  the motor state machine.
//
//  The required states vary with the type of control chosen for the motor.
//  When motor shaft angle position sensors are available (for example, analog
//  hall sensors) the state machine is fairly simple.  When sensor-less control
//  is selected, the startup sequence is more complex.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "adc_interface.h"
#include "data_logger.h"
#include "global_definitions.h"
#include "low_pass_filter.h"
#include "motor_angle_n_speed.h"
#include "motor_transforms.h"
#include "pi_controller.h"
#include "zero_sequence_modulator.h"

class motorStateMachine
{
public:
    motorStateMachine();
    ~motorStateMachine() {};

    //
    //  This prepares the motor for operation.  This includes initializing
    //  the motor control hardware and the state machine.
    //
    void initializeMotorControlStateMachine();

    //
    //  This enables the motor.  Calling this will start the process of
    //  transitioning the motor from an idle state to a state where it is
    //  running with controlled speed.
    //
    void enableMotor() { startCommanded = true; }

    //
    //  This disables the motor.  The motor will be immediately shut down
    //  regardless of this present state.
    //
    void stopMotor() { stopCommanded = true; }

    //
    //  This commands a new speed to the controller. At present the
    //  motor starts at a fixed speed.  This allows speed updates once
    //  the controller is running.
    //
    void setMotorSpeed (const electricalSpeed_RPM desiredSpeed) { speedTargetCommand = desiredSpeed; }

    //
    //  This performs an update of the state machine.
    //
    void updateMotorStateMachine();

    //
    //  These provide high level motor status.  The motorEnabled method
    //  indicates that the motor is in one of the enabled state while motorRunning
    //  indicates that the motor is running in a closed loop fashion.
    //
    bool motorEnabled() const
        { return !((presentMotorState == motorIdleState) || (presentMotorState == motorErrorState)); }
    bool running()      const
        { return presentMotorState == closedLoopControlState; }

    //
    //  This is a helper method to get the direction associated with a given
    //  speed.  This is different than the "sgn" function in that a speed of
    //  zero is considered positive.
    //
    const float32_t positiveDirection = 1.0F;
    const float32_t negativeDirection = -1.0F;
    float32_t getMotorDirection(const electricalSpeed_RPM speed) const;

    //
    //  This indicates if the motor is in an error state.  The stopMotor
    //  method must be called to clear an error before a new enable may
    //  be attempted.
    //
    bool isMotorError() const { return errorDetected; }

private:

    //
    //  These are the possible motor states.  Not all states are used in all
    //  control methods.
    //
    typedef enum
    {
        motorIdleState,
        warmUpBridgeState,
        warmUpPWMsState,
        alignRotorState,
        openLoopStartupState,
        transitionToClosedLoopState,
        closedLoopControlState,
        motorErrorState,
        gainTuningState
    } motorStates;

    //
    //  These process each state.  Each returns the next state for the
    //  state machine.
    //
    void processIdleState(motorStates &newState);
    void processWarmUpBridgeState(motorStates &newState);
    void processWarmUpPWMsState(motorStates &newState);
    void processAlignRotorState(motorStates &newState);
    void processOpenLoopStartupState(motorStates &newState);
    void processTransitionToClosedLoopState(motorStates &newState);
    void processClosedLoopControlState(motorStates &newState);
    void processMotorErrorState(motorStates &newState);
    void processGainTuningState(motorStates &newState);

    //
    //  This makes the decisions required when a speed ramp has
    //  completed.
    //
    void processOpenLoopSpeedRampComplete(motorStates &newState);

    //
    //  This indicates if two speeds are, for all intensive porpoises,
    //  equal.
    //
    bool speedsAreEqual(
        const electricalSpeed_RPM firstSpeed,
        const electricalSpeed_RPM secondSpeed) const;

    //
    //  This starts or updates the open-loop ramp to the target speed.
    //
    void initializeOpenLoopSpeedRamp(
        const electricalAngle_rad presentAngle,
        const electricalSpeed_RPM presentSpeed);

    //
    //  These are used to control the state and manage state transitions.
    //
    motorStates presentMotorState;
    tickTime_ms stateEntryTime;
    bool startCommanded;
    bool stopCommanded;
    bool errorDetected;
    bool initializeOffsetCalibration;

    //
    //  This performs activities that are common to all states.
    //
    void performCommonActions();

    //
    //  These items manage the sensor feedback data used for each motor update.
    //  We store the in both its raw and converted form to for debugging
    //  convenience.
    //
    void convertSensorInputs();
    lowPassFilter phaseACurrentOffset;
    lowPassFilter phaseBCurrentOffset;
    lowPassFilter phaseCCurrentOffset;

    //
    //  These items are used to control the motor current loops
    //
    void controlMotorCurrents();
    motorTransforms theMotorTransforms;
    zeroSequenceModulator theZeroSequenceModulator;

    //
    //  This determines the motor's shaft angle and speed.
    //
    motorAngleAndSpeed motorAngleAndSpeedManager;

    //
    //  We may have to slew-limit the current command used during open-loop
    //  control.
    //
    amperes slewedOpenLoopIQCommand;

    //
    //  These items are used to transition from controlling the motor
    //  with simulated shaft angle to an estimate of the motor's actual
    //  shaft angle.
    //
    float32_t angleHandoverOffset;
    float32_t angleOffsetWeight;

    //
    //  These are the controllers for the motor D component current,
    //  Q component current, and motor speed.
    //
    PIController speedController;
    PIController IDController;
    PIController IQController;

    //
    //  This is used in the gain tuning state.
    //
    bool isFirstSetting;

    //
    //  This shuts the motor down when a stop is commanded.
    //
    void shutDownMotor();

    //
    //  This checks for transient over-current on the feedback current
    //  and shuts the motor down.
    //
    void monitorMotorCurrents();

    //
    //  These items support an auto-shut down feature.  When enabled,
    //  the motor will only run for a fixed number of frames before
    //  disabling.  This is useful for verifying feedback signs, etc.
    //
    void monitorAutoShutdown();
    uint32_t autoShutDownCounter;

    //
    //  We store the results of all of the processing steps to support test
    //  and integration.  The first items are the motor feedback and sensor
    //  data it its various forms.
    //
    ADCInterface::rawMotorData dataFromADC;
    rawADCCounts               rawHallA;
    rawADCCounts               rawHallB;
    rawADCCounts               rawHallC;
    amperes   phaseACurrent;
    amperes   phaseBCurrent;
    amperes   phaseCCurrent;
    amperes   busCurrent;
    volts     phaseAVoltage;
    volts     phaseBVoltage;
    volts     phaseCVoltage;
    volts     busVoltage;
    float32_t analogHallA;
    float32_t analogHallB;
    float32_t analogHallC;

    //
    //  These are the shaft angle and speed actually used for the motor control.
    //
    electricalAngle_rad feedbackShaftAngle;
    electricalSpeed_RPM feedbackMotorSpeed;

    //
    //  The next group of items are the results of the various transformations
    //  performed on the motor feedback.
    //
    amperes feedbackCurrent_Alpha;
    amperes feedbackCurrent_Beta;
    amperes feedbackCurrent_D;
    amperes feedbackCurrent_Q;

    //
    //  These are the targets for the controllers.
    //
    electricalSpeed_RPM speedTargetCommand;
    amperes IDTargetCommand;
    amperes IQTargetCommand;

    //
    //  These are the outputs from the current control loops.
    //
    signedDutyCycle IDCommandOutput;
    signedDutyCycle IQCommandOutput;

    //
    //  These are the various intermediate forms the output command takes as
    //  it is transformed into the final command for the motor.
    //
    signedDutyCycle currentCommand_Alpha;
    signedDutyCycle currentCommand_Beta;
    signedDutyCycle nonModulatedDutyA;
    signedDutyCycle nonModulatedDutyB;
    signedDutyCycle nonModulatedDutyC;
    dutyCycle_pct   outputDutyA;
    dutyCycle_pct   outputDutyB;
    dutyCycle_pct   outputDutyC;

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;

};
