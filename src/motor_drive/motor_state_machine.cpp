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
//  This class computes manages the motor control state machine.   The motor
//  state machine is responsible for the operations required to start the
//  motor and get it running in a closed loop fashion.  Most of the motor
//  control takes place in this class.
//
////////////////////////////////////////////////////////////////////////////////
#include "adc_interface.h"
#include "config.h"
#include "dac_interface.h"
#include "global_constants.h"
#include "global_definitions.h"
#include "gpio_interface.h"
#include "loop_coefficients.h"
#include "math_util.h"
#include "motor_angle_n_speed.h"
#include "motor_constants.h"
#include "motor_controller.h"
#include "motor_transforms.h"
#include "motor_state_machine.h"
#include "pwm_interface.h"
#include "tick_timers.h"
#include "zero_sequence_modulator.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
motorStateMachine::motorStateMachine() :
    presentMotorState(motorIdleState),
    stateEntryTime(0U),
    startCommanded(false),
    stopCommanded(false),
    errorDetected(false),
    initializeOffsetCalibration(false),
    phaseACurrentOffset(),
    phaseBCurrentOffset(),
    phaseCCurrentOffset(),
    theMotorTransforms(),
    theZeroSequenceModulator(),
    motorAngleAndSpeedManager(),
    slewedOpenLoopIQCommand(0.0F),
    angleHandoverOffset(0.0F),
    angleOffsetWeight(0.0F),
    speedController(),
    IDController(),
    IQController(),
    isFirstSetting(false),
    autoShutDownCounter(0U),
    dataFromADC(),
    rawHallA(0U),
    rawHallB(0U),
    rawHallC(0U),
    phaseACurrent(0.0F),
    phaseBCurrent(0.0F),
    phaseCCurrent(0.0F),
    busCurrent(0.0F),
    phaseAVoltage(0.0F),
    phaseBVoltage(0.0F),
    phaseCVoltage(0.0F),
    busVoltage(0.0F),
    analogHallA(0.0F),
    analogHallB(0.0F),
    analogHallC(0.0F),
    feedbackShaftAngle(0.0F),
    feedbackMotorSpeed(0.0F),
    feedbackCurrent_Alpha(0.0F),
    feedbackCurrent_Beta(0.0F),
    feedbackCurrent_D(0.0F),
    feedbackCurrent_Q(0.0F),
    speedTargetCommand(0.0F),
    IDTargetCommand(0.0F),
    IQTargetCommand(0.0F),
    IDCommandOutput(0.0F),
    IQCommandOutput(0.0F),
    currentCommand_Alpha(0.0F),
    currentCommand_Beta(0.0F),
    nonModulatedDutyA(0.0F),
    nonModulatedDutyB(0.0F),
    nonModulatedDutyC(0.0F),
    outputDutyA(0.0F),
    outputDutyB(0.0F),
    outputDutyC(0.0F)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This prepares the motor and the state machine for operation.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::initializeMotorControlStateMachine()
{
    using namespace LoopCoefficients;
    using namespace MotorConstants;

    //
    //  We filter the phase currents during motor startup to determine their
    //  zero offset.
    //
    phaseACurrentOffset.computeFilterCoefficient(
        phaseCurrentOffsetFilterFrequency, controllerUpdateFrequency);
    phaseBCurrentOffset.computeFilterCoefficient(
        phaseCurrentOffsetFilterFrequency, controllerUpdateFrequency);
    phaseCCurrentOffset.computeFilterCoefficient(
        phaseCurrentOffsetFilterFrequency, controllerUpdateFrequency);

    //
    //  We initialize the motor speed calculation.
    //
    motorAngleAndSpeedManager.initializeSpeedCalculation(
        MotorConstants::controllerUpdateFrequency,
        MotorConstants::motorSpeedUpdateFrequency);

    //
    //  We initialize the processing for the enabled types of motor angle
    //  measurement.
    //
    if (MotorConstants::estimateShaftAngle)
    {
        motorAngleAndSpeedManager.computeSensorlessEstimatorCoefficients(
            SegwayMotorStatorResistance,
            SegwayMotorStatorInductance,
            SegwayMotorBackEMF,
            SegwayMotorPolePairs,
            controllerUpdateFrequency);
    }

    //
    //  We set up the PI controllers.
    //
    speedController.initializePIController(
        speedPGain,
        speedIGain,
        0.0F,
        maxSpeedCurrentCommand,
        maxSpeedIntegrator,
        targetSpeedSlewLimit);

    IDController.initializePIController(
        currentLoopPGain,
        currentLoopIGain,
        0.0F,
        maxCurrentLoopOutputDutyCommand,
        maxCurrentLoopIntegrator);

    IQController.initializePIController(
        currentLoopPGain,
        currentLoopIGain,
        0.0F,
        maxCurrentLoopOutputDutyCommand,
        maxCurrentLoopIntegrator);

    //
    //  We setup the basic motor control hardware and start the ADC.  The ADC
    //  data-ready interrupts will cause the high speed controller to begin
    //  executing.
    //
    thePWMInterface.initializePWMInterface(
        MotorConstants::PWMCarrierFrequency,
        MotorConstants::commandedPWMBlinkingPeriod,
        MotorConstants::dutyCycleCommandLimit);

    theADCInterface.initializeADCInterface();
    theADCInterface.startADCCollections();
}

////////////////////////////////////////////////////////////////////////////////
//
//  This is the main state machine entry point.  It performs actions that are
//  are common to all states and invokes the various methods that process
//  each specific state.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::updateMotorStateMachine()
{
    //
    //  We get the raw data from the ADC.
    //
    theADCInterface.getRawMotorData(dataFromADC);
    theADCInterface.getRawAnalogHallInputs(rawHallA, rawHallB, rawHallC);

    //
    //  We can now let the ADC command the next frame's data collection.  We want
    //  to do this early in the update so that we don't miss the next start-of-
    //  conversion trigger.
    //
    theADCInterface.commandADCDataCollection();

    //
    //  Now, we perform the actions that are state-independent.
    //
    performCommonActions();

    //
    //  If an error has been detected, we force the state machine to its
    //  error state.
    //
    if (isMotorError())
    {
        presentMotorState = motorErrorState;
    }

    //
    //  We check to see if we should perform an auto-shutdown.
    //
    monitorAutoShutdown();

    //
    //  We check for a stop command.  This shuts down motor.  This will also
    //  clear a motor error.
    //
    if (stopCommanded)
    {
        stopCommanded     = false;
        startCommanded    = false;
        errorDetected     = false;
        presentMotorState = motorIdleState;
        shutDownMotor();
        GPIOInterface::turnTriggerLEDOff();
    }

    //
    //  Now we execute the state machine.
    //
    motorStates newState;

    switch(presentMotorState)
    {
        case motorIdleState:
            processIdleState(newState);
            break;

        case warmUpBridgeState:
            processWarmUpBridgeState(newState);
            break;

        case warmUpPWMsState:
            processWarmUpPWMsState(newState);
            break;

        case alignRotorState:
            processAlignRotorState(newState);
            break;

        case openLoopStartupState:
            processOpenLoopStartupState(newState);
            break;

        case transitionToClosedLoopState:
            processTransitionToClosedLoopState(newState);
            break;

        case closedLoopControlState:
            processClosedLoopControlState(newState);
            break;

        case gainTuningState:
            processGainTuningState(newState);
            break;

        case motorErrorState:
        default:
            processMotorErrorState(newState);
            break;
    }

    if (newState != presentMotorState)
    {
        SystemConfig::turnTimingLEDOnForEvent(SystemConfig::timeStateChanges);

        //
        //  We have transitioned states so we note the new start time.
        //
        presentMotorState = newState;
        stateEntryTime = theTimers.getMedResTime();

        SystemConfig::turnTimingLEDOffForEvent(SystemConfig::timeStateChanges);

    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method provides the "direction" (AKA sign) of a given motor speed
//  command.
//
////////////////////////////////////////////////////////////////////////////////
float32_t motorStateMachine::getMotorDirection(const electricalSpeed_RPM speed) const
{
    //
    //  Note that we do not use the sgn of the commanded speed as we want to
    //  treat zero as positive.
    //
    float32_t returnDirection;
    if (speed >= 0.0F)
    {
        returnDirection = positiveDirection;
    }
    else
    {
        returnDirection = negativeDirection;
    }

    return returnDirection;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method manages the motor's idle state.  Presently
//  this involves starting up the motor controller when commanded to do so.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processIdleState(motorStates &newState)
{
    newState = motorIdleState;

    if (startCommanded)
    {
        //
        //  We enable the bridge driver with the PWMs disabled to allow the
        //  basic power electronics to turn on.
        //
        speedTargetCommand  = LoopCoefficients::defaultTargetSpeed;
        startCommanded      = false;
        autoShutDownCounter = MotorConstants::autoShutdownFrames;
        newState            = warmUpBridgeState;
        thePWMInterface.disablePWMOutputs();
        GPIOInterface::enableBridgeOutput();
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method monitors the bridge driver warm up state.  In this state,
//  the power electronics are given time to power on before the bridge starts
//  to switch.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processWarmUpBridgeState(motorStates &newState)
{
    newState = warmUpBridgeState;

    //
    //  We give the bridge driver some time to fully come out of
    //  reset.
    //
    if (theTimers.getMedResDelta(stateEntryTime) > MotorConstants::bridgeDriverWarmupTime)
    {
        //
        //  We start the PWMs with a nominal duty cycle.  No current should flow
        //  through the motor at this point.
        //
        thePWMInterface.enablePWMOutputs();
        newState = warmUpPWMsState;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method controls the period of time when the bridge is enabled and the
//  PWMs are switching but no current should be flowing through the motor.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processWarmUpPWMsState(motorStates &newState)
{
    using namespace LoopCoefficients;

    newState = warmUpPWMsState;

    if (MotorConstants::computeCurrentSensorOffsets)
    {
        //
        //  The phase currents should all be reading zero amperes.  Any non-zero
        //  value is considered a sensor offset.  We filter the currents during
        //  this state to compute an offset for each phase current.
        //
        if (theTimers.getMedResDelta(stateEntryTime) >
            MotorConstants::initialOffsetCalSettlingTime)
        {
            float32_t tempResult;
            phaseACurrentOffset.updateFilter(phaseACurrent, tempResult);
            phaseBCurrentOffset.updateFilter(phaseBCurrent, tempResult);
            phaseCCurrentOffset.updateFilter(phaseCCurrent, tempResult);
        }
        else
        {
            //
            //  At the beginning of the state, we initialize the filters.
            //
            phaseACurrentOffset.setFilterState(phaseACurrent);
            phaseBCurrentOffset.setFilterState(phaseBCurrent);
            phaseCCurrentOffset.setFilterState(phaseCCurrent);
        }
    }

    //
    //  Once we have given the PWMs enough time to settle and finished any
    //  startup calibrations, we can start the next state.
    //
    if (theTimers.getMedResDelta(stateEntryTime) > MotorConstants::PWMWarmupTime)
    {
        //
        //  The next state will start controlling the motor current so
        //  we re-initialize the controllers.
        //
        IDController.reInitializePIController();
        IQController.reInitializePIController();

        //
        //  Our next state depends on how we are controlling the motor.  If
        //  we are performing sensor-less control, we must start the open-loop
        //  speed ramp-up by aligning the motor rotor.  If we are using an
        //  actual shaft encoder, we can simply start the controller.
        //
        if (MotorConstants::currentControlAngleSelection ==
            MotorConstants::controlWithEstimatedAngle)
        {
            newState = alignRotorState;
        }
        else
        {
            //
            //  We are going straight to closed-loop control.  We initialize
            //  the speed controller with a target speed of zero.  This will
            //  cause it to slew up to the actual target speed.  Recall that
            //  when the PI controller is initialized, it sets its target to
            //  the provided value.  When its target is set via call to
            //  setControllerTarget, the loop's target is slewed to that value.
            //
            speedController.reInitializePIController(
                0.0F,
                0.0F);

            newState = closedLoopControlState;
        }

        //
        //  If the gain tuning state has been selected, we override the
        //  normal state transition.
        //
        if (MotorConstants::enableGainTuning)
        {
            newState = gainTuningState;
            isFirstSetting = true;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method controls the align rotor state.  In this state, a fixed
//  current command is sent to the motor.  This should cause the motor's
//  rotor to align to a known startup position.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processAlignRotorState(motorStates &newState)
{
    newState = alignRotorState;

    //
    //  To align the rotor we command a fixed current with a fixed value
    //  for the shaft angle.
    //
    feedbackShaftAngle = LoopCoefficients::rotorAlignmentPosition;
    feedbackMotorSpeed = 0.0F;
    IDTargetCommand    =
        LoopCoefficients::targetDComponentCurrent *
        getMotorDirection(speedTargetCommand);
    IQTargetCommand    =
        LoopCoefficients::openLoopStartupQComponentCurrent *
        getMotorDirection(speedTargetCommand);

    //
    //  This is the first state where we actually perform current control.
    //
    controlMotorCurrents();

    //
    //  Once we've given the rotor time to align to its start position, we
    //  can start to ramp up the motor speed.
    //
    if (theTimers.getMedResDelta(stateEntryTime) > LoopCoefficients::timeForRotorAlignment)
    {
        //
        //  We initialize the open-loop speed ramp and the sensor-less shaft
        //  angle estimator.
        //
        initializeOpenLoopSpeedRamp(LoopCoefficients::openLoopStartingPosition,
                                    0.0F);
        slewedOpenLoopIQCommand = IQTargetCommand;
        newState = openLoopStartupState;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method controls the open-loop ramp to speed.  This is used to start
//  the motor when a shaft angle encoder is not used to control the motor.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processOpenLoopStartupState(motorStates &newState)
{
    using namespace MotorConstants;
    using namespace LoopCoefficients;

    newState = openLoopStartupState;

    //
    //  We update the open-loop speed ramp and the estimated shaft angle.
    //
    motorAngleAndSpeedManager.updateSpeedRamp();

    //
    //  We continue to use a fixed current command but the shaft angle is
    //  that produced by the ramp.
    //
    feedbackShaftAngle  = motorAngleAndSpeedManager.getPresentRampAngle();
    feedbackMotorSpeed  = motorAngleAndSpeedManager.getPresentRampSpeed();
    IDTargetCommand     =
        LoopCoefficients::targetDComponentCurrent *
        getMotorDirection(speedTargetCommand);

    //
    //  We may have to slew in the IQ command.  If we are moving from closed-
    //  loop control back to open-loop control, we were most likely controlling
    //  the motor with a much lower current command.  If we don't slew-limit
    //  the command, the big step change will actually cause the motor to suddenly
    //  speed up.
    //
    const amperes finalIQTargetCommand =
        LoopCoefficients::openLoopStartupQComponentCurrent *
        getMotorDirection(speedTargetCommand);

    slewLimit(
        slewedOpenLoopIQCommand,
        finalIQTargetCommand,
        OpenLoopIQCommandSlewLimit);

    IQTargetCommand = slewedOpenLoopIQCommand;

    controlMotorCurrents();

    //
    //  When the ramp completes, we have a couple of different possibilities.
    //
    if (motorAngleAndSpeedManager.speedRampComplete())
    {
        processOpenLoopSpeedRampComplete(newState);
    }
    else
    {
        //
        //  @TODO:  Handle speed changes while running open loop at times
        //  other than the end of the ramp.  This is trickier than it seems
        //  because if the target is faster than the open-to-closed loop
        //  transition speed, it will never match the ramp target so a simple
        //  comparison won't work.
        //
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method controls the transition between the open-loop ramp-to-speed
//  and closed loop speed control.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processTransitionToClosedLoopState(motorStates &newState)
{
    newState = transitionToClosedLoopState;

    //
    //  When we transition from the open-loop startup to the closed loop
    //  speed control we slew the angle used in the current loop transforms
    //  from the simulated startup angle to the "actual" estimated angle.
    //  We do this by offsetting the estimated angle by the difference
    //  that was latched at the end of the open-loop startup.  We will
    //  reduce the offset each pass through this state.
    //
    feedbackShaftAngle = motorAngleAndSpeedManager.getPresentEstimatedAngle();

    feedbackShaftAngle = motorAngleAndSpeed::offsetShaftAngle(
        feedbackShaftAngle,
        (angleHandoverOffset * angleOffsetWeight));

    //
    //  In this state we start to use the speed controller to generate the
    //  current loop commands.  If the target is different from the speed
    //  ramp speed, we will start to move to the desired target speed.
    //
    if (motorAngleAndSpeedManager.newSpeedAvailable())
    {
        feedbackMotorSpeed = motorAngleAndSpeedManager.getFilteredSpeed();
        speedController.setPIControllerTarget(speedTargetCommand);
        speedController.updatePIController(feedbackMotorSpeed, IQTargetCommand);
    }

    IDTargetCommand =
        LoopCoefficients::targetDComponentCurrent *
        getMotorDirection(speedTargetCommand);

    controlMotorCurrents();

    //
    //  We reduce the weighting value that is used to reduce the offset between
    //  the last simulated angle and the actual estimated angle.  When we have
    //  completely eliminated the offset, we are done with this state.
    //
    angleOffsetWeight =
        angleOffsetWeight - LoopCoefficients::angleOffsetWeightReductionPerFrame;

    if (angleOffsetWeight <= 0.0F)
    {
        angleOffsetWeight = 0.0F;
        newState          = closedLoopControlState;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method is active once the motor is running with closed-loop speed
//  control.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processClosedLoopControlState(motorStates &newState)
{
    newState = closedLoopControlState;

    //
    //  The source of our motor speed and shaft angle depends on the
    //  control method.
    //
    feedbackShaftAngle = 0.0F;

    if (MotorConstants::currentControlAngleSelection ==
        MotorConstants::controlWithMeasuredAnalogAngle)
    {
        //
        //  As a check, we make sure we actually have sensors.
        //
        if (MotorConstants::computeAnalogHallBasedShaftAngle)
        {
            feedbackShaftAngle = motorAngleAndSpeedManager.getPresentAnalogHallAngle();
        }
        else
        {
            //
            //  Oops..we don't have the proper encoder.
            //
            errorDetected = true;
        }
    }
    else if (MotorConstants::currentControlAngleSelection ==
             MotorConstants::controlWithEstimatedAngle)
    {
        if (MotorConstants::estimateShaftAngle)
        {
            feedbackShaftAngle = motorAngleAndSpeedManager.getPresentEstimatedAngle();
        }
        else
        {
            errorDetected = true;
        }
    }
    else
    {
        errorDetected = true;
    }

    //
    //  If the speed was updated, we update the speed controller.
    //
    if (motorAngleAndSpeedManager.newSpeedAvailable())
    {
        feedbackMotorSpeed = motorAngleAndSpeedManager.getFilteredSpeed();
        speedController.setPIControllerTarget(speedTargetCommand);
        speedController.updatePIController(feedbackMotorSpeed, IQTargetCommand);
    }

    controlMotorCurrents();

    if (MotorConstants::currentControlAngleSelection ==
        MotorConstants::controlWithEstimatedAngle)
    {
        //
        //  We have to switch back to open-loop control if the present speed
        //  is less than speed required to compute an accurate angle estimate.
        //
        //  @TODO:  Can the angle estimator provide any insight to when it has
        //   good angle data?
        //
        if (fabs(feedbackMotorSpeed) <
            LoopCoefficients::closedToOpenLoopSpeedControlThreshold)
        {
            initializeOpenLoopSpeedRamp(feedbackShaftAngle, feedbackMotorSpeed);

            //
            //  We want to avoid a big step-change in the current command from
            //  what we've been using during closed-loop control to the command
            //  used for open-loop control.
            //
            slewedOpenLoopIQCommand = IQTargetCommand;

            //
            //  We can go back to open-loop control.
            //
            newState = openLoopStartupState;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method is active when the motor has entered an error state.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processMotorErrorState(motorStates &newState)
{
    newState = motorErrorState;

    //
    //  The only way out of an error state is with a stop command.  If a start
    //  command comes along, we ignore it.
    //
    startCommanded      = false;
    autoShutDownCounter = MotorConstants::dontAutoShutdown;
    shutDownMotor();
    GPIOInterface::turnTriggerLEDOn();
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method is active when the motor has entered an error state.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processGainTuningState(motorStates &newState)
{
    using namespace MotorConstants;

    newState = gainTuningState;

    //
    //  If we are configured to control the motor with a measured shaft
    //  angle, we use it.  Otherwise we use a fixed shaft angle as the
    //  angle estimator will likely not operate properly in this state.
    //
    feedbackShaftAngle = 0.0F;

    if (MotorConstants::currentControlAngleSelection ==
        MotorConstants::controlWithMeasuredAnalogAngle)
    {
        //
        //  As a check, we make sure we actually have sensors.
        //
        if (MotorConstants::computeAnalogHallBasedShaftAngle)
        {
            feedbackShaftAngle = motorAngleAndSpeedManager.getPresentAnalogHallAngle();
        }
    }

    //
    //  We toggle between fixed commands.
    //
    if (isFirstSetting)
    {
        IQTargetCommand = IQCommand_1;
        IDTargetCommand = IDCommand_1;
    }
    else
    {
        IQTargetCommand = IQCommand_2;
        IDTargetCommand = IDCommand_2;
    }

    //
    //  We toggle the loop commands at a periodic rate.
    //
    if (theTimers.getMedResDelta(stateEntryTime) > MotorConstants::gainTuningToggleTime)
    {
        stateEntryTime = theTimers.getMedResTime();

        if (isFirstSetting)
        {
            isFirstSetting = false;
        }
        else
        {
            isFirstSetting = true;
        }
    }

    controlMotorCurrents();

}

////////////////////////////////////////////////////////////////////////////////
//
//  This method makes the decisions required when a speed ramp has completed.
//  The result may be to transition states or the motor may stay in the open-
//  loop speed control.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::processOpenLoopSpeedRampComplete(motorStates &newState)
{
    using namespace MotorConstants;
    using namespace LoopCoefficients;

    newState = openLoopStartupState;

    const electricalSpeed_RPM estimatedMotorSpeed =
        motorAngleAndSpeedManager.getFilteredSpeed();

    if (fabs(estimatedMotorSpeed) > openToClosedLoopSpeedControlThreshold)
    {
        //
        //  We are moving fast enough to use the estimated angle for closed-loop
        //  speed control so we prepare for the transition to closed loop control.
        //  Up until this point, we have been using a simulated motor angle to
        //  transform current feedback to the rotating frame form used to close the
        //  current loops.  This means that the ID and IQ current vector components
        //  are not actually aligned with the motor's rotor.  When we switch
        //  to closed loop control we will start using an estimated rotor
        //  angle that should be close to the actual rotor angle.  Rather than
        //  just let the angle jump, we will slew the angle from its present
        //  simulated value to the estimated value. We note the difference between
        //  the simulated angle and the estimated angle.  We will used this
        //  difference to slew from the simulated angle to the estimated angle
        //  when we transition.
        //
        const electricalAngle_rad simulatedAngle =
            motorAngleAndSpeedManager.getPresentRampAngle();
        const electricalAngle_rad estimatedAngle =
            motorAngleAndSpeedManager.getPresentEstimatedAngle();

        angleHandoverOffset = simulatedAngle - estimatedAngle;
        angleOffsetWeight   = initialAngleOffsetWeight;

        //
        //  Me make sure we pick the most close offset distance.  It never makes
        //  sense to move more that half a rotation.
        //
        if (angleHandoverOffset > GlobalConstants::Pi)
        {
            angleHandoverOffset = angleHandoverOffset - GlobalConstants::twoPi;
        }
        else if (angleHandoverOffset < -GlobalConstants::Pi)
        {
            angleHandoverOffset = angleHandoverOffset + GlobalConstants::twoPi;
        }
        else
        {
            //  Offset is already less than half a rotation.
        }

        //
        //  We initialize the speed controller.  We seed it with our present
        //  speed and current command.
        //
        speedController.reInitializePIController(
            motorAngleAndSpeedManager.getFilteredSpeed(),
            openLoopStartupQComponentCurrent * getMotorDirection(speedTargetCommand));

        //
        //  We start to transition to closed loop control.
        //
        newState = transitionToClosedLoopState;
    }
    else
    {
        //
        //  If our ramp is complete and we moving too slowly to transition
        //  to closed-loop control it either means that we have a slow
        //  target speed, or we need to start a need ramp to a new target.
        //  In either case, we stay in the open-loop startup state.
        //
        if (!(speedsAreEqual(feedbackMotorSpeed, speedTargetCommand)))
        {
            initializeOpenLoopSpeedRamp(feedbackShaftAngle, feedbackMotorSpeed);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method determines if two provided speeds are essentially equal.
//
////////////////////////////////////////////////////////////////////////////////
bool motorStateMachine::speedsAreEqual(
    const electricalSpeed_RPM firstSpeed,
    const electricalSpeed_RPM secondSpeed) const
{
    bool essentiallyEqual;

    if (fabs(firstSpeed - secondSpeed) < LoopCoefficients::essentiallyEqualSpeed)
    {
        essentiallyEqual = true;
    }
    else
    {
        essentiallyEqual = false;
    }

    return essentiallyEqual;

}

////////////////////////////////////////////////////////////////////////////////
//
//  This method initializes the start-up open-loop ramp.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::initializeOpenLoopSpeedRamp(
    const electricalAngle_rad presentAngle,
    const electricalSpeed_RPM presentSpeed)
{
    //
    //  We have several cases for the target speed our our ramp.  The first
    //  case is where the target speed will move the motor in a different
    //  direction than our present speed.  If this is the case, we must stop
    //  and then restart the motor.
    //
    electricalSpeed_RPM openLoopTargetSpeed;

    if ((!speedsAreEqual(presentSpeed, 0.0F)) &&
        (sgn(presentSpeed) != sgn(speedTargetCommand)))
    {
        openLoopTargetSpeed = 0.0F;
    }
    else
    {
        //
        //  We can start the ramp to speed.   Normally, our target speed is a
        //  speed that is fast enough to allow the shaft feedback angle estimate
        //  to determine an accurate angle.  However, if the target speed is
        //  less than the minimum estimator speed we will ramp to target speed.
        //  In that case, we will stay in the open-loop state once the ramp
        //  completes.
        //
        if (fabs(speedTargetCommand) <
            fabs(LoopCoefficients::openLoopRampTargetSpeed))
        {
            openLoopTargetSpeed = speedTargetCommand;
        }
        else
        {
            openLoopTargetSpeed =
               fabs(LoopCoefficients::openLoopRampTargetSpeed) *
               getMotorDirection(speedTargetCommand);
        }
    }

    motorAngleAndSpeedManager.startSpeedRamp(
        presentAngle,
        presentSpeed,
        openLoopTargetSpeed,
        LoopCoefficients::openLoopSpeedAcceleration,
        MotorConstants::controllerUpdateFrequency);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method performs actions common to all motor states.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::performCommonActions()
{
    //
    //  We convert our new ADC data into proper engineering units.
    //
    convertSensorInputs();

    //
    //  We compute the motor shaft angle(s) based on the selected method(s).
    //
    if (MotorConstants::computeAnalogHallBasedShaftAngle)
    {
        motorAngleAndSpeedManager.updateAnalogHallEncoderBasedAngle(
            analogHallA,
            analogHallB,
            analogHallC);
    }

    if (MotorConstants::estimateShaftAngle)
    {
       motorAngleAndSpeedManager.updateSensorlessAngle(
            busVoltage,
            feedbackCurrent_Alpha,
            feedbackCurrent_Beta,
            outputDutyA,
            outputDutyB,
            outputDutyC);
    }

    //
    //  We update the motor speed.
    //
    motorAngleAndSpeedManager.updateMotorSpeed();
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method converts feedback sensor inputs from their raw sensor form
//  to the expected engineering units.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::convertSensorInputs()
{
    //
    //  We convert each raw ADC input to the expected "real" unit.
    //
    phaseACurrent = ADCConversions::convertPhaseCurrent(dataFromADC.currentA);
    phaseBCurrent = ADCConversions::convertPhaseCurrent(dataFromADC.currentB);
    phaseCCurrent = ADCConversions::convertPhaseCurrent(dataFromADC.currentC);

    phaseAVoltage = ADCConversions::convertPhaseVoltage(dataFromADC.voltageA);
    phaseBVoltage = ADCConversions::convertPhaseVoltage(dataFromADC.voltageB);
    phaseCVoltage = ADCConversions::convertPhaseVoltage(dataFromADC.voltageC);

    busCurrent    = ADCConversions::convertBusCurrent(dataFromADC.busCurrent);
    busVoltage    = ADCConversions::convertBusVoltage(dataFromADC.busVoltage);

    //
    //  For now, we leave the analog hall sensor data in ADC scaling.
    //
    analogHallA = static_cast<float32_t>(rawHallA);
    analogHallB = static_cast<float32_t>(rawHallB);
    analogHallC = static_cast<float32_t>(rawHallC);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method closes the motor current loops.  This method sends the commands
//  to the motor control electronics.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::controlMotorCurrents()
{
    //
    //  We adjust the phase current feedback for the offsets computed when we first
    //  started the motor.
    //
    if (MotorConstants::computeCurrentSensorOffsets)
    {
        phaseACurrent = phaseACurrent - phaseACurrentOffset.getFilterState();
        phaseBCurrent = phaseBCurrent - phaseBCurrentOffset.getFilterState();
        phaseCCurrent = phaseCCurrent - phaseCCurrentOffset.getFilterState();
    }

    //
    //  Now we execute the series of transformations that convert the feedback
    //  currents from the stationary ABC frame to the rotating DQ vector components
    //  required by the current controllers.  Please see motor_transforms.h
    //  for a discussion of the reference frames.
    //
    theMotorTransforms.provideRotatingFrameAngle(feedbackShaftAngle);

    theMotorTransforms.performReducedClarkeTransformation(
        phaseACurrent,
        phaseBCurrent,
        feedbackCurrent_Alpha,
        feedbackCurrent_Beta);

    theMotorTransforms.performParkTransformation(
        feedbackCurrent_Alpha,
        feedbackCurrent_Beta,
        feedbackCurrent_D,
        feedbackCurrent_Q);

    //
    //  We can now update the current controllers.
    //
    IDController.setPIControllerTarget(IDTargetCommand);
    IQController.setPIControllerTarget(IQTargetCommand);

    IDController.updatePIController(feedbackCurrent_D, IDCommandOutput);
    IQController.updatePIController(feedbackCurrent_Q, IQCommandOutput);

    //
    //  The controllers output duty cycle commands for the motor.  The duty
    //  cycle commands start out as rotation frame DQ vector components. We must
    //  transform  them back to stationary frame ABC values before we can send
    //  them to the bridge hardware.
    //
    theMotorTransforms.performInverseParkTransformation(
        IDCommandOutput,
        IQCommandOutput,
        currentCommand_Alpha,
        currentCommand_Beta);

    theMotorTransforms.performInverseClarkeTransformation(
        currentCommand_Alpha,
        currentCommand_Beta,
        nonModulatedDutyA,
        nonModulatedDutyB,
        nonModulatedDutyC);

    //
    //  We have three duty cycle commands in the proper reference frame.  We
    //  now invoke a modulation function to attempt to best utilize the available
    //  DC bus.
    //
    theZeroSequenceModulator.performZeroSequenceModulation(
        nonModulatedDutyA,
        nonModulatedDutyB,
        nonModulatedDutyC,
        outputDutyA,
        outputDutyB,
        outputDutyC);

    //
    //  We can finally send the new duty cycle commands to the motor drive
    //  hardware.
    //
    thePWMInterface.setPWMDutyCycles(
        outputDutyA,
        outputDutyB,
        outputDutyC);

    //
    //  We make sure we are not risking damage to the motor.   Note
    //  this will only work while the PWMs are running.  The motor
    //  currents look really large when the motor is disabled.
    //
    monitorMotorCurrents();
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method shuts down the motor when either commanded to do so or when an
//  error is detected.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::shutDownMotor()
{
    GPIOInterface::disableBridgeOutput();
    thePWMInterface.disablePWMOutputs();
    IDTargetCommand = 0.0F;
    IQTargetCommand = 0.0F;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method monitors the feedback phase currents and declares an error if
//  any of them are out of an expected range.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::monitorMotorCurrents()
{
    //
    //  We check each phase.  Remember the phase currents are signed.  Note
    //  that the PWMs must be enabled or this will always trigger as the
    //  currents look quite large when the PWMs are low.
    //
    if ((fabsf(phaseACurrent) > MotorConstants::phaseOverCurrentThreshold) ||
        (fabsf(phaseBCurrent) > MotorConstants::phaseOverCurrentThreshold) ||
        (fabsf(phaseCCurrent) > MotorConstants::phaseOverCurrentThreshold))
    {
        errorDetected = true;
        shutDownMotor();
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method implements an auto-shutdown feature.  When enabled, this allows
//  the motor to run a fixed number of frames and then shuts it down.  This is useful
//  during development to test changes that could put the system hardware at
//  risk.  For example, when confirming the signs of the phase current feedback
//  it can be useful to only allow the drive to operate long enough to ensure
//  the signs are correct.  If the signs were incorrect, the auto-shutdown could
//  stop the controller before the positive feedback results in overly large
//  current commands.
//
////////////////////////////////////////////////////////////////////////////////
void motorStateMachine::monitorAutoShutdown()
{
    //
    //  If the counter is set to a non-zero value, that indicates an auto-
    //  shutdown is desired.
    //
    if (autoShutDownCounter != MotorConstants::dontAutoShutdown)
    {
        autoShutDownCounter--;
        if (autoShutDownCounter == 0U)
        {
            stopCommanded = true;
        }
    }
}
