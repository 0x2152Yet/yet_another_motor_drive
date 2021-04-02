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
//  This class computes the motor shaft angle and speed.  The angle may
//  be computed in three different ways:
//    1)  The shaft angle may be computed using a shaft encoder (for example
//        analog hall sensors)
//    2)  The shaft angle may be estimated using a sensor-less estimator.
//    3)  The motor angle and speed may be simulated using a ramp pattern that
//        simulates a motor that starts and then accelerates to a given speed.
//        This may be used to start a sensor-less drive
//
//  Both a measured and estimated shaft angle may be computed concurrently.
//  However, only the angle-type selected with MotorConstants::currentControlAngleSelection
//  is used to compute the actual motor speed
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "data_logger.h"
#include "global_definitions.h"
#include "low_pass_filter.h"
#include "motor_transforms.h"
#include "pll_angle_est.h"
#include "smo_angle_est.h"

class motorAngleAndSpeed
{
public:
    motorAngleAndSpeed();
    ~motorAngleAndSpeed() {};

    //
    //  These methods provide the present motor shaft angle for each of the
    //  methods available.
    //
    electricalAngle_rad getPresentAnalogHallAngle() const { return presentAnalogHallAngle; }
    electricalAngle_rad getPresentEstimatedAngle()  const { return presentEstimatedAngle; }

    //
    //  These provide both the most recently computed speed and a filtered
    //  speed.
    //
    electricalSpeed_RPM getPresentSpeed()  const { return presentSpeed; }
    electricalSpeed_RPM getFilteredSpeed() const { return filteredSpeed.getFilterState(); }

    //
    //  This initializes the actual motor speed calculation.  It is provided
    //  two frequencies:  the frequency at which updated motor angle data is
    //  computed (this is usually the drive's update frequency) and the frequency
    //  at which the speed should be updated.  It is common to update the motor
    //  speed data at a lower rate than the full drive update rate.
    //
    //  Note that the speed update is downsampled from the angle update.  The
    //  actual update rate will be integral result of the shaftAngleUpdateFrequency
    //  divided by the speedUpdateFrequency.  For example, if the angle update
    //  frequency is 100 Hz and a speed update frequency of 30 Hz is provided,
    //  the actual update frequency will be 33.33 Hz:
    //     100U / 30U => 3U
    //     A speed update will occur every 3rd angle update => 33.33 Hz
    //
    void initializeSpeedCalculation(
        const frequency_Hz shaftAngleUpdateFrequency,
        const frequency_Hz speedUpdateFrequency);

    //
    //  This updates the motor speed.  This should be called every motor drive
    //  update.  It will handle any necessary downsampling internally.  The
    //  angle used in the speed calculation is determined by the constant
    //  MotorConstants::currentControlAngleSelection.
    //
    void updateMotorSpeed();

    //
    //  This indicates if the last update to the motor speed computed a new
    //  value.
    //
    bool newSpeedAvailable() const { return (speedUpdateCounter == 0U); }

    //
    //  This updates the motor angle when analog hall sensors are used
    //  as a shaft encoder.  The three sensor values are the inputs
    //  from the analog hall sensors.
    //
    void updateAnalogHallEncoderBasedAngle(
        const float32_t hallSensorA,
        const float32_t hallSensorB,
        const float32_t hallSensorC);

    //
    //  The following methods support estimating the motor shaft angle
    //  without a shaft encoder (sensor-less estimation). The first method,
    //  computeSensorlessEstimatorCoefficients must be called at least once
    //  before updateSensorlessAngle is invoked.
    //
    void computeSensorlessEstimatorCoefficients(
        const resistance_Ohms statorResistance,
        const inductance_H statorInductance,
        const voltsPer1000MechanicalRPM motorBackEMFConstant,
        const float32_t motorPolePairs,
        const frequency_Hz sampleFrequency);

    //
    //  This computes an estimated motor shaft angle. The sensor-less angle
    //  estimation requires the most recent bus voltage feedback, current sensor
    //  feedback and the last duty cycles that were commanded to the bridge driver.
    //
    void updateSensorlessAngle(
        const volts busVoltage,
        const float32_t feedbackCurrentAlpha,
        const float32_t feedbackCurrentBeta,
        const dutyCycle_pct phaseACommandedDutyCycle,
        const dutyCycle_pct phaseBCommandedDutyCycle,
        const dutyCycle_pct phaseCCommandedDutyCycle);

    //
    //  This is called to generate the simulated motor angle and speed necessary
    //  to perform an open-loop start-up.
    //
    //  Calling startSpeedRamp will simulate a motor that starts moving from a
    //  commanded starting speed and accelerates or decelerates to a final
    //  target speed.  The parameters should be set as follows:
    //    initialAngle provides the angle to seed the generator with.  For
    //      a stationary motor this would usually be zero.  If the motor is
    //      already moving, the motor's present angle would be used.
    //    initialSpeed provides the speed to seed the start the generator
    //      with.  For a stationary motor, this would usually be zero.  for a motor
    //      that is already moving, this would be the motor's present speed.
    //    finalSpeed provides the speed at which the generator should
    //      should stop changing.  Note that the final speed may have a different
    //      sign than the initial speed (i.e. the motor should change directions).
    //    rampSpeedAcceleration provides the rate at which the ramp speed should
    //      accelerate in rotations/seconds-squared.
    //    callingFrequency provides the frequency at which updateSpeedRamp
    //      will be called
    //
    void startSpeedRamp(
        const electricalAngle_rad initialAngle,
        const electricalSpeed_RPM initialSpeed,
        const electricalSpeed_RPM finalSpeed,
        const acceleration_RotationsPerSec2 rampSpeedAcceleration,
        const frequency_Hz callingFrequency);

    //
    //  This updates the simulated angle/speed generator.  It should be called
    //  at the updateFrequency provided when the startSpeedRamp method was
    //  called.
    //
    void updateSpeedRamp();

    //
    //  These provide the present simulated speed and angle data when the
    //  open-loop ramp-to-speed is being performed.
    //
    electricalAngle_rad getPresentRampAngle() const { return presentRampAngle; }
    electricalSpeed_RPM getPresentRampSpeed() const { return presentRampSpeed; }

    //
    //  This indicates when the simulated open-loop speed has reached the
    //  commanded target.
    //
    bool speedRampComplete() const { return rampComplete; }

    //
    //  This provides the most recently commanded open-loop ramp target speed.
    //
    electricalSpeed_RPM getPresentRampSpeedTarget() const { return rampTargetSpeed; }

    //
    //  This is a helper method that adds an offset to an angle.  This handles
    //  the case where adding the offset may cause the angle to cross the zero
    //  angle rollover (i.e. the 0.00/6.28 radian rollover).
    //
    static electricalAngle_rad offsetShaftAngle(
        const electricalAngle_rad originalAngle,
        const electricalAngle_rad angleOffset);

private:

    //
    //  These track the angle and speed data computed by this
    //  package.  We may be configured to compute the motor angle
    //  in multiple ways.  Only one type of angle is used to determine the
    //  motor's actual speed.
    //
    electricalAngle_rad presentRampAngle;
    electricalSpeed_RPM presentRampSpeed;
    electricalAngle_rad presentAnalogHallAngle;
    electricalAngle_rad presentEstimatedAngle;
    electricalSpeed_RPM presentSpeed;
    lowPassFilter       filteredSpeed;

    //
    //  These are intermediate values created when the analog hall
    //  sensors are converted to a shaft angle.
    //
    float32_t analogHallX;
    float32_t analogHallY;

    //
    //  These are the parameters associated with the present ramp-based angle
    //  generation activity.
    //
    electricalAngle_rad     angleStep;
    electricalAngle_rad     finalAngleStep;
    electricalSpeed_RPM     rampTargetSpeed;
    float32_t               rampAngleStepChangePerUpdate;
    float32_t               rampAngleStepToSpeedRPMGain;
    bool                    rampComplete;

    //
    //  These are used for tracking the motor's speed.
    //
    bool                firstSpeedUpdate;
    uint32_t            speedUpdateDownsample;
    uint32_t            speedUpdateCounter;
    float32_t           deltaAngleToSpeedRPSGain;
    electricalAngle_rad lastSpeedAngle;

    //
    //  These are the estimators that may be used to estimate the motor's
    //  shaft angle without an encoder.
    //
    slidingModeObserverAngleEstimator smoAngleEstimator;
    PLLAngleEstimator                 pllAngleEstimator;

    //
    //  The sensor-less angle estimators utilize estimated motor phase
    //  voltages.
    //
    void estimatePhaseVoltages(
        const volts busVoltage,
        const dutyCycle_pct phaseACommandedDutyCycle,
        const dutyCycle_pct phaseBCommandedDutyCycle,
        const dutyCycle_pct phaseCCommandedDutyCycle);

    motorTransforms voltageTransforms;
    volts estimatedPhaseAVoltage;
    volts estimatedPhaseBVoltage;
    volts estimatedPhaseCVoltage;
    float32_t estimatedPhaseVoltageAlpha;
    float32_t estimatedPhaseVoltageBeta;

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;

};
