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
//  Defines the interface to the processor's PWM generator
//
//  This class controls a fairly "textbook" BLDC motor drive center-based
//  PWM.  It supports 3 complimentary outputs with a blanking period (AKA dead
//  band).  In addition it generates a signal to start ADC sampling at the center
//  of the PWM  pulse-train.  It is up to other code to configure the IO pins, etc.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"
#include "processor.h"

namespace PWMConsts
{
    //
    //  This defines the absolute range of the duty cycle commands.  Duty
    //  cycles are provided as a percentage.  For example if a 30% duty cycle
    //  value is desired, a value of 0.3 should be provided.
    //
    const dutyCycle_pct minDutyCycleCommand = 0.0F;
    const dutyCycle_pct maxDutyCycleCommand = 1.0F;

    //
	//  This is the nominal duty cycle that is assumed to result in a phase
	//  current that is close to 0 amps.
	//
	const dutyCycle_pct nominalDutyCycle = 0.5F;
}

class PWMInterface
{
public:
	PWMInterface();
	~PWMInterface() {};

	//
	//  This prepares the PWM interface for operation.  The PWM outputs
	//  are disabled when this returns.  The PWM limit is defined as a
	//  percentage from 0.0 to 1.0.  This limit is applied to both the
	//  maximum and minimum duty cycle.  For example, if a limit of 0.1 (10%)
	//  is provided, the duty cycle commands will be allowed to range from
	//  0.1 to 0.9 (10% to 90%).
	//
    void initializePWMInterface(
        const frequency_Hz desiredPWMCarrierFrequency,
        const time_s PWMBlankingPeriod,
        const dutyCycle_pct dutyCycleLimit_pct);

	//
	//  This disables the PWM output.  All signals are forced low.
	//
	void disablePWMOutputs();

	//
	//  This enables the PWM output.  If the nominal value is not desired, the
	//  duty cycle should be
	//  duty cycle.
	//
	void enablePWMOutputs(
		const dutyCycle_pct initialDutyCycleA = PWMConsts::nominalDutyCycle,
		const dutyCycle_pct initialDutyCycleB = PWMConsts::nominalDutyCycle,
		const dutyCycle_pct initialDutyCycleC = PWMConsts::nominalDutyCycle);

	//
	//  This commands a duty cycle for each PWM phase.  Each duty cycle
	//  is provided as a percentage.  The nominal is 0.0 to 1.0 (0% to
	//  100%).  The dutyCycleLimit_pct provided when the interface was
	//  initialized will be enforced.
	//
	void setPWMDutyCycles(
		const dutyCycle_pct desiredDutyCycleA,
		const dutyCycle_pct desiredDutyCycleB,
		const dutyCycle_pct desiredDutyCycleC);

	//
	//  These provide access to the PWM state and the last commanded duty cycles.
	//
	bool thePWMsAreEnabled()            { return pwmsAreEnabled; }
	dutyCycle_pct getLastDutyACommand() { return lastDutyACommand; }
	dutyCycle_pct getLastDutyBCommand() { return lastDutyBCommand; }
	dutyCycle_pct getLastDutyCCommand() { return lastDutyCCommand; }

private:

	//
	//  These latch the PWM parameters after they have been converted from
	//  the units provided in the interface to the count values required by
	//  the hardware.
	//
	bool      pwmsAreEnabled;
	float32_t dutyPctToCountsConversion;
	uint32_t  nominalDutyCommand_counts;
	uint32_t  minDutyCommand_counts;
	uint32_t  maxDutyCommand_counts;

	//
	//  We latch the last PWM command as a convenience for the consumer.
	//
	dutyCycle_pct lastDutyACommand;
	dutyCycle_pct lastDutyBCommand;
	dutyCycle_pct lastDutyCCommand;
};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern PWMInterface thePWMInterface;

