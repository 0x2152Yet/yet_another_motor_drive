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
//  Provides an interface to the processor's PWM generator
//
//  This class controls the PWM outputs that are used to drive the motor.
//  We create a three phase center-based PWM with a blanking period (AKA
//  deadband).  Both sides of the complimentary PWM pair are created along
//  with an ADC start-of-conversion command that is aligned with the center
//  of the PWMs.  Any discrete GPIOs used to enable or reset a motor drive are
//  controlled elsewhere.
//
////////////////////////////////////////////////////////////////////////////////

#include "global_definitions.h"
#include "hw_definitions.h"
#include "processor.h"
#include "pwm_interface.h"

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
PWMInterface::PWMInterface() :
    pwmsAreEnabled(false),
    dutyPctToCountsConversion(0.0F),
    nominalDutyCommand_counts(0.0F),
    minDutyCommand_counts(0.0F),
    maxDutyCommand_counts(0.0F),
    lastDutyACommand(0.0F),
    lastDutyBCommand(0.0F),
    lastDutyCCommand(0.0F)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes the PWM interface.  This prepares the PWM interface and
//  computes the values that will be used to convert commands from
//  engineering units to the appropriate form required by the hardware.
//
////////////////////////////////////////////////////////////////////////////////
void PWMInterface::initializePWMInterface(
    const frequency_Hz desiredPWMCarrierFrequency,
    const time_s PWMBlankingPeriod,
    const dutyCycle_pct dutyCycleLimit_pct)
{
    //
    //  Now we compute the various values we will used to initialize the
    //  timer and at runtime when the process PWM commands.
    //
    const float32_t totalCountsForPWMPeriod =
        static_cast<float32_t>(SystemCoreClock) / desiredPWMCarrierFrequency;

    //
    //  Remember that we are creating a center-based PWM.  In this case,
    //  the timer counts from 0 to a midpoint and then turns around and
    //  counts back to zero.  To get the desired duty cycle we command
    //  a timer value that is centered around timer zero. The PWM will change
    //  state when the timer's counter matches the command value both on the way
    //  up to from zero to the midpoint and then on the way down from the
    //  midpoint back to zero.  For example:  if the timer turns around at 500
    //  counts a command value of 250 will result in a 50% duty cycle.
    //
    const float32_t timerMidpoint = totalCountsForPWMPeriod / 2.0F;
    const uint32_t timerMidpoint_counts = static_cast<uint32_t>(timerMidpoint + 0.5F);

    dutyPctToCountsConversion = timerMidpoint;
    nominalDutyCommand_counts =
        static_cast<uint32_t>((PWMConsts::nominalDutyCycle * dutyPctToCountsConversion) + 0.5F);

    //
    //  We limit the duty cycle based on the commanded values.  We will never
    //  let it go out of the range of 0 to 100%.
    //
    minDutyCommand_counts = 0U;
    maxDutyCommand_counts = static_cast<uint32_t>(timerMidpoint);

    if ((dutyCycleLimit_pct > 0.0F) && (dutyCycleLimit_pct < 1.0F))
    {
        minDutyCommand_counts =
            static_cast<uint32_t>((dutyCycleLimit_pct * dutyPctToCountsConversion) + 0.5F);
        maxDutyCommand_counts =
            static_cast<uint32_t>(((1.0F - dutyCycleLimit_pct) * dutyPctToCountsConversion) + 0.5F);
    }

    //
    //  We compute the PWM blanking setting.  This is also specified in timer
    //  counts.
    //
    const float32_t PWMBlankingSetting =
        static_cast<float32_t>(SystemCoreClock) * PWMBlankingPeriod;

    //
    //  We must limit the blanking setting to fit in the hardware's
    //  restrictions.
    //
    uint32_t blankingCounts;
    const float32_t minBlankingSetting = 0.0F;
    const float32_t maxBlankingSetting = 255.0F;

    if (PWMBlankingSetting <= minBlankingSetting)
    {
        blankingCounts = static_cast<uint8_t>(minBlankingSetting);
    }
    else if (PWMBlankingSetting >= maxBlankingSetting)
    {
        blankingCounts = static_cast<uint8_t>(maxBlankingSetting);
    }
    else
    {
        blankingCounts = static_cast<uint8_t>(PWMBlankingSetting + 0.5F);
    }

    //
    //  Our blanking setting forces a limit onto our PWM limits.  They
    //  may not fall within the blanking period.  If the do, the PWMs will not
    //  switch.
    //
    if (minDutyCommand_counts <= blankingCounts)
    {
        minDutyCommand_counts = blankingCounts + 1U;
    }

    if (maxDutyCommand_counts <= blankingCounts)
    {
        maxDutyCommand_counts = blankingCounts + 1;
    }

    //
    //  Now we can configure the timer.  We start with the basic timer.  We
    //  configure it to count up from zero to a midpoint and then back down
    //  to zero.
    //
    LL_TIM_InitTypeDef timerInitData;
    LL_TIM_StructInit(&timerInitData);

    timerInitData.Prescaler = 0U;
    timerInitData.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;
    timerInitData.Autoreload = timerMidpoint_counts;
    timerInitData.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    timerInitData.RepetitionCounter = 0U;

    LL_TIM_Init(TimerDefinitions::PWMTimer, &timerInitData);

    //
    //  We set up the blanking period.  There are other items in this configuration
    //  that we presently do no care about.
    //
    LL_TIM_BDTR_InitTypeDef blankingInitData;
    LL_TIM_BDTR_StructInit(&blankingInitData);

    blankingInitData.OSSRState = LL_TIM_OSSR_ENABLE;
    blankingInitData.OSSIState = LL_TIM_OSSI_ENABLE;
    blankingInitData.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    blankingInitData.DeadTime = static_cast<uint8_t>(blankingCounts);
    blankingInitData.BreakState = LL_TIM_BREAK_ENABLE;
    blankingInitData.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    blankingInitData.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;

    LL_TIM_BDTR_Init(TimerDefinitions::PWMTimer, &blankingInitData);

    //
    //  We enable timer-level compare capture preload.
    //
    LL_TIM_CC_EnablePreload(TimerDefinitions::PWMTimer);

    //
    //  Now we configure the timer output compare channels.  These create the
    //  PWMs.  Each is configured to output a complimentary PWM where the
    //  active time is centered around the timer zero.
    //
    LL_TIM_OC_InitTypeDef ocInitData;
    LL_TIM_OC_StructInit(&ocInitData);

    ocInitData.OCMode = LL_TIM_OCMODE_PWM1;
    ocInitData.OCState = LL_TIM_OCSTATE_DISABLE;
    ocInitData.OCNState = LL_TIM_OCSTATE_DISABLE;
    ocInitData.CompareValue = nominalDutyCommand_counts;
    ocInitData.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    ocInitData.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    ocInitData.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    ocInitData.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init(TimerDefinitions::PWMTimer, LL_TIM_CHANNEL_CH1, &ocInitData);
    LL_TIM_OC_Init(TimerDefinitions::PWMTimer, LL_TIM_CHANNEL_CH2, &ocInitData);
    LL_TIM_OC_Init(TimerDefinitions::PWMTimer, LL_TIM_CHANNEL_CH3, &ocInitData);

    LL_TIM_OC_EnablePreload(TimerDefinitions::PWMTimer, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TimerDefinitions::PWMTimer, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TimerDefinitions::PWMTimer, LL_TIM_CHANNEL_CH3);

    //
    //  We use another timer to trigger the ADC.  We set this up just like the
    //  PWM timer, except we never change its configuration.  We want the ADC
    //  to sample data in the center of the PWM (timer zero).  We leave this
    //  channel running always.  That way the ADC will sample even when the PWMs
    //  are not active.
    //
    //  Note the setting of this timer allows for a certain amount of tuning
    //  about where the ADC will sample.  A few notes:
    //    1) This timer runs at half the speed of the PWM timer so we must
    //       adjust its reset value accordingly.
    //    2) We command it to generate a PWM.  The rising edge of the PWM will
    //       trigger the ADC.
    //    3) If we set the PWM to a very small duty cycle, then the ADC
    //       trigger will occur just before the midpoint of the PWM.
    //    4) As we increase the duty cycle, the ADC trigger will move toward
    //       the "front" of the PWM.
    //    5) If we want to move the trigger point behind the PWM mid-point
    //       we would need to configure the timer to use PWM mode 2.
    //
    //  Important note:  with the present hardware, the ADC trigger must occur
    //  during the PWM's low period.  That is the only time the current feedback
    //  data is valid.  With our present setup, the low portion of the PWM is
    //  centered around the timer's turn-around point (PWM mode 1 in the timer
    //  parlance).
    //
    //  We choose our trigger point such that it is be between the PWM's falling
    //  edge and its midpoint when the PWM has its shortest low-time.  We have
    //  tuned this value to allow for a wide range of PWMs will still giving the
    //  ADC time to sample its data.  Remember in the following that we must take
    //  into account that the actual PWM timer is running at twice the frequency
    //  of the trigger timer.
    //
    const uint32_t ADCTriggerTimerMidpoint = timerMidpoint_counts / 2U;
    const uint32_t ADCTriggerOffset        = (minDutyCommand_counts * 2U) / 4U;
    const uint32_t ADCTriggerSOCPoint      = ADCTriggerTimerMidpoint - ADCTriggerOffset;

    timerInitData.Autoreload = ADCTriggerTimerMidpoint;
    LL_TIM_Init(TimerDefinitions::ADCTimer, &timerInitData);

    ocInitData.OCState = LL_TIM_OCSTATE_ENABLE;
    ocInitData.CompareValue = ADCTriggerSOCPoint;
    LL_TIM_OC_Init(TimerDefinitions::ADCTimer, LL_TIM_CHANNEL_CH1, &ocInitData);
    LL_TIM_OC_EnablePreload(TimerDefinitions::ADCTimer, LL_TIM_CHANNEL_CH1);

    //
    //  We configure the ADC timer to generate its trigger output.  The ADC
    //  will be configured to start converting data when this trigger occurs.
    //
    LL_TIM_SetTriggerOutput(TimerDefinitions::ADCTimer, LL_TIM_TRGO_OC1REF);

    //
    //  The ADC trigger timer is always enabled with a fixed duty cycle (see
    //  the note, above).
    //
    TimerDefinitions::ADCTimer->CCR1 = ADCTriggerSOCPoint;
    LL_TIM_CC_EnableChannel(TimerDefinitions::ADCTimer, LL_TIM_CHANNEL_CH1);

    //
    //  Finally, we enable everything.  We leave the PWM outputs off
    //  for now.  We start with commutation events so that all settings
    //  are loaded.
    //
    LL_TIM_GenerateEvent_COM(TimerDefinitions::PWMTimer);
    LL_TIM_GenerateEvent_COM(TimerDefinitions::ADCTimer);

    LL_TIM_EnableAllOutputs(TimerDefinitions::PWMTimer);
    LL_TIM_EnableAllOutputs(TimerDefinitions::ADCTimer);

    LL_TIM_EnableCounter(TimerDefinitions::ADCTimer);
    LL_TIM_EnableCounter(TimerDefinitions::PWMTimer);

    pwmsAreEnabled = false;
    lastDutyACommand = PWMConsts::nominalDutyCycle;
    lastDutyBCommand = PWMConsts::nominalDutyCycle;
    lastDutyCCommand = PWMConsts::nominalDutyCycle;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Disables the PWM outputs.  All PWM signals are driven low.
//
////////////////////////////////////////////////////////////////////////////////
void PWMInterface::disablePWMOutputs()
{
    const uint32_t DisableAllMotorPWMChannels =
        (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
         LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
         LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

    LL_TIM_CC_DisableChannel(TimerDefinitions::PWMTimer, DisableAllMotorPWMChannels);

    LL_TIM_GenerateEvent_COM(TimerDefinitions::PWMTimer);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Enables the PWM outputs.  Initial values for the PWM duty cycles may be provided.
//
////////////////////////////////////////////////////////////////////////////////
void PWMInterface::enablePWMOutputs(
    const dutyCycle_pct initialDutyCycleA,
    const dutyCycle_pct initialDutyCycleB,
    const dutyCycle_pct initialDutyCycleC)
{
    //
    //  We command the initial duty cycles before we start the PWMs.
    //
    setPWMDutyCycles(initialDutyCycleA, initialDutyCycleB, initialDutyCycleC);

    //
    //  Now we enable the PWM outputs.
    const uint32_t EnableAllMotorPWMChannels =
        (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
         LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
         LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

    LL_TIM_CC_EnableChannel(TimerDefinitions::PWMTimer, EnableAllMotorPWMChannels);

    LL_TIM_GenerateEvent_COM(TimerDefinitions::PWMTimer);

}

////////////////////////////////////////////////////////////////////////////////
//
//  Sets the desired duty cycle for each PWM output.  The duty cycles are
//  provided as a percentage in the range PWMConsts::minDutyCycleCommand to
//  PWMConsts::maxDutyCycleCommand.
//
////////////////////////////////////////////////////////////////////////////////
void PWMInterface::setPWMDutyCycles(
    const dutyCycle_pct desiredDutyCycleA,
    const dutyCycle_pct desiredDutyCycleB,
    const dutyCycle_pct desiredDutyCycleC)
{
    //
    //  First, we convert each percentage to the equivalent timer counts.
    //
    uint32_t dutyACommandCounts =
        static_cast<uint32_t>((desiredDutyCycleA * dutyPctToCountsConversion) + 0.5F);
    uint32_t dutyBCommandCounts =
        static_cast<uint32_t>((desiredDutyCycleB * dutyPctToCountsConversion) + 0.5F);
    uint32_t dutyCCommandCounts =
        static_cast<uint32_t>((desiredDutyCycleC * dutyPctToCountsConversion) + 0.5F);

    //
    //  Now we apply the limits.
    //
    if (dutyACommandCounts < minDutyCommand_counts)
    {
        dutyACommandCounts = minDutyCommand_counts;
    }
    else if (dutyACommandCounts > maxDutyCommand_counts)
    {
        dutyACommandCounts = maxDutyCommand_counts;
    }

    if (dutyBCommandCounts < minDutyCommand_counts)
    {
        dutyBCommandCounts = minDutyCommand_counts;
    }
    else if (dutyBCommandCounts > maxDutyCommand_counts)
    {
        dutyBCommandCounts = maxDutyCommand_counts;
    }

    if (dutyCCommandCounts < minDutyCommand_counts)
    {
        dutyCCommandCounts = minDutyCommand_counts;
    }
    else if (dutyCCommandCounts > maxDutyCommand_counts)
    {
        dutyCCommandCounts = maxDutyCommand_counts;
    }

    //
    //  We can now write the new commands to the timer.  Note that in code
    //  that may be called at a high rate, we do not use the HAL driver.
    //
    TimerDefinitions::PWMTimer->CCR1 = dutyACommandCounts;
    TimerDefinitions::PWMTimer->CCR2 = dutyBCommandCounts;
    TimerDefinitions::PWMTimer->CCR3 = dutyCCommandCounts;

    LL_TIM_GenerateEvent_COM(TimerDefinitions::PWMTimer);
}

