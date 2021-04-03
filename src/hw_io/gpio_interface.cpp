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
//  Provides an interface to the processor's GPIOs
//
//  This class prepares the GPIOs for operation and provides the interfaces
//  to control them.
//
////////////////////////////////////////////////////////////////////////////////
#include "config.h"
#include "gpio_interface.h"
#include "hw_definitions.h"
#include "processor.h"

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
GPIOInterface::GPIOInterface()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Performs initializations required before GPIOs may be used
//
////////////////////////////////////////////////////////////////////////////////
void GPIOInterface::initializeGPIOInterface()
{
    //
    //  We initialize each type of GPIO that is used.
    //
    initializeDigitalInputs();
    initializeDigitalOutputs();
    initializeAnalogPins();
    initializeTimerPins();
    initializeUARTPins();
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes pins that are used as discrete digital outputs
//
////////////////////////////////////////////////////////////////////////////////
void GPIOInterface::initializeDigitalOutputs()
{
    using namespace GPIODefinitions;

    LL_GPIO_InitTypeDef initData;

    LL_GPIO_StructInit(&initData);

    initData.Mode       = LL_GPIO_MODE_OUTPUT;
    initData.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
    initData.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    initData.Pull       = LL_GPIO_PULL_NO;
    initData.Alternate  = LL_GPIO_AF_0;

    //
    //  We set the state of each GPIO before we set its mode to keep it from
    //  glitching.  We start with the pins that enable/reset each bridge
    //  driver phase.  If another processor is controlling the motor (i.e.
    //  we are snooping its outputs) we do not set our motor control signals
    //  as outputs.
    //
    if (!(SystemConfig::snoopRemoteMotorController))
    {
        disableBridgeOutput();
        initData.Pin = phaseAControlPin;
        LL_GPIO_Init(bridgeDriverControlPort1, &initData);
        initData.Pin = phaseBControlPin;
        LL_GPIO_Init(bridgeDriverControlPort1, &initData);
        initData.Pin = phaseCControlPin;
        LL_GPIO_Init(bridgeDriverControlPort2, &initData);
    }

    //
    //  These control the status LEDs.
    //
    turnStatusLEDOff();
    initData.Pin = statusLEDPin;
    LL_GPIO_Init(statusLEDPort, &initData);

    turnTimingLEDOff();
    initData.Pin = timingLEDPin;
    LL_GPIO_Init(timingLEDPort, &initData);

    turnTriggerLEDOff();
    initData.Pin = triggerLEDPin;
    LL_GPIO_Init(triggerLEDPort, &initData);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes pins that are used a discrete digital inputs
//
////////////////////////////////////////////////////////////////////////////////
void GPIOInterface::initializeDigitalInputs()
{
    using namespace GPIODefinitions;

    LL_GPIO_InitTypeDef initData;

    LL_GPIO_StructInit(&initData);

    initData.Mode       = LL_GPIO_MODE_INPUT;
    initData.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
    initData.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    initData.Pull       = LL_GPIO_PULL_UP;
    initData.Alternate  = LL_GPIO_AF_0;

    //
    //  This pin provides an external trigger input for the ADC.
    //
    initData.Pin = externalADCTriggerPin;
    LL_GPIO_Init(externalADCTriggerPort, &initData);

    //
    //  These are the physical push-buttons.
    //
    initData.Pull = LL_GPIO_PULL_DOWN;
    initData.Pin = onBoardpushButtonPin;
    LL_GPIO_Init(onBoardPushButtonPort, &initData);

    initData.Pull = LL_GPIO_PULL_UP;
    initData.Pin = externalPushButtonPin;
    LL_GPIO_Init(externalPushButtonPort, &initData);

    //
    //  If we are monitoring the outputs of another motor controller, we do
    //  not drive our usual motor control outputs.  For now this includes
    //  the PWM outputs and the bridge driver phase enables.
    //
    if (SystemConfig::snoopRemoteMotorController)
    {
        initData.Pull = LL_GPIO_PULL_NO;

        initData.Pin = chan1NPin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan1Pin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan2NPin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan2Pin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan3NPin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan3Pin;
        LL_GPIO_Init(pwmOutputPort, &initData);

        initData.Pin = phaseAControlPin;
        LL_GPIO_Init(bridgeDriverControlPort1, &initData);
        initData.Pin = phaseBControlPin;
        LL_GPIO_Init(bridgeDriverControlPort1, &initData);
        initData.Pin = phaseCControlPin;
        LL_GPIO_Init(bridgeDriverControlPort2, &initData);
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes pins associated with analog inputs and outputs.  This
//  includes the ADCs and the DACs
//
////////////////////////////////////////////////////////////////////////////////
void GPIOInterface::initializeAnalogPins()
{
    using namespace GPIODefinitions;

    LL_GPIO_InitTypeDef initData;

    LL_GPIO_StructInit(&initData);

    initData.Mode       = LL_GPIO_MODE_ANALOG;
    initData.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
    initData.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    initData.Pull       = LL_GPIO_PULL_NO;
    initData.Alternate  = analogAlternateFunction;

    //
    //  We configure each ADC pin.  Note that they tend to be grouped together.
    //  Some of the ADC inputs are not presently used as the pins conflict
    //  with other hardware on the target.  Removing solder bridges from the
    //  target can make these inputs available.
    //
    initData.Pin = ADC123channel0Pin;
    LL_GPIO_Init(ADCInputPortGroup1, &initData);
    initData.Pin = ADC123channel1Pin;
    LL_GPIO_Init(ADCInputPortGroup1, &initData);
    initData.Pin = ADC123channel2Pin;
    LL_GPIO_Init(ADCInputPortGroup1, &initData);
    initData.Pin = ADC123channel3Pin;
    LL_GPIO_Init(ADCInputPortGroup1, &initData);

    initData.Pin = ADC3channel4Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);
    initData.Pin = ADC3channel5Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);
    initData.Pin = ADC3channel6Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);
    initData.Pin = ADC3channel7Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);
    initData.Pin = ADC3channel8Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);
    initData.Pin = ADC3channel9Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);
    initData.Pin = ADC3channel14Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);
    initData.Pin = ADC3channel15Pin;
    LL_GPIO_Init(ADCInputPortGroup2, &initData);

    initData.Pin = ADC123channel10Pin;
    LL_GPIO_Init(ADCInputPortGroup3, &initData);
    initData.Pin = ADC123channel11Pin;
    LL_GPIO_Init(ADCInputPortGroup3, &initData);
    initData.Pin = ADC123channel12Pin;
    LL_GPIO_Init(ADCInputPortGroup3, &initData);
    initData.Pin = ADC123channel13Pin;
    LL_GPIO_Init(ADCInputPortGroup3, &initData);

    //
    //  These are the DAC's outputs.
    //
    initData.Pin = DACOut1Pin;
    LL_GPIO_Init(DACPort, &initData);
    initData.Pin = DACOut2Pin;
    LL_GPIO_Init(DACPort, &initData);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes pins associated with timer inputs and output.  This includes
//  the motor PWMs and the timer used to decode the quadrature encoded speed
//  command.
//
////////////////////////////////////////////////////////////////////////////////
void GPIOInterface::initializeTimerPins()
{
    using namespace GPIODefinitions;

    //
    //  We configure each pin that is a PWM output.  If another processor is
    //  controlling the motor we skip this step so that we do not disturb its
    //  PWM outputs.
    //
    LL_GPIO_InitTypeDef initData;
    LL_GPIO_StructInit(&initData);
    initData.Mode       = LL_GPIO_MODE_ALTERNATE;
    initData.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
    initData.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    initData.Pull       = LL_GPIO_PULL_NO;

    if (!(SystemConfig::snoopRemoteMotorController))
    {
        initData.Alternate = PWMAlternateFunction;

        initData.Pin = chan1NPin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan1Pin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan2NPin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan2Pin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan3NPin;
        LL_GPIO_Init(pwmOutputPort, &initData);
        initData.Pin = chan3Pin;
        LL_GPIO_Init(pwmOutputPort, &initData);
    }

    //
    //  We configure the timer inputs that are used to decode the speed
    //  command encoder.
    //
    initData.Alternate = speedControlEncoderAlternateFunction;

    initData.Pin = speedControlerEncoderAPin;
    LL_GPIO_Init(speedControlEncoderPort, &initData);
    initData.Pin = speedControlerEncoderBPin;
    LL_GPIO_Init(speedControlEncoderPort, &initData);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes pins associated with the UARTs outputs.
//
////////////////////////////////////////////////////////////////////////////////
void GPIOInterface::initializeUARTPins()
{
    using namespace GPIODefinitions;

    LL_GPIO_InitTypeDef initData;

    LL_GPIO_StructInit(&initData);

    initData.Mode       = LL_GPIO_MODE_ALTERNATE;
    initData.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
    initData.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    initData.Alternate  = uartAlternateFunction;

    //
    //  We configure each pin that is used for a UART.  Note that the TX and
    //  RX have slightly different configurations.
    //
    initData.Pull = LL_GPIO_PULL_NO;
    initData.Pin  = uartTXPin;
    LL_GPIO_Init(uartPort, &initData);

    initData.Pull = LL_GPIO_PULL_UP;
    initData.Pin  = uartRXPin;
    LL_GPIO_Init(uartPort, &initData);
}


