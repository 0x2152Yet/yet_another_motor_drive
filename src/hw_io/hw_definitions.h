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
//  Defines how the various hardware items are used in the application.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"
#include "processor.h"

//
//  These items describe the GPIOs that are used.
//
namespace GPIODefinitions
{
    //
    //  These are the PWM outputs.
    //
    GPIO_TypeDef *const pwmOutputPort   = GPIOE;
    const uint32_t PWMAlternateFunction = LL_GPIO_AF_1;
    const uint32_t chan1NPin            = LL_GPIO_PIN_8;
    const uint32_t chan1Pin             = LL_GPIO_PIN_9;
    const uint32_t chan2NPin            = LL_GPIO_PIN_10;
    const uint32_t chan2Pin             = LL_GPIO_PIN_11;
    const uint32_t chan3NPin            = LL_GPIO_PIN_12;
    const uint32_t chan3Pin             = LL_GPIO_PIN_13;

    //
    //  These are the pins used to enable/disable each bridge driver phase.
    //
    GPIO_TypeDef *const bridgeDriverControlPort1 = GPIOD;
    const uint32_t phaseAControlPin = LL_GPIO_PIN_14;
    const uint32_t phaseBControlPin = LL_GPIO_PIN_15;

    GPIO_TypeDef *const bridgeDriverControlPort2 = GPIOF;
    const uint32_t phaseCControlPin = LL_GPIO_PIN_12;

    //
    //  These are the ADC inputs.
    //
    const uint32_t analogAlternateFunction = LL_GPIO_AF_0;
    GPIO_TypeDef *const ADCInputPortGroup1 = GPIOA;
    const uint32_t ADC123channel0Pin       = LL_GPIO_PIN_0;
    const uint32_t ADC123channel1Pin       = LL_GPIO_PIN_1;
    const uint32_t ADC123channel2Pin       = LL_GPIO_PIN_2;
    const uint32_t ADC123channel3Pin       = LL_GPIO_PIN_3;

    GPIO_TypeDef *const ADCInputPortGroup2 = GPIOF;
    const uint32_t ADC3channel4Pin         = LL_GPIO_PIN_6;
    const uint32_t ADC3channel5Pin         = LL_GPIO_PIN_7;
    const uint32_t ADC3channel6Pin         = LL_GPIO_PIN_8;
    const uint32_t ADC3channel7Pin         = LL_GPIO_PIN_9;
    const uint32_t ADC3channel8Pin         = LL_GPIO_PIN_10;
    const uint32_t ADC3channel9Pin         = LL_GPIO_PIN_3;
    const uint32_t ADC3channel14Pin        = LL_GPIO_PIN_4;
    const uint32_t ADC3channel15Pin        = LL_GPIO_PIN_5;

    GPIO_TypeDef *const ADCInputPortGroup3 = GPIOC;
    const uint32_t ADC123channel10Pin      = LL_GPIO_PIN_0;
    const uint32_t ADC123channel11Pin      = LL_GPIO_PIN_1;
    const uint32_t ADC123channel12Pin      = LL_GPIO_PIN_2;
    const uint32_t ADC123channel13Pin      = LL_GPIO_PIN_3;

    //
    //  These are the DAC outputs.
    //
    GPIO_TypeDef *const DACPort = GPIOA;
    const uint32_t DACOut1Pin              = LL_GPIO_PIN_4;
    const uint32_t DACOut2Pin              = LL_GPIO_PIN_5;

    //
    //  These are the pins used for the test support UART.
    //
    GPIO_TypeDef *const uartPort         = GPIOD;
    const uint32_t uartAlternateFunction = LL_GPIO_AF_7;
    const uint32_t uartTXPin             = LL_GPIO_PIN_8;
    const uint32_t uartRXPin             = LL_GPIO_PIN_9;

    //
    //  These are the LEDs used to provide status and triggers.
    //
    GPIO_TypeDef *const statusLEDPort  = GPIOB;
    const uint32_t statusLEDPin        = LL_GPIO_PIN_0;
    GPIO_TypeDef *const timingLEDPort  = GPIOB;
    const uint32_t timingLEDPin        = LL_GPIO_PIN_7;
    GPIO_TypeDef *const triggerLEDPort = GPIOB;
    const uint32_t triggerLEDPin       = LL_GPIO_PIN_14;

    //
    //  These are the pins used to interface to the quadrature encoded
    //  speed command input.
    //
    GPIO_TypeDef *const speedControlEncoderPort         = GPIOC;
    const uint32_t speedControlEncoderAlternateFunction = LL_GPIO_AF_3;
    const uint32_t speedControlerEncoderAPin            = LL_GPIO_PIN_6;
    const uint32_t speedControlerEncoderBPin            = LL_GPIO_PIN_7;

    //
    // These are the pins that interface to a physical push buttons.
    //
    GPIO_TypeDef *const onBoardPushButtonPort  = GPIOC;
    const uint32_t onBoardpushButtonPin        = LL_GPIO_PIN_13;
    GPIO_TypeDef *const externalPushButtonPort = GPIOB;
    const uint32_t externalPushButtonPin       = LL_GPIO_PIN_11;

    //
    //  These are the pins for the I2C bus
    //
    GPIO_TypeDef *const I2CBusPort       = GPIOB;
    const uint32_t I2CAlternateFunction  = LL_GPIO_AF_4;
    const uint32_t I2CClockPin           = LL_GPIO_PIN_8;
    const uint32_t I2CDataPin            = LL_GPIO_PIN_9;

    //
    //  For testing, we configure an external trigger for the
    //  ADC.
    //
    GPIO_TypeDef *const externalADCTriggerPort  = GPIOC;
    const uint32_t externalADCTriggerPin        = LL_GPIO_PIN_11;

}

//
//  These items describe how the timers are used.
//
namespace TimerDefinitions
{
    //
    //  These are the timers that are used to control the PWM outputs and
    //  ADC timing.
    //
    TIM_TypeDef *const PWMTimer = TIM1;
    TIM_TypeDef *const ADCTimer = TIM3;

    //
    //  This is the timer used to capture the quadrature encoded speed
    //  command input.
    //
    TIM_TypeDef *const speedControlEncoderTimer = TIM8;

    //
    //  This is our general purpose tick timer.
    //
    TIM_TypeDef *const highResolutionTickTimer = TIM2;
}

//
//  These items describe how the ADC is used
//
namespace ADCDefinitions
{
    //
    //  We use all three ADCs to make sure that we sample the data items
    //  as closely as possible to the PWM midpoint.
    //
    ADC_Common_TypeDef *const allADCs = ADC123_COMMON;
    ADC_TypeDef *const theADC1 = ADC1;
    ADC_TypeDef *const theADC2 = ADC2;
    ADC_TypeDef *const theADC3 = ADC3;

    //
    //  We can trigger ADC samples based on the local PWM timer (the normal
    //  configuration if this processor is controlling the motor) or
    //  we can trigger the ADC from an external I/O if another processor is
    //  controlling the motor and we are simply monitoring its data.  Whether
    //  or not this processor is controlling the motor is defined in config.h
    //  with the boolean snoopRemoteMotorController.
    //
    const uint32_t localPWMstartOfConversionTrigger = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
    const uint32_t externalstartOfConversionTrigger = LL_ADC_REG_TRIG_EXT_EXTI_LINE11;
}

//
//  These items describe how the DAC is used
//
namespace DACDefinitions
{
    //
    //  There is only 1 DAC
    //
    DAC_TypeDef *const theDAC = DAC1;
}

//
//  These items define how the various DMA streams are used.
//
namespace DMADefinitions
{
    DMA_TypeDef *const ADCDMA = DMA2;
    const uint32_t ADC1DMAChannel = LL_DMA_CHANNEL_0;
    const uint32_t ADC1DMAStream  = LL_DMA_STREAM_4;
    const uint32_t ADC2DMAChannel = LL_DMA_CHANNEL_1;
    const uint32_t ADC2DMAStream  = LL_DMA_STREAM_2;
    const uint32_t ADC3DMAChannel = LL_DMA_CHANNEL_2;
    const uint32_t ADC3DMAStream  = LL_DMA_STREAM_0;

    DMA_TypeDef *const UARTDMA = DMA1;
    const uint32_t UARTTxDMAChannel = LL_DMA_CHANNEL_4;
    const uint32_t UARTTxDMAStream  = LL_DMA_STREAM_3;
    const uint32_t UARTRxDMAChannel = LL_DMA_CHANNEL_4;
    const uint32_t UARTRxDMAStream  = LL_DMA_STREAM_1;
}

//
//  These items define how the UARTs are used.
//
namespace UARTDefinitions
{
    //
    //  The test support UART is set up with 8 data bits, 1 stop bit,
    //  no parity, and no hardware flow control.
    //
    USART_TypeDef *const testSupportUART = USART3;
    const uint32_t testSupportBaudRate   = 921600U;
}

//
//  These define how we use the I2C bus. For now, this is just for test
//  support.
//
namespace I2CDefinitions
{
    I2C_TypeDef *const theI2C = I2C1;
    const bitsPerSecond I2CClockSpeed = 100000U;
}

