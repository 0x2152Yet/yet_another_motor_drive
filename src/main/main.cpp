
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
// Application Main
//
// This file contains the application "main" function.  This is not actually
// the first code run after startup.  It is called after the reset handler
// performs its basic initializations.
//
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif

#include "background_task.h"
#include "FreeRTOS.h"
#include "gpio_interface.h"
#include "i2c_interface.h"
#include "hw_definitions.h"
#include "motor_controller.h"
#include "os_interface.h"
#include "processor.h"
#include "physical_inputs.h"
#include "task.h"
#include "tick_timers.h"

//
//  We create a global instance of the items required by the main initializers
//  and also of each task.
//
BackgroundTask   theBackgroundTask;
GPIOInterface    theGPIOInterface;
I2CInterface     theI2CInterface;
MotorController  theMotorController;
OSInterface      theOSInterface;
PhysicalInputs   thePhysicalInputs;
TickTimers       theTimers;

//
//  This powers on the various peripherals we use.
//
static void powerOnPeripherals();

////////////////////////////////////////////////////////////////////////////////
//
// Main function
//
// This performs pre-OS initializations and starts the scheduler.
//
////////////////////////////////////////////////////////////////////////////////
int main(void)
{
    //
    //  We setup the hardware.  The system clocks should have already been
    //  configured in c_startup.cpp.  Note that we initialize the timers
    //  early in the setup so other hardware setups and use the high-res
    //  delay if necessary.
    //
    powerOnPeripherals();
    theTimers.initTimers();
    theGPIOInterface.initializeGPIOInterface();
    thePhysicalInputs.initializePhysicalInputs();

    //
    //  We initialize each task.
    //
    theBackgroundTask.initBackgroundTask();
    theMotorController.initializeMotorController();

    //
    //  We start the kernel.  This should not return.
    //
    OSInterface::startScheduler();

    while (true)
    {
    }
    
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Powers on peripherals.
//
//  This turns on the clocks to the various peripherals that are used.
//
////////////////////////////////////////////////////////////////////////////////
static void powerOnPeripherals(void)
{
    //
    //  Before we power on the peripherals, we "de-init" them.  In many cases,
    //  the de-init shuts off the clocks, so if we do it after we enable the
    //  clocks, things may not work as expected.
    //
    LL_GPIO_DeInit(GPIOA);
    LL_GPIO_DeInit(GPIOB);
    LL_GPIO_DeInit(GPIOC);
    LL_GPIO_DeInit(GPIOD);
    LL_GPIO_DeInit(GPIOE);
    LL_GPIO_DeInit(GPIOF);
    LL_EXTI_DeInit();
    LL_ADC_CommonDeInit(ADCDefinitions::allADCs);
    LL_DMA_DeInit(DMADefinitions::ADCDMA, DMADefinitions::ADC1DMAStream);
    LL_DMA_DeInit(DMADefinitions::ADCDMA, DMADefinitions::ADC2DMAStream);
    LL_DMA_DeInit(DMADefinitions::ADCDMA, DMADefinitions::ADC3DMAStream);
    LL_TIM_DeInit(TimerDefinitions::PWMTimer);
    LL_TIM_DeInit(TimerDefinitions::speedControlEncoderTimer);
    LL_TIM_DeInit(TimerDefinitions::highResolutionTickTimer);
    LL_DAC_DeInit(DACDefinitions::theDAC);
    LL_USART_DeInit(UARTDefinitions::testSupportUART);
    LL_DMA_DeInit(DMADefinitions::UARTDMA, DMADefinitions::UARTTxDMAStream);
    LL_DMA_DeInit(DMADefinitions::UARTDMA, DMADefinitions::UARTRxDMAStream);
    LL_ADC_DeInit(ADCDefinitions::theADC1);
    LL_ADC_DeInit(ADCDefinitions::theADC2);
    LL_ADC_DeInit(ADCDefinitions::theADC3);
    LL_I2C_DeInit(I2CDefinitions::theI2C);

    //
    //  We power on the GPIO peripherals.
    //
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);

    //
    //  We turn on the clock to the ADC interfaces.  We will also turn on the
    //  DMA controller used to transfer the data.
    //
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    //
    //  We power on the timer peripherals that generate the PWM, schedule
    //  ADC data collections, decode the speed command decoder and provide
    //  a general purpose time base.
    //
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    //
    //  This provides the clock to the DAC.
    //
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);

    //
    //  These are for the test support UART.
    //
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    //
    //  This provides the clock for the I2C peripheral.
    //
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

    //
    //  This support general configuration items (external interrupts, etc.)
    //
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
}

#ifdef __cplusplus
}
#endif
