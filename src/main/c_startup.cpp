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
//  Processor reset Entry Point and default trap handlers
//
//  This file contains the first code that is executed at processor startup.
//  In addition, this contains the handlers for the processor traps and
//  unexpected interrupts.
//
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "gpio_interface.h"
#include "motor_controller.h"
#include "processor.h"

//
//  These are referenced by the reset handler.
//
extern void __libc_init_array();
extern int main();

//
//  These are defined in the linker command file.
//
extern uint32_t _bssStart;
extern uint32_t _bssEnd;
extern uint32_t _dataFlashStart;
extern uint32_t _dataRAMStart;
extern uint32_t _dataRAMEnd;

//
//  Many symbols in this file are referenced via assembly routines,
//  others exist to be mapped to the interrupt vector table.
//
void defaultTrapHandler(const uint32_t *const faultStack);
void Default_Handler();
void Reset_Handler() __attribute__((naked));
void NMI_Handler() __attribute__((naked));
void HardFault_Handler() __attribute__((naked));
void MemManage_Handler(void) __attribute__((naked));
void BusFault_Handler(void) __attribute__((naked));
void UsageFault_Handler(void) __attribute__((naked));
void DebugMon_Handler() __attribute__((naked));

//
//  These store information for troubleshooting traps and unexpected interrupts.
//
typedef enum
{
    NoInterrupt = 0,
    NMIInterrupt,
    HardFaultInterrupt,
    MemManagerInterrupt,
    BusFaultInterrupt,
    UsageFaultInterrupt,
    DebugMonitorInterrupt,
    DefaultInterrupt,
    RAMTestFailure
} InterruptHandlers;

static volatile InterruptHandlers  whichFault = NoInterrupt;
static volatile uint32_t           stackedR0;
static volatile uint32_t           stackedR1;
static volatile uint32_t           stackedR2;
static volatile uint32_t           stackedR3;
static volatile uint32_t           stackedR12;
static volatile uint32_t           stackedLR;
static volatile uint32_t           stackedPC;
static volatile uint32_t           stackedPSR;
static volatile uint32_t           trapCFSR;
static volatile uint32_t           trapHFSR;
static volatile uint32_t           trapDFSR;
static volatile uint32_t           trapAFSR;
static volatile uint32_t           trapMMAR;
static volatile uint32_t           trapBFAR;
static volatile uint32_t           whichVector;


////////////////////////////////////////////////////////////////////////////////
//
//  Processor Reset Entry Point
//
//  Prepares the processor for operation including performing basic hardware
//  initializations and setting up memory.
//
////////////////////////////////////////////////////////////////////////////////
void Reset_Handler(void)
{
    //
    //  Reset the initial stack pointer (again) just in case the software is
    //  restarted without being reset.
    //
    __asm("    ldr     r1, = _estack   \n"
          "    mov     sp, r1");

    //
    //  Clear out the bss segments.
    //
    const uint32_t bssStartAddr = reinterpret_cast<uint32_t>(&_bssStart);
    const uint32_t bssEndAddr   = reinterpret_cast<uint32_t>(&_bssEnd);
    const size_t bssSize = bssEndAddr - bssStartAddr;
    memset (&_bssStart, 0U, bssSize);

    //
    //  Copy the data segment initializers from flash to RAM.  The compiler/linker
    //  generates a table that is placed in flash after the portion that is used
    //  by the code (AKA after the .text segment).  We copy that table to the
    //  appropriate spot in RAM.
    //
    uint32_t const *dataTableSourcePtr;
    uint32_t       *dataRAMDestinationPtr;

    dataTableSourcePtr   = &_dataFlashStart;
    dataRAMDestinationPtr = &_dataRAMStart;

    while (dataRAMDestinationPtr < &_dataRAMEnd)
    {
        *dataRAMDestinationPtr = *dataTableSourcePtr;

        dataTableSourcePtr++;
        dataRAMDestinationPtr++;
    }

    //
    //  Perform basic hardware setup including setting up the processor clock
    //  and memory wait-states.
    //
    SystemInit();

    //
    //  Before we let any higher-level code execute, we enable the CPU's
    //  Vectored Floating Point (VFP) unit.
    //
    __asm("    ldr.w   r0, =0xE000ED88      \n"  //Read the CPACR control register
          "    ldr     r1, [r0]             \n"
          "    orr     r1, r1, #(0xF << 20) \n"  //Set bits to enable CP10 and CP11 co-processors
          "    str     r1, [r0]             \n"  //Write back the modified value to the CPACR
          "    dsb                          \n"  //wait for store to complete
          "    isb                          \n"  //reset pipeline now that the FPU is enabled
          );

    //
    //  Now call the C++ library initializations and constructors.
    //
    __libc_init_array();

    //
    //  Finally, call the application's entry point.
    //
    (void)main();

    //
    //  main() shouldn't return, but if it does...
    //
    while (true)
    {
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  Default trap handler
//
//  This is invoked for any interrupt that is not explicitly handled.  This saves
//  information that may be viewed via a debugger.
//
////////////////////////////////////////////////////////////////////////////////
void defaultTrapHandler(const uint32_t *const faultStack)
{
    //  
    //  We retrieve the registers from the fault stack.
    //
    stackedR0  = faultStack[0];
    stackedR1  = faultStack[1];
    stackedR2  = faultStack[2];
    stackedR3  = faultStack[3];
    stackedR12 = faultStack[4];
    stackedLR  = faultStack[5];
    stackedPC  = faultStack[6];
    stackedPSR = faultStack[7];
    
    //
    //  Now we read the various fault status registers.  First the configurable 
    //  Fault Status Register (consists of MMSR, BFSR and UFSR).
    //
    trapCFSR = *(reinterpret_cast<uint32_t *>(0xE000ED28));
    trapHFSR = *(reinterpret_cast<uint32_t *>(0xE000ED2C));
    trapDFSR = *(reinterpret_cast<uint32_t *>(0xE000ED30));
    trapAFSR = *(reinterpret_cast<uint32_t *>(0xE000ED3C));
    trapMMAR = *(reinterpret_cast<uint32_t *>(0xE000ED34));  //  MemManage Fault Address Register
    trapBFAR = *(reinterpret_cast<uint32_t *>(0xE000ED38));  //  Bus Fault Address Register

    //
    //  If we got here via the "Default_Handler" (whichFault is DefaultInterrupt)
    //  this can be used to figure out which interrupt caused it.  "whichVector"
    //  will be the offset into the vector table of the culprit, where zero is the
    //  start of the vendor defined interrupts.  A whichVector of 0 indicates the
    //  WWDG_IRQHandler was invoked, 1 indicates that it was PVD_IRQHandler, etc.
    //  For reference, we are using the ARM Cortex M4 Interrupt Control State
    //
    whichVector = *(reinterpret_cast<uint32_t *>(0xE000ED04U));
    whichVector = (whichVector & 0x000000FFU) - 16U;
    
    //
    //  We shut off the motor electronics.
    //
    shutOffMotor();

    //
    //  Turn on all of the LEDs to give some indication that we are stuck
    //  here.
    //
    GPIOInterface::turnStatusLEDOn();
    GPIOInterface::turnTimingLEDOn();
    GPIOInterface::turnTriggerLEDOn();

    //
    //  We loop forever...
    //
    while (true)
    {
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  NMI Handler
//
//  All of the following ISRs extract state data from the stack and then call
//  the defaultTrapHandler which saves potentially useful data into global
//  variables.
//
////////////////////////////////////////////////////////////////////////////////
void NMI_Handler(void)
{
    whichFault = NMIInterrupt;
    __asm volatile
    (
        " tst lr, #4                            \n"
        " ite eq                                \n"
        " mrseq r0, msp                         \n"
        " mrsne r0, psp                         \n"
        " ldr r1, [r0, #24]                     \n"
        " ldr r2, handler1                      \n"
        " bx r2                                 \n"
        " handler1: .word defaultTrapHandler \n"
    );
}

////////////////////////////////////////////////////////////////////////////////
//
//  Hard Fault Handler
//
////////////////////////////////////////////////////////////////////////////////
void HardFault_Handler(void)
{
    whichFault = HardFaultInterrupt;
    __asm volatile
    (
        " tst lr, #4                            \n"
        " ite eq                                \n"
        " mrseq r0, msp                         \n"
        " mrsne r0, psp                         \n"
        " ldr r1, [r0, #24]                     \n"
        " ldr r2, handler2                      \n"
        " bx r2                                 \n"
        " handler2: .word defaultTrapHandler \n"
    );
}

////////////////////////////////////////////////////////////////////////////////
//
//  Memory Manager Fault Handler
//
////////////////////////////////////////////////////////////////////////////////
void MemManage_Handler(void)
{
    whichFault = MemManagerInterrupt;
    __asm volatile
    (
        " tst lr, #4                            \n"
        " ite eq                                \n"
        " mrseq r0, msp                         \n"
        " mrsne r0, psp                         \n"
        " ldr r1, [r0, #24]                     \n"
        " ldr r2, handler3                      \n"
        " bx r2                                 \n"
        " handler3: .word defaultTrapHandler \n"
    );
}

////////////////////////////////////////////////////////////////////////////////
//
//  Bus Fault Handler
//
////////////////////////////////////////////////////////////////////////////////
void BusFault_Handler(void)
{
    whichFault = BusFaultInterrupt;
    __asm volatile
    (
        " tst lr, #4                            \n"
        " ite eq                                \n"
        " mrseq r0, msp                         \n"
        " mrsne r0, psp                         \n"
        " ldr r1, [r0, #24]                     \n"
        " ldr r2, handler4                      \n"
        " bx r2                                 \n"
        " handler4: .word defaultTrapHandler \n"
    );
}

////////////////////////////////////////////////////////////////////////////////
//
//  Usage Fault Handler
//
////////////////////////////////////////////////////////////////////////////////
void UsageFault_Handler(void)
{
    whichFault = UsageFaultInterrupt;
    __asm volatile
    (
        " tst lr, #4                            \n"
        " ite eq                                \n"
        " mrseq r0, msp                         \n"
        " mrsne r0, psp                         \n"
        " ldr r1, [r0, #24]                     \n"
        " ldr r2, handler5                      \n"
        " bx r2                                 \n"
        " handler5: .word defaultTrapHandler \n"
    );
}

////////////////////////////////////////////////////////////////////////////////
//
//  Debug/Monitor Fault Handler
//
////////////////////////////////////////////////////////////////////////////////
void DebugMon_Handler(void)
{
    whichFault = DebugMonitorInterrupt;
    __asm volatile
    (
        " tst lr, #4                            \n"
        " ite eq                                \n"
        " mrseq r0, msp                         \n"
        " mrsne r0, psp                         \n"
        " ldr r1, [r0, #24]                     \n"
        " ldr r2, handler6                      \n"
        " bx r2                                 \n"
        " handler6: .word defaultTrapHandler \n"
    );
}

////////////////////////////////////////////////////////////////////////////////
//
//  Handler for unexpected interrupts
//
////////////////////////////////////////////////////////////////////////////////
void Default_Handler(void)
{
    whichFault = DefaultInterrupt;
    __asm volatile
    (
        " tst lr, #4                            \n"
        " ite eq                                \n"
        " mrseq r0, msp                         \n"
        " mrsne r0, psp                         \n"
        " ldr r1, [r0, #24]                     \n"
        " ldr r2, handler7                      \n"
        " bx r2                                 \n"
        " handler7: .word defaultTrapHandler \n"
    );
}

//
//  These are "weak" aliases for the possible ISRs.  If an ISR is not
//  defined elsewhere, these will point it to the default ISR handler.
//
void SVC_Handler()                    __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler()                 __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler()                __attribute__((weak, alias("Default_Handler")));
void WWDG_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler()          __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));
void CAN1_TX_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void CAN1_RX0_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler()       __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_IRQHandler()       __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_IRQHandler()  __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler()              __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler()              __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler()              __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler()           __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler()           __attribute__((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler()         __attribute__((weak, alias("Default_Handler")));
void TIM8_BRK_TIM12_IRQHandler()      __attribute__((weak, alias("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler()       __attribute__((weak, alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler()  __attribute__((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void FMC_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));
void SDIO_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void UART5_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void TIM7_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void ETH_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void CAN2_TX_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void CAN2_RX0_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void CAN2_RX1_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void CAN2_SCE_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler()              __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler()        __attribute__((weak, alias("Default_Handler")));
void USART6_IRQHandler()              __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler()      __attribute__((weak, alias("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler()       __attribute__((weak, alias("Default_Handler")));
void OTG_HS_WKUP_IRQHandler()         __attribute__((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler()              __attribute__((weak, alias("Default_Handler")));
void DCMI_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void HASH_RNG_IRQHandler()            __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));
void UART7_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void UART8_IRQHandler()               __attribute__((weak, alias("Default_Handler")));
void SPI4_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void SPI5_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void SPI6_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void SAI1_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void LTDC_IRQHandler()                __attribute__((weak, alias("Default_Handler")));
void LTDC_ER_IRQHandler()             __attribute__((weak, alias("Default_Handler")));
void DMA2D_IRQHandler()               __attribute__((weak, alias("Default_Handler")));

#ifdef __cplusplus
}
#endif
