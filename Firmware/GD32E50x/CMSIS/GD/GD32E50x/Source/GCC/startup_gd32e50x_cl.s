/**
  ******************************************************************************
  * @file      startup_stm32l552xx.s
  * @author    MCD Application Team
  * @brief     STM32L552xx devices vector table GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address,
  *                - Configure the clock system
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M33 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Apache License, Version 2.0,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/Apache-2.0
  *
  ******************************************************************************
  */

  .syntax unified
	.cpu cortex-m33
	.fpu softvfp
	.thumb

.global	g_pfnVectors
.global	Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word	_sidata
/* start address for the .data section. defined in linker script */
.word	_sdata
/* end address for the .data section. defined in linker script */
.word	_edata
/* start address for the .bss section. defined in linker script */
.word	_sbss
/* end address for the .bss section. defined in linker script */
.word	_ebss

.equ  BootRAM,        0xF1E0F85F
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
  ldr   sp, =_estack    /* set stack pointer */

/* Call the clock system initialization function.*/
  bl  SystemInit

/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4

LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
	movs	r3, #0
	str	r3, [r2], #4

LoopFillZerobss:
	ldr	r3, = _ebss
	cmp	r2, r3
	bcc	FillZerobss

/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
	bl	main

LoopForever:
    b LoopForever

.size	Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex-M33.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
	.size	g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
    .word     _estack                      // Top of Stack
    .word     Reset_Handler                     // Reset Handler
    .word     NMI_Handler                       // NMI Handler
    .word     HardFault_Handler                 // Hard Fault Handler
    .word     MemManage_Handler                 // MPU Fault Handler
    .word     BusFault_Handler                  // Bus Fault Handler
    .word     UsageFault_Handler                // Usage Fault Handler
    .word     0                                 // Reserved
    .word     0                                 // Reserved
    .word     0                                 // Reserved
    .word     0                                 // Reserved
    .word     SVC_Handler                       // SVCall Handler
    .word     DebugMon_Handler                  // Debug Monitor Handler
    .word     0                                 // Reserved
    .word     PendSV_Handler                    // PendSV Handler
    .word     SysTick_Handler                   // SysTick Handler

    /* external interrupts handler */
    .word     WWDGT_IRQHandler                  // 16:Window Watchdog Timer
    .word     LVD_IRQHandler                    // 17:LVD through EXTI Line detect
    .word     TAMPER_IRQHandler                 // 18:Tamper through EXTI Line detect
    .word     RTC_IRQHandler                    // 19:RTC through EXTI Line
    .word     FMC_IRQHandler                    // 20:FMC
    .word     RCU_CTC_IRQHandler                // 21:RCU and CTC
    .word     EXTI0_IRQHandler                  // 22:EXTI Line 0
    .word     EXTI1_IRQHandler                  // 23:EXTI Line 1
    .word     EXTI2_IRQHandler                  // 24:EXTI Line 2
    .word     EXTI3_IRQHandler                  // 25:EXTI Line 3
    .word     EXTI4_IRQHandler                  // 26:EXTI Line 4
    .word     DMA0_Channel0_IRQHandler          // 27:DMA0 Channel0
    .word     DMA0_Channel1_IRQHandler          // 28:DMA0 Channel1
    .word     DMA0_Channel2_IRQHandler          // 29:DMA0 Channel2
    .word     DMA0_Channel3_IRQHandler          // 30:DMA0 Channel3
    .word     DMA0_Channel4_IRQHandler          // 31:DMA0 Channel4
    .word     DMA0_Channel5_IRQHandler          // 32:DMA0 Channel5
    .word     DMA0_Channel6_IRQHandler          // 33:DMA0 Channel6
    .word     ADC0_1_IRQHandler                 // 34:ADC0 and ADC1
    .word     CAN0_TX_IRQHandler                // 35:CAN0 TX
    .word     CAN0_RX0_IRQHandler               // 36:CAN0 RX0
    .word     CAN0_RX1_IRQHandler               // 37:CAN0 RX1
    .word     CAN0_EWMC_IRQHandler              // 38:CAN0 EWMC
    .word     EXTI5_9_IRQHandler                // 39:EXTI5 to EXTI9
    .word     TIMER0_BRK_TIMER8_IRQHandler      // 40:TIMER0 Break and TIMER8
    .word     TIMER0_UP_TIMER9_IRQHandler       // 41:TIMER0 Update and TIMER9
    .word     TIMER0_TRG_CMT_TIMER10_IRQHandler // 42:TIMER0 Trigger and Commutation and TIMER10
    .word     TIMER0_Channel_IRQHandler         // 43:TIMER0 Channel Capture Compare
    .word     TIMER1_IRQHandler                 // 44:TIMER1
    .word     TIMER2_IRQHandler                 // 45:TIMER2
    .word     TIMER3_IRQHandler                 // 46:TIMER3
    .word     I2C0_EV_IRQHandler                // 47:I2C0 Event
    .word     I2C0_ER_IRQHandler                // 48:I2C0 Error
    .word     I2C1_EV_IRQHandler                // 49:I2C1 Event
    .word     I2C1_ER_IRQHandler                // 50:I2C1 Error
    .word     SPI0_IRQHandler                   // 51:SPI0
    .word     SPI1_I2S1ADD_IRQHandler           // 52:SPI1 or I2S1ADD
    .word     USART0_IRQHandler                 // 53:USART0
    .word     USART1_IRQHandler                 // 54:USART1
    .word     USART2_IRQHandler                 // 55:USART2
    .word     EXTI10_15_IRQHandler              // 56:EXTI10 to EXTI15
    .word     RTC_Alarm_IRQHandler              // 57:RTC Alarm
    .word     USBHS_WKUP_IRQHandler             // 58:USBHS Wakeup
    .word     TIMER7_BRK_TIMER11_IRQHandler     // 59:TIMER7 Break and TIMER11
    .word     TIMER7_UP_TIMER12_IRQHandler      // 60:TIMER7 Update and TIMER12
    .word     TIMER7_TRG_CMT_TIMER13_IRQHandler // 61:TIMER7 Trigger and Commutation and TIMER13
    .word     TIMER7_Channel_IRQHandler         // 62:TIMER7 Channel Capture Compare
    .word     0                                 // Reserved
    .word     EXMC_IRQHandler                   // 64:EXMC
    .word     0                                 // Reserved
    .word     TIMER4_IRQHandler                 // 66:TIMER4
    .word     SPI2_I2S2ADD_IRQHandler           // 67:SPI2 or I2S2ADD
    .word     UART3_IRQHandler                  // 68:UART3
    .word     UART4_IRQHandler                  // 69:UART4
    .word     TIMER5_DAC_IRQHandler             // 70:TIMER5 or DAC
    .word     TIMER6_IRQHandler                 // 71:TIMER6
    .word     DMA1_Channel0_IRQHandler          // 72:DMA1 Channel0
    .word     DMA1_Channel1_IRQHandler          // 73:DMA1 Channel1
    .word     DMA1_Channel2_IRQHandler          // 74:DMA1 Channel2
    .word     DMA1_Channel3_IRQHandler          // 75:DMA1 Channel3
    .word     DMA1_Channel4_IRQHandler          // 76:DMA1 Channel4
    .word     ENET_IRQHandler                   // 77:Ethernet
    .word     ENET_WKUP_IRQHandler              // 78:Ethernet Wakeup through EXTI Line
    .word     CAN1_TX_IRQHandler                // 79:CAN1 TX
    .word     CAN1_RX0_IRQHandler               // 80:CAN1 RX0
    .word     CAN1_RX1_IRQHandler               // 81:CAN1 RX1
    .word     CAN1_EWMC_IRQHandler              // 82:CAN1 EWMC
    .word     USBHS_IRQHandler                  // 83:USBHS
    .word     0                                 // Reserved
    .word     SHRTIMER_IRQ2_IRQHandler          // 85:SHRTIMER IRQ2
    .word     SHRTIMER_IRQ3_IRQHandler          // 86:SHRTIMER IRQ3
    .word     SHRTIMER_IRQ4_IRQHandler          // 87:SHRTIMER IRQ4
    .word     SHRTIMER_IRQ5_IRQHandler          // 88:SHRTIMER IRQ5
    .word     SHRTIMER_IRQ6_IRQHandler          // 89:SHRTIMER IRQ6
    .word     USBHS_EP1_OUT_IRQHandler          // 90:USBHS end point 1 out
    .word     USBHS_EP1_IN_IRQHandler           // 91:USBHS end point 1 in
    .word     SHRTIMER_IRQ0_IRQHandler          // 92:SHRTIMER IRQ0
    .word     SHRTIMER_IRQ1_IRQHandler          // 93:SHRTIMER IRQ1
    .word     CAN2_TX_IRQHandler                // 94:CAN2 TX
    .word     CAN2_RX0_IRQHandler               // 95:CAN2 RX0
    .word     CAN2_RX1_IRQHandler               // 96:CAN2 RX1
    .word     CAN2_EWMC_IRQHandler              // 97:CAN2 EWMC
    .word     I2C2_EV_IRQHandler                // 98:I2C2 Event
    .word     I2C2_ER_IRQHandler                // 99:I2C2 Error
    .word     USART5_IRQHandler                 // 100:USART5
    .word     I2C2_WKUP_IRQHandler              // 101:I2C2 Wakeup
    .word     USART5_WKUP_IRQHandler            // 102:USART5 Wakeup
    .word     TMU_IRQHandler                    // 103:TMU
	/* .word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	SecureFault_Handler
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.word	0
	.word	PendSV_Handler
	.word	SysTick_Handler
	.word	WWDG_IRQHandler
	.word	PVD_PVM_IRQHandler
	.word	RTC_IRQHandler
	.word	RTC_S_IRQHandler
	.word	TAMP_IRQHandler
	.word	TAMP_S_IRQHandler
	.word	FLASH_IRQHandler
	.word	FLASH_S_IRQHandler
	.word	GTZC_IRQHandler
	.word	RCC_IRQHandler
	.word	RCC_S_IRQHandler
	.word	EXTI0_IRQHandler
	.word	EXTI1_IRQHandler
	.word	EXTI2_IRQHandler
	.word	EXTI3_IRQHandler
	.word	EXTI4_IRQHandler
	.word	EXTI5_IRQHandler
	.word	EXTI6_IRQHandler
	.word	EXTI7_IRQHandler
	.word	EXTI8_IRQHandler
	.word	EXTI9_IRQHandler
	.word	EXTI10_IRQHandler
	.word	EXTI11_IRQHandler
	.word	EXTI12_IRQHandler
	.word	EXTI13_IRQHandler
	.word	EXTI14_IRQHandler
	.word	EXTI15_IRQHandler
	.word	DMAMUX1_IRQHandler
	.word	DMAMUX1_S_IRQHandler
	.word	DMA1_Channel1_IRQHandler
	.word	DMA1_Channel2_IRQHandler
	.word	DMA1_Channel3_IRQHandler
	.word	DMA1_Channel4_IRQHandler
	.word	DMA1_Channel5_IRQHandler
	.word	DMA1_Channel6_IRQHandler
	.word	DMA1_Channel7_IRQHandler
	.word	DMA1_Channel8_IRQHandler
	.word	ADC1_2_IRQHandler
	.word	DAC_IRQHandler
	.word	FDCAN1_IT0_IRQHandler
	.word	FDCAN1_IT1_IRQHandler
	.word	TIM1_BRK_IRQHandler
	.word	TIM1_UP_IRQHandler
	.word	TIM1_TRG_COM_IRQHandler
	.word	TIM1_CC_IRQHandler
	.word	TIM2_IRQHandler
	.word	TIM3_IRQHandler
	.word	TIM4_IRQHandler
	.word	TIM5_IRQHandler
	.word	TIM6_IRQHandler
	.word	TIM7_IRQHandler
	.word	TIM8_BRK_IRQHandler
	.word	TIM8_UP_IRQHandler
	.word	TIM8_TRG_COM_IRQHandler
	.word	TIM8_CC_IRQHandler
	.word	I2C1_EV_IRQHandler
	.word	I2C1_ER_IRQHandler
	.word	I2C2_EV_IRQHandler
	.word	I2C2_ER_IRQHandler
	.word	SPI1_IRQHandler
	.word	SPI2_IRQHandler
	.word	USART1_IRQHandler
	.word	USART2_IRQHandler
	.word	USART3_IRQHandler
	.word	UART4_IRQHandler
	.word	UART5_IRQHandler
	.word	LPUART1_IRQHandler
	.word	LPTIM1_IRQHandler
	.word	LPTIM2_IRQHandler
	.word	TIM15_IRQHandler
	.word	TIM16_IRQHandler
	.word	TIM17_IRQHandler
	.word	COMP_IRQHandler
	.word	USB_FS_IRQHandler
	.word	CRS_IRQHandler
	.word	FMC_IRQHandler
	.word	OCTOSPI1_IRQHandler
	.word	0
	.word	SDMMC1_IRQHandler
	.word	0
	.word	DMA2_Channel1_IRQHandler
	.word	DMA2_Channel2_IRQHandler
	.word	DMA2_Channel3_IRQHandler
	.word	DMA2_Channel4_IRQHandler
	.word	DMA2_Channel5_IRQHandler
	.word	DMA2_Channel6_IRQHandler
	.word	DMA2_Channel7_IRQHandler
	.word	DMA2_Channel8_IRQHandler
	.word	I2C3_EV_IRQHandler
	.word	I2C3_ER_IRQHandler
	.word	SAI1_IRQHandler
	.word	SAI2_IRQHandler
	.word	TSC_IRQHandler
	.word	0
	.word	RNG_IRQHandler
	.word	FPU_IRQHandler
	.word	HASH_IRQHandler
	.word	0
	.word	LPTIM3_IRQHandler
	.word	SPI3_IRQHandler
	.word	I2C4_ER_IRQHandler
	.word	I2C4_EV_IRQHandler
	.word	DFSDM1_FLT0_IRQHandler
	.word	DFSDM1_FLT1_IRQHandler
	.word	DFSDM1_FLT2_IRQHandler
	.word	DFSDM1_FLT3_IRQHandler
	.word	UCPD1_IRQHandler
	.word	ICACHE_IRQHandler
	.word	0*/


/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

	.macro	Set_Default_Handler	Handler_Name
    .weak	\Handler_Name
    .thumb_set	\Handler_Name, Default_Handler
    .endm

    Set_Default_Handler     NMI_Handler                       // NMI Handler
    Set_Default_Handler     HardFault_Handler                 // Hard Fault Handler
    Set_Default_Handler     MemManage_Handler                 // MPU Fault Handler
    Set_Default_Handler     BusFault_Handler                  // Bus Fault Handler
    Set_Default_Handler     UsageFault_Handler                // Usage Fault Handler
    Set_Default_Handler     SVC_Handler                       // SVCall Handler
    Set_Default_Handler     DebugMon_Handler                  // Debug Monitor Handler
    Set_Default_Handler     PendSV_Handler                    // PendSV Handler
    Set_Default_Handler     SysTick_Handler                   // SysTick Handler

    /* external interrupts handler */
    Set_Default_Handler     WWDGT_IRQHandler                  // 16:Window Watchdog Timer
    Set_Default_Handler     LVD_IRQHandler                    // 17:LVD through EXTI Line detect
    Set_Default_Handler     TAMPER_IRQHandler                 // 18:Tamper through EXTI Line detect
    Set_Default_Handler     RTC_IRQHandler                    // 19:RTC through EXTI Line
    Set_Default_Handler     FMC_IRQHandler                    // 20:FMC
    Set_Default_Handler     RCU_CTC_IRQHandler                // 21:RCU and CTC
    Set_Default_Handler     EXTI0_IRQHandler                  // 22:EXTI Line 0
    Set_Default_Handler     EXTI1_IRQHandler                  // 23:EXTI Line 1
    Set_Default_Handler     EXTI2_IRQHandler                  // 24:EXTI Line 2
    Set_Default_Handler     EXTI3_IRQHandler                  // 25:EXTI Line 3
    Set_Default_Handler     EXTI4_IRQHandler                  // 26:EXTI Line 4
    Set_Default_Handler     DMA0_Channel0_IRQHandler          // 27:DMA0 Channel0
    Set_Default_Handler     DMA0_Channel1_IRQHandler          // 28:DMA0 Channel1
    Set_Default_Handler     DMA0_Channel2_IRQHandler          // 29:DMA0 Channel2
    Set_Default_Handler     DMA0_Channel3_IRQHandler          // 30:DMA0 Channel3
    Set_Default_Handler     DMA0_Channel4_IRQHandler          // 31:DMA0 Channel4
    Set_Default_Handler     DMA0_Channel5_IRQHandler          // 32:DMA0 Channel5
    Set_Default_Handler     DMA0_Channel6_IRQHandler          // 33:DMA0 Channel6
    Set_Default_Handler     ADC0_1_IRQHandler                 // 34:ADC0 and ADC1
    Set_Default_Handler     CAN0_TX_IRQHandler                // 35:CAN0 TX
    Set_Default_Handler     CAN0_RX0_IRQHandler               // 36:CAN0 RX0
    Set_Default_Handler     CAN0_RX1_IRQHandler               // 37:CAN0 RX1
    Set_Default_Handler     CAN0_EWMC_IRQHandler              // 38:CAN0 EWMC
    Set_Default_Handler     EXTI5_9_IRQHandler                // 39:EXTI5 to EXTI9
    Set_Default_Handler     TIMER0_BRK_TIMER8_IRQHandler      // 40:TIMER0 Break and TIMER8
    Set_Default_Handler     TIMER0_UP_TIMER9_IRQHandler       // 41:TIMER0 Update and TIMER9
    Set_Default_Handler     TIMER0_TRG_CMT_TIMER10_IRQHandler // 42:TIMER0 Trigger and Commutation and TIMER10
    Set_Default_Handler     TIMER0_Channel_IRQHandler         // 43:TIMER0 Channel Capture Compare
    Set_Default_Handler     TIMER1_IRQHandler                 // 44:TIMER1
    Set_Default_Handler     TIMER2_IRQHandler                 // 45:TIMER2
    Set_Default_Handler     TIMER3_IRQHandler                 // 46:TIMER3
    Set_Default_Handler     I2C0_EV_IRQHandler                // 47:I2C0 Event
    Set_Default_Handler     I2C0_ER_IRQHandler                // 48:I2C0 Error
    Set_Default_Handler     I2C1_EV_IRQHandler                // 49:I2C1 Event
    Set_Default_Handler     I2C1_ER_IRQHandler                // 50:I2C1 Error
    Set_Default_Handler     SPI0_IRQHandler                   // 51:SPI0
    Set_Default_Handler     SPI1_I2S1ADD_IRQHandler           // 52:SPI1 or I2S1ADD
    Set_Default_Handler     USART0_IRQHandler                 // 53:USART0
    Set_Default_Handler     USART1_IRQHandler                 // 54:USART1
    Set_Default_Handler     USART2_IRQHandler                 // 55:USART2
    Set_Default_Handler     EXTI10_15_IRQHandler              // 56:EXTI10 to EXTI15
    Set_Default_Handler     RTC_Alarm_IRQHandler              // 57:RTC Alarm
    Set_Default_Handler     USBHS_WKUP_IRQHandler             // 58:USBHS Wakeup
    Set_Default_Handler     TIMER7_BRK_TIMER11_IRQHandler     // 59:TIMER7 Break and TIMER11
    Set_Default_Handler     TIMER7_UP_TIMER12_IRQHandler      // 60:TIMER7 Update and TIMER12
    Set_Default_Handler     TIMER7_TRG_CMT_TIMER13_IRQHandler // 61:TIMER7 Trigger and Commutation and TIMER13
    Set_Default_Handler     TIMER7_Channel_IRQHandler         // 62:TIMER7 Channel Capture Compare
    Set_Default_Handler     EXMC_IRQHandler                   // 64:EXMC
    Set_Default_Handler     TIMER4_IRQHandler                 // 66:TIMER4
    Set_Default_Handler     SPI2_I2S2ADD_IRQHandler           // 67:SPI2 or I2S2ADD
    Set_Default_Handler     UART3_IRQHandler                  // 68:UART3
    Set_Default_Handler     UART4_IRQHandler                  // 69:UART4
    Set_Default_Handler     TIMER5_DAC_IRQHandler             // 70:TIMER5 or DAC
    Set_Default_Handler     TIMER6_IRQHandler                 // 71:TIMER6
    Set_Default_Handler     DMA1_Channel0_IRQHandler          // 72:DMA1 Channel0
    Set_Default_Handler     DMA1_Channel1_IRQHandler          // 73:DMA1 Channel1
    Set_Default_Handler     DMA1_Channel2_IRQHandler          // 74:DMA1 Channel2
    Set_Default_Handler     DMA1_Channel3_IRQHandler          // 75:DMA1 Channel3
    Set_Default_Handler     DMA1_Channel4_IRQHandler          // 76:DMA1 Channel4
    Set_Default_Handler     ENET_IRQHandler                   // 77:Ethernet
    Set_Default_Handler     ENET_WKUP_IRQHandler              // 78:Ethernet Wakeup through EXTI Line
    Set_Default_Handler     CAN1_TX_IRQHandler                // 79:CAN1 TX
    Set_Default_Handler     CAN1_RX0_IRQHandler               // 80:CAN1 RX0
    Set_Default_Handler     CAN1_RX1_IRQHandler               // 81:CAN1 RX1
    Set_Default_Handler     CAN1_EWMC_IRQHandler              // 82:CAN1 EWMC
    Set_Default_Handler     USBHS_IRQHandler                  // 83:USBHS
    Set_Default_Handler     SHRTIMER_IRQ2_IRQHandler          // 85:SHRTIMER IRQ2
    Set_Default_Handler     SHRTIMER_IRQ3_IRQHandler          // 86:SHRTIMER IRQ3
    Set_Default_Handler     SHRTIMER_IRQ4_IRQHandler          // 87:SHRTIMER IRQ4
    Set_Default_Handler     SHRTIMER_IRQ5_IRQHandler          // 88:SHRTIMER IRQ5
    Set_Default_Handler     SHRTIMER_IRQ6_IRQHandler          // 89:SHRTIMER IRQ6
    Set_Default_Handler     USBHS_EP1_OUT_IRQHandler          // 90:USBHS end point 1 out
    Set_Default_Handler     USBHS_EP1_IN_IRQHandler           // 91:USBHS end point 1 in
    Set_Default_Handler     SHRTIMER_IRQ0_IRQHandler          // 92:SHRTIMER IRQ0
    Set_Default_Handler     SHRTIMER_IRQ1_IRQHandler          // 93:SHRTIMER IRQ1
    Set_Default_Handler     CAN2_TX_IRQHandler                // 94:CAN2 TX
    Set_Default_Handler     CAN2_RX0_IRQHandler               // 95:CAN2 RX0
    Set_Default_Handler     CAN2_RX1_IRQHandler               // 96:CAN2 RX1
    Set_Default_Handler     CAN2_EWMC_IRQHandler              // 97:CAN2 EWMC
    Set_Default_Handler     I2C2_EV_IRQHandler                // 98:I2C2 Event
    Set_Default_Handler     I2C2_ER_IRQHandler                // 99:I2C2 Error
    Set_Default_Handler     USART5_IRQHandler                 // 100:USART5
    Set_Default_Handler     I2C2_WKUP_IRQHandler              // 101:I2C2 Wakeup
    Set_Default_Handler     USART5_WKUP_IRQHandler            // 102:USART5 Wakeup
    Set_Default_Handler     TMU_IRQHandler                    // 103:TMU


	/* .weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SecureFault_Handler
	.thumb_set SecureFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler

	.weak	PVD_PVM_IRQHandler
	.thumb_set PVD_PVM_IRQHandler,Default_Handler

	.weak	RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler

	.weak	RTC_S_IRQHandler
	.thumb_set RTC_S_IRQHandler,Default_Handler

	.weak	TAMP_IRQHandler
	.thumb_set TAMP_IRQHandler,Default_Handler

	.weak	TAMP_S_IRQHandler
	.thumb_set TAMP_S_IRQHandler,Default_Handler

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

	.weak	FLASH_S_IRQHandler
	.thumb_set FLASH_S_IRQHandler,Default_Handler

	.weak	GTZC_IRQHandler
	.thumb_set GTZC_IRQHandler,Default_Handler

	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler

	.weak	RCC_S_IRQHandler
	.thumb_set RCC_S_IRQHandler,Default_Handler

	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler

	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler

	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler

	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler

	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler

	.weak	EXTI5_IRQHandler
	.thumb_set EXTI5_IRQHandler,Default_Handler

	.weak	EXTI6_IRQHandler
	.thumb_set EXTI6_IRQHandler,Default_Handler

	.weak	EXTI7_IRQHandler
	.thumb_set EXTI7_IRQHandler,Default_Handler

	.weak	EXTI8_IRQHandler
	.thumb_set EXTI8_IRQHandler,Default_Handler

	.weak	EXTI9_IRQHandler
	.thumb_set EXTI9_IRQHandler,Default_Handler

	.weak	EXTI10_IRQHandler
	.thumb_set EXTI10_IRQHandler,Default_Handler

	.weak	EXTI11_IRQHandler
	.thumb_set EXTI11_IRQHandler,Default_Handler

	.weak	EXTI12_IRQHandler
	.thumb_set EXTI12_IRQHandler,Default_Handler

	.weak	EXTI13_IRQHandler
	.thumb_set EXTI13_IRQHandler,Default_Handler

	.weak	EXTI14_IRQHandler
	.thumb_set EXTI14_IRQHandler,Default_Handler

	.weak	EXTI15_IRQHandler
	.thumb_set EXTI15_IRQHandler,Default_Handler

	.weak	DMAMUX1_IRQHandler
	.thumb_set DMAMUX1_IRQHandler,Default_Handler

	.weak	DMAMUX1_S_IRQHandler
	.thumb_set DMAMUX1_S_IRQHandler,Default_Handler

	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler

	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler

	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler

	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler

	.weak	DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler

	.weak	DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler

	.weak	DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler

	.weak	DMA1_Channel8_IRQHandler
	.thumb_set DMA1_Channel8_IRQHandler,Default_Handler

	.weak	ADC1_2_IRQHandler
	.thumb_set ADC1_2_IRQHandler,Default_Handler

	.weak	DAC_IRQHandler
	.thumb_set DAC_IRQHandler,Default_Handler

	.weak	FDCAN1_IT0_IRQHandler
	.thumb_set FDCAN1_IT0_IRQHandler,Default_Handler

	.weak	FDCAN1_IT1_IRQHandler
	.thumb_set FDCAN1_IT1_IRQHandler,Default_Handler

	.weak	TIM1_BRK_IRQHandler
	.thumb_set TIM1_BRK_IRQHandler,Default_Handler

	.weak	TIM1_UP_IRQHandler
	.thumb_set TIM1_UP_IRQHandler,Default_Handler

	.weak	TIM1_TRG_COM_IRQHandler
	.thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler

	.weak	TIM4_IRQHandler
	.thumb_set TIM4_IRQHandler,Default_Handler

	.weak	TIM5_IRQHandler
	.thumb_set TIM5_IRQHandler,Default_Handler

	.weak	TIM6_IRQHandler
	.thumb_set TIM6_IRQHandler,Default_Handler

	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler

	.weak	TIM8_BRK_IRQHandler
	.thumb_set TIM8_BRK_IRQHandler,Default_Handler

	.weak	TIM8_UP_IRQHandler
	.thumb_set TIM8_UP_IRQHandler,Default_Handler

	.weak	TIM8_TRG_COM_IRQHandler
	.thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

	.weak	TIM8_CC_IRQHandler
	.thumb_set TIM8_CC_IRQHandler,Default_Handler

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler

	.weak	I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler

	.weak	I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler

	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler

	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler

	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler

	.weak	USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler

	.weak	UART4_IRQHandler
	.thumb_set UART4_IRQHandler,Default_Handler

	.weak	UART5_IRQHandler
	.thumb_set UART5_IRQHandler,Default_Handler

	.weak	LPUART1_IRQHandler
	.thumb_set LPUART1_IRQHandler,Default_Handler

	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler

	.weak	LPTIM2_IRQHandler
	.thumb_set LPTIM2_IRQHandler,Default_Handler

	.weak	TIM15_IRQHandler
	.thumb_set TIM15_IRQHandler,Default_Handler

	.weak	TIM16_IRQHandler
	.thumb_set TIM16_IRQHandler,Default_Handler

	.weak	TIM17_IRQHandler
	.thumb_set TIM17_IRQHandler,Default_Handler

	.weak	COMP_IRQHandler
	.thumb_set COMP_IRQHandler,Default_Handler

	.weak	USB_FS_IRQHandler
	.thumb_set USB_FS_IRQHandler,Default_Handler

	.weak	CRS_IRQHandler
	.thumb_set CRS_IRQHandler,Default_Handler

	.weak	FMC_IRQHandler
	.thumb_set FMC_IRQHandler,Default_Handler

	.weak	OCTOSPI1_IRQHandler
	.thumb_set OCTOSPI1_IRQHandler,Default_Handler

	.weak	SDMMC1_IRQHandler
	.thumb_set SDMMC1_IRQHandler,Default_Handler

	.weak	DMA2_Channel1_IRQHandler
	.thumb_set DMA2_Channel1_IRQHandler,Default_Handler

	.weak	DMA2_Channel2_IRQHandler
	.thumb_set DMA2_Channel2_IRQHandler,Default_Handler

	.weak	DMA2_Channel3_IRQHandler
	.thumb_set DMA2_Channel3_IRQHandler,Default_Handler

	.weak	DMA2_Channel4_IRQHandler
	.thumb_set DMA2_Channel4_IRQHandler,Default_Handler

	.weak	DMA2_Channel5_IRQHandler
	.thumb_set DMA2_Channel5_IRQHandler,Default_Handler

	.weak	DMA2_Channel6_IRQHandler
	.thumb_set DMA2_Channel6_IRQHandler,Default_Handler

	.weak	DMA2_Channel7_IRQHandler
	.thumb_set DMA2_Channel7_IRQHandler,Default_Handler

	.weak	DMA2_Channel8_IRQHandler
	.thumb_set DMA2_Channel8_IRQHandler,Default_Handler

	.weak	I2C3_EV_IRQHandler
	.thumb_set I2C3_EV_IRQHandler,Default_Handler

	.weak	I2C3_ER_IRQHandler
	.thumb_set I2C3_ER_IRQHandler,Default_Handler

	.weak	SAI1_IRQHandler
	.thumb_set SAI1_IRQHandler,Default_Handler

	.weak	SAI2_IRQHandler
	.thumb_set SAI2_IRQHandler,Default_Handler

	.weak	TSC_IRQHandler
	.thumb_set TSC_IRQHandler,Default_Handler

	.weak	RNG_IRQHandler
	.thumb_set RNG_IRQHandler,Default_Handler

	.weak	FPU_IRQHandler
	.thumb_set FPU_IRQHandler,Default_Handler

	.weak	HASH_IRQHandler
	.thumb_set HASH_IRQHandler,Default_Handler

	.weak	LPTIM3_IRQHandler
	.thumb_set LPTIM3_IRQHandler,Default_Handler

	.weak	SPI3_IRQHandler
	.thumb_set SPI3_IRQHandler,Default_Handler

	.weak	I2C4_ER_IRQHandler
	.thumb_set I2C4_ER_IRQHandler,Default_Handler

	.weak	I2C4_EV_IRQHandler
	.thumb_set I2C4_EV_IRQHandler,Default_Handler

	.weak	DFSDM1_FLT0_IRQHandler
	.thumb_set DFSDM1_FLT0_IRQHandler,Default_Handler

	.weak	DFSDM1_FLT1_IRQHandler
	.thumb_set DFSDM1_FLT1_IRQHandler,Default_Handler

	.weak	DFSDM1_FLT2_IRQHandler
	.thumb_set DFSDM1_FLT2_IRQHandler,Default_Handler

	.weak	DFSDM1_FLT3_IRQHandler
	.thumb_set DFSDM1_FLT3_IRQHandler,Default_Handler

	.weak	UCPD1_IRQHandler
	.thumb_set UCPD1_IRQHandler,Default_Handler

	.weak	ICACHE_IRQHandler
	.thumb_set ICACHE_IRQHandler,Default_Handler*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
