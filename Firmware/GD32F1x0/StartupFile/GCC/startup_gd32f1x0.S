/**************************************************************************//**
 * @file     startup_ARMCM3.S
 * @brief    CMSIS-Core(M) Device Startup File for Cortex-M3 Device
 * @version  V2.2.0
 * @date     26. May 2021
 ******************************************************************************/
/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

                .syntax  unified
                .arch    armv7-m

                .section .vectors
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                         /*     Top of Stack */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    MemManage_Handler                  /* -12 MPU Fault Handler */
                .long    BusFault_Handler                   /* -11 Bus Fault Handler */
                .long    UsageFault_Handler                 /* -10 Usage Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVC_Handler                        /*  -5 SVC Handler */
                .long    DebugMon_Handler                   /*  -4 Debug Monitor Handler */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

                /* Interrupts */
                .long     WWDGT_IRQHandler                  /* 16:Window Watchdog Timer */
                .long     LVD_IRQHandler                    /* 17:LVD through EXTI Line detect */
                .long     RTC_IRQHandler                    /* 18:RTC through EXTI Line */
                .long     FMC_IRQHandler                    /* 19:FMC */
                .long     RCU_IRQHandler                    /* 20:RCU */
                .long     EXTI0_1_IRQHandler                /* 21:EXTI Line 0 and EXTI Line 1 */
                .long     EXTI2_3_IRQHandler                /* 22:EXTI Line 2 and EXTI Line 3 */
                .long     EXTI4_15_IRQHandler               /* 23:EXTI Line 4 to EXTI Line 15 */
                .long     TSI_IRQHandler                    /* 24:TSI */
                .long     DMA_Channel0_IRQHandler           /* 25:DMA Channel 0  */
                .long     DMA_Channel1_2_IRQHandler         /* 26:DMA Channel 1 and DMA Channel 2 */
                .long     DMA_Channel3_4_IRQHandler         /* 27:DMA Channel 3 and DMA Channel 4 */
                .long     ADC_CMP_IRQHandler                /* 28:ADC and Comparator 0-1 */
                .long     TIMER0_BRK_UP_TRG_COM_IRQHandler  /* 29:TIMER0 Break,Update,Trigger and Commutation */
                .long     TIMER0_Channel_IRQHandler         /* 30:TIMER0 Channel */
                .long     TIMER1_IRQHandler                 /* 31:TIMER1 */
                .long     TIMER2_IRQHandler                 /* 32:TIMER2 */
                .long     TIMER5_DAC_IRQHandler             /* 33:TIMER5 and DAC */
                .long     0                                 /* Reserved */
                .long     TIMER13_IRQHandler                /* 35:TIMER13 */
                .long     TIMER14_IRQHandler                /* 36:TIMER14 */
                .long     TIMER15_IRQHandler                /* 37:TIMER15 */
                .long     TIMER16_IRQHandler                /* 38:TIMER16 */
                .long     I2C0_EV_IRQHandler                /* 39:I2C0 Event */
                .long     I2C1_EV_IRQHandler                /* 40:I2C1 Event */
                .long     SPI0_IRQHandler                   /* 41:SPI0 */
                .long     SPI1_IRQHandler                   /* 42:SPI1 */
                .long     USART0_IRQHandler                 /* 43:USART0 */
                .long     USART1_IRQHandler                 /* 44:USART1 */
                .long     0                                 /* Reserved */
                .long     CEC_IRQHandler                    /* 46:CEC */
                .long     0                                 /* Reserved */
                .long     I2C0_ER_IRQHandler                /* 48:I2C0 Error */
                .long     0                                 /* Reserved */
                .long     I2C1_ER_IRQHandler                /* 50:I2C1 Error */
                .long     I2C2_EV_IRQHandler                /* 51:I2C2 Event */
                .long     I2C2_ER_IRQHandler                /* 52:I2C2 Error */
                .long     USBD_LP_IRQHandler                /* 53:USBD LP */
                .long     USBD_HP_IRQHandler                /* 54:USBD HP */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     USBDWakeUp_IRQHandler             /* 58:USBD Wakeup */
                .long     CAN0_TX_IRQHandler                /* 59:CAN0 TX */
                .long     CAN0_RX0_IRQHandler               /* 60:CAN0 RX0 */
                .long     CAN0_RX1_IRQHandler               /* 61:CAN0 RX1 */
                .long     CAN0_SCE_IRQHandler               /* 62:CAN0 SCE */
                .long     SLCD_IRQHandler                   /* 63:SLCD */
                .long     DMA_Channel5_6_IRQHandler         /* 64:DMA Channel5 and Channel6  */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     SPI2_IRQHandler                   /* 67:SPI2 */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     0                                 /* Reserved */
                .long     CAN1_TX_IRQHandler                /* 86:CAN1 TX */
                .long     CAN1_RX0_IRQHandler               /* 87:CAN1 RX0 */
                .long     CAN1_RX1_IRQHandler               /* 88:CAN1 RX1 */
                .long     CAN1_SCE_IRQHandler               /* 89:CAN1 SCE */
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
Reset_Handler:
                bl       SystemInit

                ldr      r4, =__copy_table_start__
                ldr      r5, =__copy_table_end__

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]                /* source address */
                ldr      r2, [r4, #4]            /* destination address */
                ldr      r3, [r4, #8]            /* word count */
                lsls     r3, r3, #2              /* byte count */

.L_loop0_0:
                subs     r3, #4                  /* decrement byte count */
                ittt     ge
                ldrge    r0, [r1, r3]
                strge    r0, [r2, r3]
                bge      .L_loop0_0

                adds     r4, #12
                b        .L_loop0
.L_loop0_done:

                ldr      r3, =__zero_table_start__
                ldr      r4, =__zero_table_end__

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]                /* destination address */
                ldr      r2, [r3, #4]            /* word count */
                lsls     r2, r2, #2              /* byte count */
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4                  /* decrement byte count */
                itt      ge
                strge    r0, [r1, r2]
                bge      .L_loop2_0

                adds     r3, #8
                b        .L_loop2
.L_loop2_done:

                bl       _start

                .fnend
                .size    Reset_Handler, . - Reset_Handler

/* The default macro is not used for HardFault_Handler
 * because this results in a poor debug illusion.
 */
                .thumb_func
                .type    HardFault_Handler, %function
                .weak    HardFault_Handler
                .fnstart
HardFault_Handler:
                b        .
                .fnend
                .size    HardFault_Handler, . - HardFault_Handler

                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm


/* Default exception/interrupt handler */

                Set_Default_Handler  NMI_Handler
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

                Set_Default_Handler  WWDGT_IRQHandler
                Set_Default_Handler  LVD_IRQHandler
                Set_Default_Handler  RTC_IRQHandler
                Set_Default_Handler  FMC_IRQHandler
                Set_Default_Handler  RCU_IRQHandler
                Set_Default_Handler  EXTI0_1_IRQHandler
                Set_Default_Handler  EXTI2_3_IRQHandler
                Set_Default_Handler  EXTI4_15_IRQHandler
                Set_Default_Handler  TSI_IRQHandler
                Set_Default_Handler  DMA_Channel0_IRQHandler
                Set_Default_Handler  DMA_Channel1_2_IRQHandler
                Set_Default_Handler  DMA_Channel3_4_IRQHandler
                Set_Default_Handler  ADC_CMP_IRQHandler
                Set_Default_Handler  TIMER0_BRK_UP_TRG_COM_IRQHandler
                Set_Default_Handler  TIMER0_Channel_IRQHandler
                Set_Default_Handler  TIMER1_IRQHandler
                Set_Default_Handler  TIMER2_IRQHandler
                Set_Default_Handler  TIMER5_DAC_IRQHandler
                Set_Default_Handler  TIMER13_IRQHandler
                Set_Default_Handler  TIMER14_IRQHandler
                Set_Default_Handler  TIMER15_IRQHandler
                Set_Default_Handler  TIMER16_IRQHandler
                Set_Default_Handler  I2C0_EV_IRQHandler
                Set_Default_Handler  I2C1_EV_IRQHandler
                Set_Default_Handler  SPI0_IRQHandler
                Set_Default_Handler  SPI1_IRQHandler
                Set_Default_Handler  USART0_IRQHandler
                Set_Default_Handler  USART1_IRQHandler
                Set_Default_Handler  CEC_IRQHandler
                Set_Default_Handler  I2C0_ER_IRQHandler
                Set_Default_Handler  I2C1_ER_IRQHandler
                Set_Default_Handler  I2C2_EV_IRQHandler
                Set_Default_Handler  I2C2_ER_IRQHandler
                Set_Default_Handler  USBD_LP_IRQHandler
                Set_Default_Handler  USBD_HP_IRQHandler
                Set_Default_Handler  USBDWakeUp_IRQHandler
                Set_Default_Handler  CAN0_TX_IRQHandler
                Set_Default_Handler  CAN0_RX0_IRQHandler
                Set_Default_Handler  CAN0_RX1_IRQHandler
                Set_Default_Handler  CAN0_SCE_IRQHandler
                Set_Default_Handler  SLCD_IRQHandler
                Set_Default_Handler  DMA_Channel5_6_IRQHandler
                Set_Default_Handler  SPI2_IRQHandler
                Set_Default_Handler  CAN1_TX_IRQHandler
                Set_Default_Handler  CAN1_RX0_IRQHandler
                Set_Default_Handler  CAN1_RX1_IRQHandler
                Set_Default_Handler  CAN1_SCE_IRQHandler


                .end
