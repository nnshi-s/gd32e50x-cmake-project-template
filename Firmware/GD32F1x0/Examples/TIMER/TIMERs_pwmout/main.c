/*!
    \file    main.c
    \brief   TIMER1 PWM output demo

    \version 2014-12-26, V1.0.0, platform GD32F1x0(x=3,5)
    \version 2016-01-15, V2.0.0, platform GD32F1x0(x=3,5,7,9)
    \version 2016-04-30, V3.0.0, firmware update for GD32F1x0(x=3,5,7,9)
    \version 2017-06-19, V3.1.0, firmware update for GD32F1x0(x=3,5,7,9)
    \version 2019-11-20, V3.2.0, firmware update for GD32F1x0(x=3,5,7,9)
    \version 2020-09-21, V3.3.0, firmware update for GD32F1x0(x=3,5,7,9)
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f1x0.h"
#include <stdio.h>

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    
    /*configure PA9 PA10 PA11(TIMER0 CH1 CH2 CH3) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9 |GPIO_PIN_10 |GPIO_PIN_11 );
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9 |GPIO_PIN_10 |GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_10);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_11);
    
    /*configure PB3 PB10 PB11(TIMER1 CH1 CH2 CH3) as alternate function*/
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3 |GPIO_PIN_10 |GPIO_PIN_11 );
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 |GPIO_PIN_10 |GPIO_PIN_11);
    gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_3);
    gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_10);
    gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_11);
    
}

/*!
    \brief      configure the TIMER0 peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer0_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER0 configuration: generate 3 PWM signals with 3 different duty cycles:
    TIMER0CLK = SystemCoreClock / 72 = 1MHz

    TIMER0 channel 1 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER0 channel 2 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER0 channel 3 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER0);

    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 71;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 15999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0,&timer_initpara);

    /* CH1, CH2 and CH3 configuration in PWM0 mode */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;

    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);

    /* CH1 configuration in PWM mode1,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 3999);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode1,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 7999);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* CH3 configuration in PWM mode1,duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 11999);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);
    
    /* enable TIMER0 primary output */
    timer_primary_output_config(TIMER0,ENABLE);
    
    /* auto-reload preload enable */
    timer_enable(TIMER0);
}
/*!
    \brief      configure the TIMER1 peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer1_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 3 PWM signals with 3 different duty cycles:
    TIMER1CLK = SystemCoreClock / 72 = 1MHz

    TIMER1 channel 1 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER1 channel 2 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER1 channel 3 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 71;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 15999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH1, CH2 and CH3 configuration in PWM0 mode */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;

    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(TIMER1, TIMER_CH_3, &timer_ocintpara);

    /* CH1 configuration in PWM mode1,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 3999);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode1,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 7999);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* CH3 configuration in PWM mode1,duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, 11999);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* auto-reload preload enable */
    timer_enable(TIMER1);
}
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    gpio_config();
    timer0_config();
    timer1_config();
    while (1);
}
