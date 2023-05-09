/*!
    \file    main.c
    \brief   GPIO running led

    \version 2016-01-15, V1.0.0, demo for GD32F1x0
    \version 2016-05-13, V2.0.0, demo for GD32F1x0
    \version 2019-11-20, V3.0.0, demo for GD32F1x0
    \version 2021-12-31, V3.1.0, demo for GD32F1x0
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

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
#include "gd32f1x0_libopt.h"
#include "systick.h"
#include "gd32f150r_eval.h"
#include <stdio.h>

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{  
    /* enable the led clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    /* configure led GPIO port */ 
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

    gpio_bit_reset(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    gpio_bit_reset(GPIOD,  GPIO_PIN_2);
    
    systick_config();
    
    while(1){
        /* turn on LED1 */
        gpio_bit_set(GPIOC, GPIO_PIN_10);
        /* insert 200 ms delay */
        delay_1ms(200);
        
        /* turn on LED2 */
        gpio_bit_set(GPIOC, GPIO_PIN_11);
        /* insert 200 ms delay */
        delay_1ms(200);
        
        /* turn on LED3 */
        gpio_bit_set(GPIOC, GPIO_PIN_12);
        /* insert 200 ms delay */
        delay_1ms(200);        
        
        /* turn on LED4 */
        gpio_bit_set(GPIOD, GPIO_PIN_2);
        /* insert 200 ms delay */
        delay_1ms(200);        

        /* turn off LEDs */ 
        gpio_bit_reset(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
        gpio_bit_reset(GPIOD, GPIO_PIN_2);
        
        /* insert 200 ms delay */
        delay_1ms(200);
    }
}
