/*!
    \file    main.c
    \brief   description of SPI block operation

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
#include "lcd_driver.h"
#include "gui.h"
#include "systick.h"
#include "spi_sd.h"
#include "gd32f150r_eval.h"

uint8_t write_array[2048];
uint8_t read_array[2048];

ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length);
void led_init(void);
void led_flash(uint32_t times);
void display_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{   
    uint8_t status = 0;
    uint32_t i=0;
    /* enable peripheral */ 
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOF);
    /* initializes LED */ 
    led_init();
    /* configure the systick */
    systick_config();
    /* flash LED */
    led_flash(1);
    /* initializes SPI TFT */
    lcd_init();
    /* initializes TFT display */
    lcd_clear(BLUE);
    display_init();
    /* initializes write_array */
    for(i=0;i<2048;i++)
        write_array[i]= i%256;
    /* initializes sd card */
    status = sd_card_init();
    /* if SD card init success */
    if (SD_RESPONSE_NO_ERROR == status){
        /* write 4 block of 2048 bytes on address 0 */
        status = sd_multiblocks_write(write_array, 0, 512,4);
        /* read 4 block of 2048 bytes from address 0 */
        status = sd_multiblocks_read(read_array, 0, 512,4);
        /* check the correctness of written data */
        if(SUCCESS == memory_compare(write_array, read_array, 512)){
            SD_CS_HIGH();
            gui_draw_font_gbk24(2, 240, YELLOW,BLUE, "    Result : Passed !");
            SD_CS_HIGH();
        }else{
            SD_CS_HIGH();
            gui_draw_font_gbk24(2, 240, YELLOW,BLUE, "    Result : Failed !");
            SD_CS_HIGH();
        }
    }else
        gui_draw_font_gbk24(2, 240, RED, BLUE, "SD Card Initializes Failed !");
    while (1);
}

/*!
    \brief      led initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_init( void )
{
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_init(LED4);
}

/*!
    \brief      led flash
    \param[in]  times : specifies the delay time length, in milliseconds
    \param[out] none
    \retval     none
*/
void led_flash(uint32_t times)
{
    uint32_t i;
    for( i = 0 ; i < times ; i ++ ){
        /* insert 200 ms delay */
        delay_1ms(200);
        /* turn on LED */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED3);
        gd_eval_led_on(LED4);
        /* insert 200 ms delay */
        delay_1ms(200);
        /* turn off LED */
        gd_eval_led_off(LED1);
        gd_eval_led_off(LED2);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED4);
         
    }
}

/*!
    \brief      memory compare function
    \param[in]  src : source data pointer
    \param[in]  dst : destination data pointer
    \param[in]  length : the compare data length
    \param[out] none
    \retval     ErrStatus : ERROR or SUCCESS
*/
ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length) 
{
    while (length--){
        if (*src++ != *dst++)
            return ERROR;
    }
    return SUCCESS;
}

/*!
    \brief      display initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void display_init( void )
{
    gui_draw_font_gbk16(2, 10, WHITE,BLUE, " GigaDevice Semiconductor Inc.");
    gui_draw_font_gbk16(2, 30, WHITE,BLUE, "  -- GD32F1x0 Series MCU --   ");
    gui_draw_font_gbk16(2, 50, WHITE,BLUE, "     GD32F150R_EAVL_V1.3  ");
    gui_draw_font_gbk16(2, 70, WHITE,BLUE, " SPI SD Card Block Test :");
    gui_draw_font_gbk16(2, 90, WHITE,BLUE, " Please insert the SD Card !");

    gui_box2(10, 120, 62, 80, 2);
    gui_draw_line(64, 130, 64, 190, WHITE);
    gui_draw_line(64, 130, 72, 120, WHITE);
    gui_draw_line(64, 190, 72, 200, WHITE);

    gui_rect(134, 135, 214, 185, WHITE);
    delay_1ms(200);
    gui_rect(134, 135, 214, 185, BLUE);
    gui_rect(104, 135, 184, 185, WHITE);
    delay_1ms(200);
    gui_rect(104, 135, 184, 185, BLUE);
    gui_rect(74, 135, 134, 185, WHITE);
}
