/*!
    \file    I2C0_IE.c
    \brief   I2C0 master receiver interrupt program

    \version 2017-06-06, V1.0.0, firmware for GD32F3x0
    \version 2019-06-01, V2.0.0, firmware for GD32F3x0
    \version 2020-09-30, V2.1.0, firmware for GD32F3x0
    \version 2022-01-06, V2.2.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

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

#include "gd32f3x0_i2c.h"
#include "I2C_IE.h"

uint32_t event1;

/*!
    \brief      handle I2C0 event interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_EventIRQ_Handler(void)
{
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SBSEND)) {
        /* the master sends slave address */
        i2c_master_addressing(I2C0, I2C1_SLAVE_ADDRESS7, I2C_RECEIVER);
    } else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_ADDSEND)) {
        if((1 == I2C_nBytes) || (2 == I2C_nBytes)) {
            /* clear the ACKEN before the ADDSEND is cleared */
            i2c_ack_config(I2C0, I2C_ACK_DISABLE);
            /* clear the ADDSEND bit */
            i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_ADDSEND);
        } else {
            /* clear the ADDSEND bit */
            i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_ADDSEND);
        }
    } else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_RBNE)) {
        if(I2C_nBytes > 0) {
            if(3 == I2C_nBytes) {
                /* wait until the second last data byte is received into the shift register */
                while(!i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BTC));
                /* send a NACK for the last data byte */
                i2c_ack_config(I2C0, I2C_ACK_DISABLE);
            }
            /* read a data byte from I2C_DATA*/
            *i2c_rxbuffer++ = i2c_data_receive(I2C0);
            I2C_nBytes--;
            if(0 == I2C_nBytes) {
                /* send a stop condition */
                i2c_stop_on_bus(I2C0);
                status = SUCCESS;
                i2c_ack_config(I2C0, I2C_ACK_ENABLE);
                i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
                /* disable the I2C0 interrupt */
                i2c_interrupt_disable(I2C0, I2C_INT_ERR);
                i2c_interrupt_disable(I2C0, I2C_INT_BUF);
                i2c_interrupt_disable(I2C0, I2C_INT_EV);
            }
        }
    }
}

/*!
    \brief      handle I2C0 error interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_ErrorIRQ_Handler(void)
{
    /* no acknowledge received */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_AERR)) {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_AERR);
    }

    /* SMBus alert */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBALT)) {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBALT);
    }

    /* bus timeout in SMBus mode */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBTO)) {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBTO);
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_OUERR)) {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_OUERR);
    }

    /* arbitration lost */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_LOSTARB)) {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_LOSTARB);
    }

    /* bus error */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BERR)) {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_BERR);
    }

    /* CRC value doesn't match */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_PECERR)) {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_PECERR);
    }

    /* disable the I2C0 interrupt */
    i2c_interrupt_disable(I2C0, I2C_INT_ERR);
    i2c_interrupt_disable(I2C0, I2C_INT_BUF);
    i2c_interrupt_disable(I2C0, I2C_INT_EV);
}
