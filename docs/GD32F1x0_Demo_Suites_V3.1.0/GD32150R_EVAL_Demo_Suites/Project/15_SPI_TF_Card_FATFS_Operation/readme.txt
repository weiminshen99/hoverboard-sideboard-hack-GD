/*!
    \file    readme.txt
    \brief   description of the tf card fatfs file operation by SPI

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

  This demo is based on the GD32150R-EVAL-V1.3 board, it provides a description of how 
to use the SPI to drive TFT LCD and file write and read in TF card with FATFS. 

  After system start-up, all the LEDs are turned on and then turned off for test.
After that, the LCD panel is cleared to blue. If the initialization of SD card is 
wrong, show the fail information on the LCD screen. Otherwise, try to open a file
from SD card. If the result is equal to FR_NO_FILESYSTEM, notify users to format 
the SD card first. If the result is equal to FR_OK, write data to SD card and show 
the information on the LCD screen. If the file is already exist in SD card, display
the file exist information on the LCD screen. At last, read data of FATFS.TXT to
buffer to check whether the result is success or not.

  JP10 to SPI.
  JP11 to SPI.
