# BLDC_MOTOR_CONTROLLER

Use Stm32CubeIDE software and import "bldc_test" project

Controller used in schematic STM32F407 DISCOVERY

Controller used for source code STM32F401 Nucleo-64

#### Note
Since STM32F407 DISCOVERY is damaged I used STM32F401 Nucleo-64 to write code.
So, I have labeled the pin names of the schematic in the "bldc_test.ioc" file to relate between
the two controllers

#### Important files
1. main.h
2. main.c
3. stm32f4xx_it.c
4. bldc_test.ioc

#### Reference Docs
1. https://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en012037<br/>
2. https://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en546013<br/>
  
#### Future updates
There might be updates in hardware and code to implement Field Oriented Control of BLDC
