# BLDC_MOTOR_CONTROLLER

Use Stm32CubeIDE software and import "bldc_test" project

Controller used in schematic "STM32F407 DISCOVERY"

Controller used for source code "STM32F401 Nucleo-64"

#### Note
Since STM32F407 DISCOVERY is damaged I used STM32F401 Nucleo-64 to write code.
So, I have labeled the pin names of the schematic in the "bldc_test.ioc" file to relate between
the two controllers</br>
</br>
Label: Discovery --> Nucleo</br>
PH_A: PB_9 --> PB_0</br>
PH_B: PE_6 --> PB_1</br>
PH_C: PA_1 --> PB_2</br>
PL_A: PB_8 --> PA_8</br>
PL_B: PE_5 --> PA_9</br>
PL_C: PA_0 --> PA_10</br>
ZPH_A: PE_9 --> PA_0</br>
ZPH_B: PE_11 --> PA_1</br>
ZPH_C: PE_13 --> PB_10</br>
POT_ANALOG: PA_7 --> PA_4</br>

#### Important files
1. main.h
2. main.c
3. stm32f4xx_it.c
4. bldc_test.ioc

#### Reference Docs
1. https://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en012037<br/>
2. https://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en546013<br/>
3. Ready-made board: https://www.st.com/en/ecosystems/x-nucleo-ihm08m1.html<br/>
