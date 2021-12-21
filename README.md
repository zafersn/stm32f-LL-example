# stm32f072B-Disco-LL-example
This example is based on the STM32f072B ST low level API. The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size). 
In this repository, you will find an example in periharels/modules below

Table of contents
=================

<!--ts-->
  * [st-ll-1](#st-ll-1)
    *  I2C, TIM, GPIO(DHT11), UART examples.
  * [st-ll-2](#st-ll-2)
    *   PWM, GPIO-EXTI, I2C examples.
  * [st-ll-3](#st-ll-3)
    *   UART-HALF-DUPLEX  example
<!--te-->


# st-ll-1
* Periharels <br>
  * ✓ 1- I2C (SSD1306 Screen) <br>
  * ✓ 2- TIM (To obtain 1us timer)<br>
    * The timebase frequency is calculated as follows:
    * timebase frequency = TIM6 counter clock /((PSC + 1)*(ARR + 1)*(RCR + 1)) 
    * timebase frequency = 48 000 000 / ( (47 + 1) * (65535 + 1) (0 + 1) )
    * timebase frequency = 15.258 = ~15 hz

  * ✓ 3- GPIO (To write and read gpio pin value for DHT11)<br>
    * Using Single pin (PB2) in two different mode (Input- Output)
* Modules<br>
  * ✓ SSD1306 Screen 128x32<br>  
    * PB10 -> SCLK <br>
    * PB11 -> SDA <br>
  * ✓ DHT11 Temperature and humidity sensor<br>
    * PB2 -> DHT11 In

## Pin Configuration <br>
![logo](https://github.com/zafersn/stm32f-LL-example/blob/main/072B-Disco/st-ll-1-pinout.PNG)


## I2C (SSD1306 Screen) <br>

[![logo](https://github.com/zafersn/stm32f-LL-example/blob/main/072B-Disco/update-screen.gif)](https://youtu.be/smKaRiu-GGo)

## GPIO (To write and read gpio pin value for DHT11)<br>

[![logo](https://github.com/zafersn/stm32f-LL-example/blob/main/072B-Disco/temperature.gif)](https://youtu.be/smKaRiu-GGo)

# st-ll-2

## Pin Configuration <br>
![logo](https://github.com/zafersn/stm32f-LL-example/blob/main/072B-Disco/st-ll-2-pinout.PNG)

# st-ll-3
This example is one-board UART half-duplex(single-wire) communication using ST LL and HAL API on stm32f072-disco <br>
*  PC4 - UART3 
*  PA9 - UART1
*  38400 B
*  

## Pin Configuration <br>
