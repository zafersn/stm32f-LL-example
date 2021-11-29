# stm32f072B-Disco-LL-example
This example is based on the STM32f072B ST low level API. The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size). 
In this repository, you will find an example in periharels/modules below

* Periharels <br>
✓ I2C (SSD1306 Screen) <br>
✓ TIM (To obtain 1us timer)<br>
✓ GPIO (To write and read gpio pin value for DHT11)<br>

* Modules<br>
✓ SSD1306 Screen 128x32<br>
✓ DHT11 Temperature and humidity sensor<br>

The timebase frequency is calculated as follows:
timebase frequency = TIM1 counter clock /((PSC + 1)*(ARR + 1)*(RCR + 1))

## I2C (SSD1306 Screen) <br>

[![logo](https://github.com/zafersn/stm32f-LL-example/blob/main/072B-Disco/update-screen.gif)](https://youtu.be/smKaRiu-GGo)

## GPIO (To write and read gpio pin value for DHT11)<br>

[![logo](https://github.com/zafersn/stm32f-LL-example/blob/main/072B-Disco/temperature.gif)](https://youtu.be/smKaRiu-GGo)

