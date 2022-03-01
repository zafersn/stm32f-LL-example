This example describes how to use a ADC peripheral to perform a single ADC conversion of a channel, at each trigger event from timer.
Conversion data are transferred by DMA into a table, indefinitely (circular mode).
This example is based on the STM32F072x ADC LL API, Peripheral initialization done using LL unitary services functions.

## NOTE: This example converted from STM32G0xx ADC LL API;

* ADC GPIO Configuration

  PA0   ------> ADC_IN0
 
