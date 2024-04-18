# RS485 Communication with Energy Meter To Write and Read a Holding Register

This example shows how to read and write holding registers for energy meter using RS485 interface and display them to character display or console. It's a basic setup that you can build upon for more advanced serial communication applications.

## How to use example

### Hardware Required

The example can be run on any commonly available STM32 based development board. You will need a USB cable to connect the
development board to a computer.

### Setup the Hardware

- RS485 Interface - UART4_RX(pin79), UART4_TX(pin78) and UART2_RTS(pin77) 
- I2C Interface - I2C1_SDA(pin92), I2C1_SCL(pin93).

## Example Output

````bash

HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, 32);

- It enables the UART to receive data until an idle line condition is detected, indicating the end of the transmission.

- Display the Energy meter output parameters on the console and character display.

````
