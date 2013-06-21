uC/OS-II RTOS on EFM32 using example.

This is a port of the uC/OS-II RTOS from Micrium Inc, http://www.micrium.com

The port has been adapted by Energy Micro for use with our Development
Kit. It requires the standard EFM32 CMSIS package (including emlib).

The example has three tasks:
 - Task1: LED blink task
 - Task2: Receives characters from serial port and posts message to Task3
 - Task3: Receives message from Task2 and writes it on TFT display

Standard output is redirected to TFT display and standard input is UART1.

Note: The Keil toolchain port does not support FPU context preservation
      across tasks.

uC/OS-II is provided in source form for FREE evaluation, for educational use
or for peaceful research.

If you plan on using uC/OS-II in a commercial product you need to contact
Micri�m to properly license its use in your product. We provide ALL the
source code for your convenience and to help you experience uC/OS-II.
The fact that the source is provided does NOT mean that you can use it without
paying a licensing fee.

For commercial use, contact Micrium, http://www.micrium.com for details.

Board:  Energy Micro EFM32WG_DK3850 Starter Kit
Device: EFM32WG990F256
