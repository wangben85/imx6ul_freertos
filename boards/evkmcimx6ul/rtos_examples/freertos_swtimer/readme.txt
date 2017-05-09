Overview
========
This document explains the freertos_swtimer example. It shows usage of software timer and its
callback.

The example application creates one software timer SwTimer. The timer’s callback SwTimerCallback is
periodically executed and text “Tick.” is printed to terminal.



Toolchain supported
===================
- IAR embedded Workbench 7.80.4

Hardware requirements
=====================
- Micro USB cable
- MCIMX6UL-EVK  board
- JLink Plus
- 5V power supply
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect 5V power supply and JLink Plus to the board, switch SW2001 to power on the board.
2.  Connect a USB cable between the host PC and the J1901 USB port on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
After the board is flashed the Tera Term will show output message.

Example output:
Tick.
Tick.
Tick.
Customization options
=====================

