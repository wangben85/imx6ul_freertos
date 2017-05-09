Overview
========
The QR decoder demo shows how to decode a bar code or QR code with iMX6UL.
This example is based on the iMX6UL EVK and LCD8000-43T, the camera usd in
the demo is ov5640.

Toolchain supported
===================
- IAR embedded Workbench 7.80.2

Hardware requirements
=====================
- Micro USB cable
- MCIMX6UL EVK
- JLink Plus
- 5V power supply
- Personal Computer
- LCD8000-43T LCD board
- OV5640 camera module

Board settings
==============
1. Connect the LCD8000-43T board to J901.
2. Connect the OV5640 camera module to J1801.
3. By default, JTAG is dedicated with Audio CODEC signals. So please
   Remove R1407,R1431~R1434 resistors while using JTAG debug.

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
5.  Launch the debugger in your IDE to begin running the demo.

Running the demo
================
When the demo runs successfully, the camera received pictures are shown in the LCD.
If the correct bar/QR code is scanned correctly, the serial terminal will output the results like below:

Zxing Decode successfully!
Format: UPC_A
001234567895
Zxing Decode successfully!
Format: QR_CODE
http://weibo.cn/qr/userinfo?uid=1656835961

Customization options
=====================

