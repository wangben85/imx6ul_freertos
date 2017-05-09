Overview
========
The dvfs_simulation demo application provides an example to simulate the Dynamic Voltage and Frequency Scaling(DVFS) function on IMX6UL EVB.
The dvfs_simulation demo uses the SW3(SW-1,SW-2) on EVB to change the CPU loading, then the ARM\CPU frequency and vDD_ARM_CAP voltage will update accordingly.

===================
- IAR embedded Workbench 7.80.4

Hardware requirements
=====================
- Micro USB cable
- MCIMX6UL-EVB  board
- JLink Plus
- 5V power supply
- Personal Computer

Board settings
==============
No special is needed.

Prepare the Demo
================
1.  Connect 5V power supply and JLink Plus to the board, switch SW2001 to power on the board
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
When the example runs successfully, following information can be seen on the terminal:

~~~~~~~~~~~~~~~~~~~~~
CPU Core Clock = 396000000Hz, ARM voltage = 1.000000V
Hello_task		55162		7%
IDLE			664215          92%
Tmr Svc			0		<1%

CPU Core Clock = 198000000Hz, ARM voltage = 0.925000V
Hello_task		56878		6%
IDLE			787265		93%
Tmr Svc			0		<1%

CPU Core Clock = 528000000Hz, ARM voltage = 1.15000V
Hello_task		102573		12%
IDLE			814322		87%
Tmr Svc			0		<1%

~~~~~~~~~~~~~~~~~~~~~
Customization options
=====================
Change the state of SW3-1 on IMX6UL_EVB, will decrease the CPU loading, as a result the CPU frequency and VDD_ARM_CAP will decrease too;
Change the state of SW3-2 on IMX6UL_EVB, will increase the CPU loading, as a result the CPU frequency and VDD_ARM_CAP will increase too;
The example frequency and voltage setpoint comes from AN5170 as below:
ARM Freeuency		LDO State	VDD_ARM_CAP
   528Mhz		 Enable		    1.150V
   396Mhz		 Enable		    1.000V
   198Mhz		 Enable		    0.925V