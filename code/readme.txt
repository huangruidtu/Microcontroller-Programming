########################################################################
#
#                           TouchDemo.eww
#
# $Revision: 28 $
#
########################################################################

DESCRIPTION
===========
  This example project shows how to use the IAR Embedded Workbench for ARM
 to develop code for IAR LPC2478-SK board. It shows basic use of I/O,
 timers, interrupts, LCD controllers and LCD touch screen.

  A cursor is shown and moves when the screen is touched.
  
COMPATIBILITY
=============

   The Touch Screen demo project is compatible with IAR LPC2478-SK
  evaluation board. By default, the project is configured to use the
  J-Link JTAG interface.

CONFIGURATION
=============

   After power-up the controller get it's clock from internal RC oscillator that
  is unstable and may fail with J-Link auto detect, therefore adaptive clocking
  should always be used. The adaptive clock can be select from menu:
  Project->Options..., section Debugger->J-Link/J-Trace  JTAG Speed - Adaptive.

   The Touch demo application is downloaded to the iFlash.
  
  Jumpers:
    EXT/JLINK  - depending of power source
    ISP_E      - unfilled
    RST_E      - unfilled
    BDS_E      - unfilled
    C/SC       - SC

  The LCD shares pins with Trace port. If ETM is enabled the LCD will not work.
  
GETTING STARTED
===============

  1) Start the IAR Embedded Workbench for ARM.

  2) Select File->Open->Workspace...
     Open the following workspace:

      <installation-root>\arm\examples\NXP\
     LPC24xx\IAR-LPC-2478\TouchDemo\TouchDemo.eww

  3) Run the program.
