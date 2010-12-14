########
 PyFTDI
########

=============
Primary goals
=============

PyFTDI aims at providing a user-space driver for modern FTDI_ devices, 
implemented in pure Python language.

Modern FTDI_ devices include:

* FT2232H (dual port)
* FT4232H (quad port)

Other FTDI_ devices could also been supported (including FT232* devices), 
although these devices are not a primary goal for PyFTDI

It should support the following modes:

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device 
  capability)
* SPI master
* JTAG controller
* Bitbang (GPIO) support (not a primary goal)

.. _FTDI: www.ftdichip.com

============
Requirements
============

PyFTDI relies on PyUSB_, which itself depends on some of the following native
libraries:

* libusb-1.0 (recommended)
* libusb-0.1 (deprecated)
* openusb (not tested)

.. _PyUSB: http://sourceforge.net/projects/pyusb/
