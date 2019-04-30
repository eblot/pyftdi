PyFtdi
======

Overview
--------

PyFtdi aims at providing a user-space driver for modern FTDI devices,
implemented in pure Python language.

Modern FTDI devices include:

* UART-only bridges

  * FT232R (single port, clock up to 6 MHz, 3Mbps)
  * FT230X (single port, clock up to 48 Mhz, 3Mbps)

* UART and multi-serial protocols (SPI, |I2C|, JTAG) bridges

  * FT2232D (dual port, clock up to 6 MHz)
  * FT232H (single port, clock up to 30 MHz)
  * FT2232H (dual port, clock up to 30 MHz)
  * FT4232H (quad port, clock up to 30 MHz)

Other FTDI devices could also be supported (including FT232* devices),
although these devices are not a primary goal for PyFtdi, and therefore have
not been tested with PyFtdi.

Primary goals
-------------

PyFtdi currently supports the following features:

.. |I2C| replace:: I\ :sup:`2`\ C

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device
  capability)
* Bitbang/GPIO support
* SPI master
* |I2C| master
* JTAG master

PyFtdi provides a pyserial compliant API, so it can be used as a drop-in
module to access USB-serial converters based on FTDI devices.


Documentation
-------------

PyFtdi documentation is available from https://eblot.github.io/pyftdi/
