PyFtdi
======

.. include:: defs.rst

Overview
--------

PyFtdi_ aims at providing a user-space driver for modern FTDI_ devices,
implemented in pure Python language.

Modern FTDI_ devices include:

* UART-only bridges

  * FT232R_ (single port, clock up to 6 MHz, 3Mbps)
  * FT230X_ (single port, clock up to 48 Mhz, 3Mbps)

* UART and multi-serial protocols (SPI, |I2C|, JTAG) bridges

  * FT2232D_ (dual port, clock up to 6 MHz)
  * FT232H_ (single port, clock up to 30 MHz)
  * FT2232H_ (dual port, clock up to 30 MHz)
  * FT4232H_ (quad port, clock up to 30 MHz)

Other FTDI_ devices could also be supported (including FT232* devices),
although these devices are not a primary goal for PyFtdi_, and therefore have
not been tested with PyFtdi_.

Primary goals
-------------

PyFtdi_ currently supports the following features:

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device
  capability)
* Bitbang/GPIO support
* SPI master
* |I2C| master
* (JTAG master)

PyFtdi_ provides a pyserial_ compliant API, so it can be used as a drop-in
module to access USB-serial converters based on FTDI_ devices.

PyFTDI in details
-----------------

.. toctree::
   :maxdepth: 1
   :glob:

   features
   requirements
   installation
   urlscheme
   api/index
   pinout
   troubleshooting
   authors
   licenses
