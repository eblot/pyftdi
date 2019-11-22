PyFtdi
======

Documentation
-------------

PyFtdi documentation is available from https://eblot.github.io/pyftdi/

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
* Bitbang/GPIO legacy support, 8 pins per port
* SPI master, with simultanous GPIO support, up to 12 pins per port
* |I2C| master, with simultanous GPIO support, up to 14 pins per port
* (JTAG master)
* Prelimiary EEPROM support (R/O access + R/W for serial/product/manufacturer)

Warning
-------

Starting with version *v0.40.0*, several API changes are being introduced.
While PyFtdi tries to maintain backward compatibility with previous versions,
some of these changes may require existing clients to update calls to PyFtdi.

Do not upgrade to *v0.40.0* or above without testing your client against the
new PyFtdi releases. PyFtdi versions up to *v0.39.9* keep a stable API
with *v0.22+* series.

See the *Major Changes* section on the online documentation for details about
potential API breaks.
