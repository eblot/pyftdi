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

* UART/Serial USB converter, up to 6 Mbps (depending on the FTDI device
  capability)
* Bitbang/GPIO legacy support, 8 pins per port
* SPI master, with simultanous GPIO support, up to 12 pins per port
* |I2C| master, with simultanous GPIO support, up to 14 pins per port
* (JTAG master)
* Prelimiary EEPROM support (R/O access + R/W for serial/product/manufacturer)

PyFtdi_ provides a pyserial_ compliant API, so it can be used as a drop-in
module to access USB-serial converters based on FTDI_ devices.

Potential API break
-------------------

Starting with version *v0.40.0*, several API changes are being introduced.
While PyFtdi tries to maintain backward compatibility with previous versions,
so of these changes may require existing clients to update calls to PyFtdi.

Do not upgrade to *v0.40.0* or above without testing your client against the
new PyFtdi releases. PyFtdi versions up to *v0.39.9* keep a stable API
with *v0.22+* series.

Major changes
~~~~~~~~~~~~~

 * *read* methods now return ``bytearray`` instead of `Array('B')` so that
   pyserial ``readline()`` may be used. It also brings some performance
   improvements.
 * PyFtdi URLs now supports ``bus:address`` alternative specifiers, which
   required to augment the ``open_*()`` methods with new, optional parameters.
 * ``SpiController`` reserves only one slave line (*/CS*) where it used to
   reserve 4 slave lines in previous releases. This frees more GPIOs when
   default value is used - it is nevertheless still possible to reserve up to 5
   slave lines.
 * type hinting is used for most, if not all, public methods.
 * simplified baudrate divider calculation.

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
   eeprom
   troubleshooting
   authors
   licenses
