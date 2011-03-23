========
 PyFtdi
========

--------
Overview
--------

PyFtdi aims at providing a user-space driver for modern FTDI_ devices,
implemented in pure Python language.

Modern FTDI_ devices include:

* FT2232H (dual port)
* FT4232H (quad port)

Other FTDI_ devices could also been supported (including FT232* devices),
although these devices are not a primary goal for PyFtdi, and therefore have
not been tested with PyFtdi.

-------------
Primary goals
-------------

It should support the following modes:

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device
  capability)
* SPI master
* JTAG master
* Bitbang/GPIO support (not a primary goal)

PyFtdi should provide a pyserial_ compliant API, so it can be used as a
drop-in module to access USB-serial converters based on FTDI_ devices.

.. _FTDI: http://www.ftdichip.com/
.. _pyserial: http://pyserial.sourceforge.net/


------------
Requirements
------------

PyFtdi relies on PyUSB_, which itself depends on one of the following native
libraries:

* libusb-1.0 (recommended)
* libusb-0.1 (deprecated)
* openusb (not tested with pyftdi)

PyFtdi does not depend on any other native library, and only uses standard
Python modules.

Python_ 2.6 or above is required. Python_ 3.x is not yet supported.

.. _PyUSB: http://sourceforge.net/projects/pyusb/
.. _Python: http://python.org/


------
Status
------

This project is still in an early alpha development stage.

However, PyFtdi is being forked from a closed-source software implementation
that has been successfully used for over a year - including serial, spi and
jtag protocols, based on top of the libftdi_ open source library.

libftdi_ is now being phased out from this closed-source project and replaced
with PyFtdi, to ease maintenance and customization.

Meanwhile, PyFtdi is developed as an open-source solution.

Supported features
------------------
* All FTDI device ports (UART, MPSSE) can be used simultaneously.
* Serial port, up to 12 Mbps. PyFtdi includes a pyserial_ emulation layer that
  offers transparent access to the FTDI serial ports through a pyserial_-
  compliant API. `serialext` directory contains a minimal serial terminal
  that demonstrates the use of this extension, and a dispatcher that
  automatically selects the serial backend (pyserial_, PyFtdi), depending on
  the serial port name.
* SPI master. PyFtdi includes several examples that demonstrate how to use
  the FTDI SPI master, with a pure-Python serial flash device driver for
  several common serial flash devices.
  These tests show an average 400 KB/s read out from the flash devices running
  with a 6MHz SPI clock on a Core2Duo Mac Book Pro.

.. _libftdi: http://www.intra2net.com/en/developer/libftdi/

-----------
Development
-----------

PyFtdi is developed on Mac OS X platforms (including 64-bit kernels), and is
validated on a regular basis on Linux hosts.

As it contains no native code, it should work on any platforms PyUSB_ and
libusb_ support (including, but not limited to, Windows).

.. _libusb: http://www.libusb.org/
