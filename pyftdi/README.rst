========
 PyFtdi
========

Overview
~~~~~~~~

PyFtdi_ aims at providing a user-space driver for modern FTDI_ devices,
implemented in pure Python language.

Modern FTDI_ devices include:

* FT232R (single port, clock up to 6 MHz, 3Mbps)
* FT2232D (dual port, clock up to 6 MHz)
* FT232H (single port, clock up to 30 MHz)
* FT2232H (dual port, clock up to 30 MHz)
* FT4232H (quad port, clock up to 30 MHz)
* FT230X (single port, clock up to 48 Mhz, 3Mbps)

Other FTDI_ devices could also be supported (including FT232* devices),
although these devices are not a primary goal for PyFtdi_, and therefore have
not been tested with PyFtdi_.

Primary goals
~~~~~~~~~~~~~

PyFtdi_ currently supports the following features:

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device
  capability)
* Bitbang/GPIO support
* SPI master
* I2C master
* JTAG master

PyFtdi_ provides a pyserial_ compliant API, so it can be used as a drop-in
module to access USB-serial converters based on FTDI_ devices.

Requirements
~~~~~~~~~~~~

Python_ 3.5 or above is required.

PyFtdi_ relies on PyUSB_, which itself depends on one of the following native
libraries:

* libusb_, tested with 1.0.20

may still work, but are fully untested there are nowaways obsolete.

PyFtdi_ does not depend on any other native library, and only uses standard
Python modules along with PyUSB_

PyFTDI_ has been tested with PyUSB_ 1.0.0. PyUSB_ 1.0.0b1 or below is no longer
supported.

Note about previous releases
----------------------------

``open()``, ``open_mpsse()`` and ``open_bitbang`` arguments have changed in
v0.22.0, be sure to update your code or even better use the URL variants
(``open_from_url``, ``open_mpsse_from_url`` or ``open_bitbang_from_url``).

If you have no choice but using previous releases of software, such as

* Python_ (2.6+, 3.3+),
* other PyUSB_ backends such as the deprecated libusb-0.1, or openusb,
* PyUSB_ 1.0.0b1 or below,
* pyserial_ 2.6+ (previous versions of pyserial_ will NOT work)

please checkout the latest PyFTDI_ 0.1x series (0.13.3) which provides support
for these deprecated environmement, but is no longer actively maintained.

Status
~~~~~~

This project is still in beta development stage.

However, PyFtdi_ is being forked from a closed-source software implementation
that has been successfully used for over several years - including serial
@ 3Mbps, spi and jtag protocols. PyFtdi_ is developed as an open-source
solution.

Supported features
------------------

* All FTDI device ports (UART, MPSSE) can be used simultaneously.

* Several FTDI adapters can be accessed simultaneously from the same Python
  runtime instance.

* Serial port, up to 12 Mbps. PyFtdi_ includes a pyserial_ emulation layer that
  offers transparent access to the FTDI serial ports through a pyserial_-
  compliant API. The ``serialext`` directory contains a minimal serial terminal
  demonstrating the use of this extension, and a dispatcher automatically
  selecting the serial backend (pyserial_, PyFtdi_), based on the serial port
  name.

* SPI master. For now, SPI Mode 0 (CPOL=0, CPHA=0) is the only supported
  mode. It should be easy to extend the SPI master to deal with less common
  modes. PyFtdi_ can be used with pyspiflash_ module that demonstrates how to
  use the FTDI SPI master with a pure-Python serial flash device driver for
  several common devices.

* I2C master. For now, only 7-bit address are supported.

* JTAG is under development and is not fully supported yet.

Installation
~~~~~~~~~~~~

* Install native dependency. The actual command to install depends on your OS
  and/or your distribution. Examples:

  * Debian Linux

      apt-get install libusb-1.0

  * Homebrew macOS

      brew install libusb

* Install Python dependencies

    pip3 install pyusb
    pip3 install pyserial
    pip3 install pyftdi_

Troubleshooting
---------------

*"Error: No backend available"*
  libusb native library cannot be loaded. Try helping the dynamic loader:

  * On Linux: ``export LD_LIBRARY_PATH=<path>``

    where ``<path>`` is the directory containing the ``libusb-1.*.so``
    library file

  * On macOS: ``export DYLD_LIBRARY_PATH=.../lib``

    where ``<path>`` is the directory containing the ``libusb-1.*.dylib``
    library file

*"Error: Access denied (insufficient permissions)"*
  The system may already be using the device.

  * On OS X 10.9+: starting with Mavericks, OS X ships with a native FTDI
    driver that preempts access to the FTDI device.

    The driver can be unloaded this way:

      ``sudo kextunload [-v] -bundle com.apple.driver.AppleUSBFTDI``

    You may want to use an alias or a tiny script such as
    ``pyftdi/tools/uphy.sh``

    Please note that the system automatically reloads the driver, so it may be
    useful to move the kernel extension so that the system never loads it.

  * This error message may also be triggered whenever the communication port is
    already in use.

*"serial.serialutil.SerialException: Unable to open USB port"*
  May be caused by a conflict with the FTDI virtual COM port (VCOM). Try
  uninstalling the driver. On macOS, refer to this FTDI macOs
  `guide <http://www.ftdichip.com/Support/Documents/AppNotes/AN_134_FTDI_Drivers_Installation_Guide_for_MAC_OSX.pdf>`_.

*Slow initialisation on OS X El Capitan*
 It may take several seconds to open or enumerate FTDI devices.

 If you run libusb <= v1.0.20, be sure to read the
 `issue <https://github.com/libusb/libusb/commit/5e45e0741daee4fa295c6cc977edfb986c872152>`_
 with OS X 10.11+.

URL Scheme
~~~~~~~~~~

There are generally two ways to open a connection to an Ftdi() object. The
first method is to use the ``open()`` methods which accept VID, PID, and serial
parameters (among others). These methods are:

* ``open()``
* ``open_mpsse()``
* ``open_bitbang()``

The second way to open a connection is to specify connection details using a
URL. The URL scheme is defined as:

``protocol://[vendor[:product[:index|:serial]]]/interface``

Where:

* protocol: always ``ftdi``
* vendor: the USB vendor ID of the manufacturer

  * ex: ``ftdi`` or ``0x403``

* product: the USB product ID of the device

  * ex: ``232h`` or ``0x6014``
  * Supported product IDs: ``0x6001``, ``0x6010``, ``0x6011``, ``0x6014``, ``0x6015``
  * Supported product aliases:

    * ``232``, ``232r``, ``232h``, ``2232d``, ``2232h``, ``4232h``, ``230x``
    * ``ft`` prefix for all aliases is also accepted, as for example ``ft232h``

* serial: the serial number as a string
* index: an integer (not particularly useful, as it depends on the enumeration
  order on the USB buses)
* interface: the interface of FTDI device, starting from 1

  * ex: ``1`` for 232\*, ``1`` or ``2`` for 2232\*, ``1``-``4`` for 4232\* devices

All parameters but the interface are optional, PyFtdi tries to find the best
match. Therefore, if you have a single FTDI device connected to your system,
``ftdi:///1`` should be enough.

You can also ask PyFtdi to enumerate all the compatible devices with the
special ``ftdi:///?`` syntax.

URLs can be used with the same methods as above by appending ``_from_url`` to
the method name such as:

* ``open_from_url()``
* ``open_mpsse_from_url()``
* ``open_bitbang_from_url()``

Development
~~~~~~~~~~~

PyFtdi_ is developed on macOS platforms (64-bit kernel), and is validated on a
regular basis on Linux hosts.

As it contains no native code, it should work on any PyUSB_ and libusb_
supported platforms. However, Ms Windows is a seamless source of issues and is
not supported. Your mileage may vary.

Examples
~~~~~~~~

See pyftdi_/tests directory for GPIO examples.

See pyspiflash_ module for SPI examples.

.. include:: serialext/README.rst

.. _PyFtdi: https://www.github.com/eblot/
.. _FTDI: http://www.ftdichip.com/
.. _PyUSB: https://walac.github.io/pyusb/
.. _Python: https://www.python.org/
.. _pyserial: https://pythonhosted.org/pyserial/
.. _libftdi: https://www.intra2net.com/en/developer/libftdi/
.. _pyspiflash: https://github.com/eblot/pyspiflash/
.. _libusb: http://www.libusb.info/
.. _macos_guide: http://www.ftdichip.com/Support/Documents/AppNotes/AN_134_FTDI_Drivers_Installation_Guide_for_MAC_OSX.pdf
