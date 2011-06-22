========
 PyFtdi
========

Overview
~~~~~~~~

PyFtdi aims at providing a user-space driver for modern FTDI_ devices,
implemented in pure Python language.

Modern FTDI_ devices include:

* FT2232D (dual port, clock up to 6 MHz)
* FT2232H (dual port, clock up to 30 MHz)
* FT4232H (quad port, clock up to 30 MHz)

Other FTDI_ devices could also be supported (including FT232* devices),
although these devices are not a primary goal for PyFtdi, and therefore have
not been tested with PyFtdi.

Extras
------
This module also contains a basic driver for Prolific PL2303 chip written in
pure Python. PL2303 is not an FTDI device, but it may serve the same purpose:
a USB-to-serial adapter.

As such, a Python driver for this device has been added to this project sarting
at version 0.4.0, so that a PL2303 serial adaptor can be used as an FTDI
alternative to drive a serial port from a USB bus.

Primary goals
~~~~~~~~~~~~~

It should support the following modes:

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device
  capability)
* SPI master
* JTAG master
* Bitbang/GPIO support (not a primary goal)

PyFtdi should provide a pyserial_ compliant API, to be used as a drop-in module
to access USB-serial converters based on FTDI_ devices.

.. _FTDI: http://www.ftdichip.com/
.. _pyserial: http://pyserial.sourceforge.net/


Requirements
~~~~~~~~~~~~

PyFtdi relies on PyUSB_, which itself depends on one of the following native
libraries:

* libusb-1.0 (recommended)
* libusb-0.1 (deprecated)
* openusb (not tested with pyftdi)

PyFtdi does not depend on any other native library, and only uses standard
Python modules.

To use the serial port feature of PyFtdi, pyserial_ 2.5+ module should be
installed.

Python_ 2.6 or above is required. Python_ 3.x is not yet supported.

.. _PyUSB: http://sourceforge.net/projects/pyusb/
.. _Python: http://python.org/


Status
~~~~~~

This project is still at an early alpha development stage.

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
  compliant API. The ``serialext`` directory contains a minimal serial terminal
  demonstrating the use of this extension, and a dispatcher automatically 
  selecting the serial backend (pyserial_, PyFtdi), based on the serial port
  name.
* SPI master. PyFtdi includes several examples demonstrating how to use the 
  FTDI SPI master with a pure-Python serial flash device driver for several 
  common devices. For now, SPI Mode 0 (CPOL=0, CPHA=0) is the only supported
  mode. It should be easy to extend the SPI master to deal with less common 
  modes. These tests show an average 470 KB/s read out from flash devices 
  running with a 6 MHz SPI clock on a Core2Duo Mac Book Pro.
* JTAG is under development and is not fully supported yet.

.. _libftdi: http://www.intra2net.com/en/developer/libftdi/

Development
~~~~~~~~~~~

PyFtdi is developed on Mac OS X platforms (including 64-bit kernels), and is
validated on a regular basis on Linux hosts.

As it contains no native code, it should work on any PyUSB_ and libusb_ 
supported platforms, including but not limited to, Windows.

.. _libusb: http://www.libusb.org/

Examples
--------

Serial port
...........

``serialext/tests/pyterm.py`` is a simple serial terminal that can be used
to test the serial port feature.::

  Usage: pyterm.py [options]
  Pure python simple serial terminal

  Options:
    -h, --help            show this help message and exit
    -d, --debug           enable debug mode
    -f, --fullmode        use full terminal mode, exit with [Ctrl]+A
    -p DEVICE, --port=DEVICE
                          serial port device name (list available ports with
                          'ftdi:///?' or 'prolific:///?')
    -b BAUDRATE, --baudrate=BAUDRATE
                          serial port baudrate
    -r RESET, --reset=RESET
                          HW reset on DTR line
    -o LOGFILE, --logfile=LOGFILE
                          path to the log file

If the PyFtdi module is not yet installed and ``pyterm.py`` is run from the
archive directory, ``PYTHONPATH`` should be defined to the current directory::

    PYTHONPATH=$PWD ./serialext/tests/pyterm.py -p ftdi:///?

The above command lists all the available FTDI device ports.

To start up a serial terminal session, use the ``-p`` option switch to select
the proper port, for example::

    PYTHONPATH=$PWD ./serialext/tests/pyterm.py -p ftdi://ftdi:2232/1
