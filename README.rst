########
 PyFtdi
########

=============
Primary goals
=============

PyFtdi aims at providing a user-space driver for modern FTDI_ devices, 
implemented in pure Python language.

Modern FTDI_ devices include:

* FT2232H (dual port)
* FT4232H (quad port)

Other FTDI_ devices could also been supported (including FT232* devices), 
although these devices are not a primary goal for PyFtdi.

It should support the following modes:

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device 
  capability)
* SPI master
* JTAG master
* Bitbang/GPIO support (not a primary goal)

PyFtdi should provide a pyserial_ compliant API, so it can be used as a 
drop-in module to access USB-serial converters based on FTDI_ devices.

.. _FTDI: www.ftdichip.com
.. _pyserial: http://pyserial.sourceforge.net/


============
Requirements
============

PyFtdi relies on PyUSB_, which itself depends on one of the following native
libraries:

* libusb-1.0 (recommended)
* libusb-0.1 (deprecated)
* openusb (not tested with pyftdi)

PyFtdi does not depend on any other native library, and only uses standard 
Python modules.

Python_ 2.6 or above is required, although PyFtdi may run on older Python 
releases with some or no modifications.

.. _PyUSB: http://sourceforge.net/projects/pyusb/
.. _Python: http://python.org/
