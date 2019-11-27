.. include:: defs.rst

Requirements
------------

Python_ 3.5 or above is required. (see next section for Python 2.x support)

PyFtdi_ relies on PyUSB_, which itself depends on one of the following native
libraries:

* libusb_, currently tested with 1.0.23

PyFtdi_ does not depend on any other native library, and only uses standard
Python modules along with PyUSB_ and pyserial_.

PyFtdi_ has been tested with PyUSB_ 1.0.0.

 * PyUSB_ 1.0.0b1 or below is no longer supported.

.. note:: Note about previous releases

   If you have no choice but using previous releases of software, such as

   * Python_ (2.6+, 3.3 or 3.4),
   * other PyUSB_ backends such as the deprecated libusb-0.1, or openusb,
   * PyUSB_ 1.0.0b1 or below,
   * pyserial_ 2.6+ (previous versions of pyserial_ will NOT work)

   please checkout the latest PyFtdi_ 0.1x series (0.13.3) which provides support
   for these deprecated environmement, but is no longer actively maintained.

Development
~~~~~~~~~~~

PyFtdi_ is developed on macOS platforms (64-bit kernel), and is validated on a
regular basis on Linux hosts.

As it contains no native code, it should work on any PyUSB_ and libusb_
supported platforms. However, M$ Windows is a seamless source of issues and is
not officially supported, although users have reported successful installation
with Windows 7 for example. Your mileage may vary.
