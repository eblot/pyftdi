.. include:: defs.rst

Requirements
------------

Python_ 3.7 or above is required.

* PyFtdi *v0.53* is the last PyFtdi version to support Python 3.6.

  * Python 3.6 has reached end-of-life on December 23rd, 2021.

* PyFtdi *v0.52* is the last PyFtdi version to support Python 3.5.

  * Python 3.5 has reached end-of-life on September 5th, 2020.

PyFtdi_ relies on PyUSB_, which itself depends on one of the following native
libraries:

* libusb_, currently tested with 1.0.23

PyFtdi_ does not depend on any other native library, and only uses standard
Python modules along with PyUSB_ and pyserial_.

PyFtdi_ is beeing tested with PyUSB_ 1.1.0.

Development
~~~~~~~~~~~

PyFtdi_ is developed on macOS platforms (64-bit kernel), and is validated on a
regular basis on Linux hosts.

As it contains no native code, it should work on any PyUSB_ and libusb_
supported platforms. However, M$ Windows is a seamless source of issues and is
not officially supported, although users have reported successful installation
with Windows 7 for example. Your mileage may vary.


API breaks
~~~~~~~~~~

Starting with version *v0.40.0*, several API changes are being introduced.
While PyFtdi tries to maintain backward compatibility with previous versions,
some of these changes may require existing clients to update calls to PyFtdi.

Do not upgrade to *v0.40.0* or above without testing your client against the
new PyFtdi releases. PyFtdi versions up to *v0.39.9* keep a stable API
with *v0.22+* series.

See the *Major Changes* section for details about potential API breaks.
