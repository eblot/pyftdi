.. include:: defs.rst

Installation
------------

Native dependencies
~~~~~~~~~~~~~~~~~~~

Install native dependency: libusb 1.x.

The actual command to install depends on your OS and/or your distribution,
see below

Debian/Ubuntu Linux
...................

.. code-block:: shell

     apt-get install libusb-1.0

On Linux, you also need to create a `udev` configuration file to allow
user-space processes to access to the FTDI devices. There are many ways to
configure `udev`, here is a typical setup:

::

    # /etc/udev/rules.d/11-ftdi.rules
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6011", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6015", GROUP="plugdev", MODE="0666"

You need to unplug / plug back the FTDI device once this file has been
created so that `udev` loads the rules for the matching device.

With this setup, be sure to add users that want to run PyFtdi_ to the
`plugdev` group, *e.g.*

  sudo adduser $USER plugdev

Remember that you need to log out / log in to get the above command
effective.

Homebrew macOS
..............

.. code-block:: shell

    brew install libusb

Windows
.......

Windows is not officially supported (*i.e.* not tested) but some users have
reported successful installations. Windows requires a specific libusb backend
installation.

Here is a brief setup guide from Andrea Concil for use with the libusb-0.1
backend:

Libusb-devel-filter
...................

 * install Libusb-win32-devel-filter from `Libusb win32`_
 * before using it, install the so called "filter" so that libusb can coexist
   with specific peripherals usb drivers:

   * go into the Libusb-Win32 menu (it should be something like
     ``C:\ProgramData\Microsoft\Windows\Start Menu\Programs\LibUSB-Win32``) and
     into the folder Class Filter launch the installation of the filter for all
     the usb devices `i.e.` Install all class filters. Once done libusb can be
     used "in parallel" with original drivers.

Zadig
.....

Another libusb backend implementation can be installed with Zadig_

See also `Libusb on Windows`_


Python dependencies
~~~~~~~~~~~~~~~~~~~

  * pyusb >= 1.0.0
  * pyserial >= 3.0


Installing with PIP
~~~~~~~~~~~~~~~~~~~

PIP should automatically install the missing dependencies.

.. code-block:: shell

     pip3 install pyftdi


Installing from source
~~~~~~~~~~~~~~~~~~~~~~

If you prefer to install from source, check out a fresh copy from PyFtdi_
github repository.

.. code-block:: shell

     pip3 install pyusb
     pip3 install pyserial
     git clone https://github.com/eblot/pyftdi.git
     cd pyftdi
     python3 setup.py ...
