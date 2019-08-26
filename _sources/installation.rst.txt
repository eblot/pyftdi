.. include:: defs.rst

Installation
------------

Prerequisites
~~~~~~~~~~~~~

PyFTDI_ relies on PyUSB_, which requires a native dependency: libusb 1.x.

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
created so that `udev` loads the rules for the matching device, or
alternatively, inform the ``udev`` daemon about the changes:

.. code-block:: shell

   sudo udevadm control --reload-rules
   sudo udevadm trigger

With this setup, be sure to add users that want to run PyFtdi_ to the
`plugdev` group, *e.g.*

.. code-block:: shell

    sudo adduser $USER plugdev

Remember that you need to log out / log in to get the above command
effective, or start a subshell to try testing PyFtdi_:

.. code-block:: shell

    newgrp plugdev


Homebrew macOS
..............

.. code-block:: shell

    brew install libusb

Windows
.......

Windows is not officially supported (*i.e.* not tested) but some users have
reported successful installations. Windows requires a specific libusb backend
installation.

Zadig
'''''

The probably easiest way to deal with libusb on Windows is to use Zadig_

1. Start up the Zadig utility

2. Select ``Options/List All Devices``, then select the FTDI devices you want
   to communicate with. Its names depends on your hardware, *i.e.* the name
   stored in the FTDI EEPROM.

  * With FTDI devices with multiple channels, such as FT2232 (2 channels) and
    FT4232 (4 channels), you **must** install the driver for the composite
    parent, **not** for the individual interfaces. If you install the driver
    for each interface, each interface will be presented as a unique FTDI
    device and you may have difficulties to select a specific FTDI device port
    once the installation is completed. To make the composite parents to appear
    in the device list, uncheck the ``Options/Ignore Hubs or Composite Parents``
    menu item.

  * Be sure to select the parent device, *i.e.* the device name should not end
    with *(Interface N)*, where *N* is the channel number.

    * for example *Dual RS232-HS* represents the composite parent, while
      *Dual RS232-HS (Interface 0)* represents a single channel of the FTDI
      device. Always select the former.

3. Select ``libusb-win32`` (not ``WinUSB``) in the driver list.

4. Click on ``Replace Driver``

See also `Libusb on Windows`_


Post-installation sanity check
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open a *shell*, or a *CMD* on Windows

.. code-block:: shell

    python3  # or 'python' on Windows
    from pyftdi.ftdi import Ftdi
    Ftdi().open_from_url('ftdi:///?')

should list all the FTDI devices available on your host.

  * Example with 1 FT232H device with a serial number and 1 FT2232 device
    with no serial number, connected to the host:

    .. code-block::

        Available interfaces:
          ftdi://ftdi:232h:FT1PWZ0Q/1   (C232HD-DDHSP-0)
          ftdi://ftdi:2232/1            (Dual RS232-HS)
          ftdi://ftdi:2232/2            (Dual RS232-HS)


Note that FTDI devices with custom VID/PID are not listed with this simple
command, please refer to the PyFtdi_ API to add custom identifiers, *i.e.* see
:py:meth:`pyftdi.ftdi.Ftdi.add_custom_vendor` and
:py:meth:`pyftdi.ftdi.Ftdi.add_custom_product` APIs.


Python dependencies
~~~~~~~~~~~~~~~~~~~

Dependencies should be automatically installed with PIP.

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
