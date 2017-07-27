.. include:: ../defs.rst

:mod:`serialext` - UART API
---------------------------

There is no dedicated module for the UART API, as PyFtdi_ acts as a backend of
the well-known pyserial_ module.

The pyserial_ backend module is implemented as the `serialext.protocol_ftdi`
module. It is not documented here as no direct call to this module is required,
as the UART client should use the regular pyserial_ API.

Usage
~~~~~

To enable PyFtdi_ as a pyserial_ backend, use the following import:

.. code-block:: python

    import pyftdi.serialext

Then use

.. code-block:: python

    pyftdi.serialext.serial_for_url(url, **options)

to open a pyserial_ serial port instance.


Quickstart
~~~~~~~~~~

.. code-block:: python

    # Enable pyserial extensions
    import pyftdi.serialext

    # Open a serial port on the second FTDI device interface (IF/2) @ 3Mbaud
    port = pyftdi.serialext.serial_for_url('ftdi://ftdi:2232h/2', baudrate=3000000)

    # Send bytes
    port.write(b'Hello World')

    # Receive bytes
    data = port.read(1024)


Mini serial terminal example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
                          'ftdi:///?)
    -b BAUDRATE, --baudrate=BAUDRATE
                          serial port baudrate
    -r RESET, --reset=RESET
                          HW reset on DTR line
    -o LOGFILE, --logfile=LOGFILE
                          path to the log file

If the PyFtdi module is not yet installed and ``pyterm.py`` is run from the
archive directory, ``PYTHONPATH`` should be defined to the current directory::

    PYTHONPATH=$PWD pyftdi/serialext/tests/pyterm.py -p ftdi:///?

The above command lists all the available FTDI device ports.

To start up a serial terminal session, use the ``-p`` option switch to select
the proper port, for example:

.. code-block:: shell

    # detect all FTDI connected devices
    PYTHONPATH=. python3 pyftdi/serialext/tests/pyterm.py -p ftdi:///?
    # use the first interface of the first FT2232H as a serial port
    PYTHONPATH=$PWD pyftdi/serialext/tests/pyterm.py -p ftdi://ftdi:2232/1
