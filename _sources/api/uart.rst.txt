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

  Usage: pyterm.py [-h] [-f] [-p DEVICE] [-b BAUDRATE] [-w] [-e] [-r] [-l] [-s]
                 [-v] [-d]

  Simple Python serial terminal

  Optional arguments:
    -h, --help            show this help message and exit
    -f, --fullmode        use full terminal mode, exit with [Ctrl]+B
    -p DEVICE, --device DEVICE
                          serial port device name (default: ftdi:///1)
    -b BAUDRATE, --baudrate BAUDRATE
                          serial port baudrate (default: 115200)
    -w, --hwflow          hardware flow control
    -e, --localecho       local echo mode (print all typed chars)
    -r, --crlf            prefix LF with CR char, use twice to replace all LF
                          with CR chars
    -l, --loopback        loopback mode (send back all received chars)
    -s, --silent          silent mode
    -v, --verbose         increase verbosity
    -d, --debug           enable debug mode

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


.. _uart-limitations:

Limitations
~~~~~~~~~~~

Although the FTDI H series are in theory capable of 12 MBps baudrate, baudrates
above 6 Mbps are barely usable.

See the following table for details.

+------------+-------------+------------+-------------+------------+--------+
|  Requ. bps |HW capability| 9-bit time |  Real bps   | Duty cycle | Stable |
+============+=============+============+=============+============+========+
| 115.2 Kbps |  115.2 Kbps |   78.08 µs | 115.26 Kbps |     49.9%  |  Yes   |
+------------+-------------+------------+-------------+------------+--------+
| 460.8 Kbps | 461.54 Kbps |   19.49 µs | 461.77 Kbps |     49.9%  |  Yes   |
+------------+-------------+------------+-------------+------------+--------+
|     1 Mbps |      1 Mbps |   8.98 µs  |  1.002 Mbps |     49.5%  |  Yes   |
+------------+-------------+------------+-------------+------------+--------+
|     4 Mbps |      4 Mbps |   2.24 µs  |  4.018 Mbps |       48%  |  Yes   |
+------------+-------------+------------+-------------+------------+--------+
|     5 Mbps |  5.052 Mbps |   1.78 µs  |  5.056 Mbps |       50%  |  Yes   |
+------------+-------------+------------+-------------+------------+--------+
|     6 Mbps |      6 Mbps |   1.49 µs  |  6.040 Mbps |     48.5%  |  Yes   |
+------------+-------------+------------+-------------+------------+--------+
|     7 Mbps |  6.857 Mbps |   1.11 µs  |  8.108 Mbps |       44%  |   No   |
+------------+-------------+------------+-------------+------------+--------+
|     8 Mbps |      8 Mbps |   1.11 µs  |  8.108 Mbps |   44%-48%  |   No   |
+------------+-------------+------------+-------------+------------+--------+
|   8.8 Mbps |  8.727 Mbps |   1.13 µs  |  7.964 Mbps |       44%  |   No   |
+------------+-------------+------------+-------------+------------+--------+
|   9.6 Mbps |    9.6 Mbps |   1.12 µs  |  8.036 Mbps |       48%  |   No   |
+------------+-------------+------------+-------------+------------+--------+
|  10.5 Mbps | 10.667 Mbps |   1.11 µs  |  8.108 Mbps |       44%  |   No   |
+------------+-------------+------------+-------------+------------+--------+
|    12 Mbps |     12 Mbps |   0.75 µs  |     12 Mbps |       43%  |  Yes   |
+------------+-------------+------------+-------------+------------+--------+

 * 9-bit time is the measured time @ FTDI output pins for a 8-bit character
   (start bit + 8 bit data)
 * Duty cycle is the ratio between a low-bit duration and a high-bit duration,
   a good UART should exhibit the same duration for low bits and high bits,
   *i.e.* a duty cycle close to 50%.
 * Stability reports whether subsequent runs, with the very same HW settings,
   produce the same timings.

Moreover, as the hardware flow control of the FTDI device is not a true HW
flow control. Quoting FTDI application note:

   *If CTS# is logic 1 it is indicating the external device cannot accept more
   data. the FTxxx will stop transmitting within 0~3 characters, depending on
   what is in the buffer.*
   **This potential 3 character overrun does occasionally present problems.**
   *Customers shoud be made aware the FTxxx is a USB device and not a "normal"
   RS232 device as seen on a PC. As such the device operates on a packet
   basis as opposed to a byte basis.*

Achieving a reliable connection over 6 Mbps has proven difficult, if not
impossible: Any baudrate greater than 6 Mbps (except the upper 12 Mbps limit)
results into an actual baudrate of about 8 Mbps, and suffer from clock
fluterring [7.95 .. 8.1Mbps].
