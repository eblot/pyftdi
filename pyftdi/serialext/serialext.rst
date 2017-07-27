Serial port
-----------

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
the proper port, for example::

    PYTHONPATH=$PWD pyftdi/serialext/tests/pyterm.py -p ftdi://ftdi:2232/1


Quick step-by-step instruction guide
....................................

Shell commands::

  pyvenv ~/pyusb
  ~/pyusb/bin/pip3 install pyserial
  ~/pyusb/bin/pip3 install pyusb
  PYTHONPATH=. ~/pyusb/bin/python3 pyftdi/serialext/tests/pyterm.py -p ftdi:///?

Note that if there's only one FTDI device connected to the host, the FTDI URL
should be as simple as ``ftdi:///n``, where n is the FTDI UART port (starting
from 1).
