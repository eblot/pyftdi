.. include:: defs.rst

Troubleshooting
---------------

Logging
~~~~~~~

FTDI uses the `pyftdi` logger.

It emits log messages with raw payload bytes at DEBUG level, and data loss
at ERROR level.

Common error messages
~~~~~~~~~~~~~~~~~~~~~

"Error: No backend available"
.............................

libusb native library cannot be loaded. Try helping the dynamic loader:

* On Linux: ``export LD_LIBRARY_PATH=<path>``

  where ``<path>`` is the directory containing the ``libusb-1.*.so``
  library file

* On macOS: ``export DYLD_LIBRARY_PATH=.../lib``

  where ``<path>`` is the directory containing the ``libusb-1.*.dylib``
  library file


"Error: Access denied (insufficient permissions)"
.................................................

The system may already be using the device.

* On macOS: starting with Mavericks (10.9+), OS X ships with a native FTDI
  driver that preempts access to the FTDI device.

  The driver can be unloaded this way:

  .. code-block:: shell

      sudo kextunload [-v] -bundle com.apple.driver.AppleUSBFTDI

  You may want to use an alias or a tiny script such as
  ``pyftdi/bin/uphy.sh``

  Please note that the system automatically reloads the driver, so it may be
  useful to move the kernel extension so that the system never loads it.

  FTDI provides a `handy helper for macOS 10.11`_ and later, to prevent automatic claiming of the device as a serial port.
  They have also included a `Installing D2xx drivers on macOS`_ video, on how to install the drivers and the helper.

* On Linux: it may indicate a missing or invalid udev configuration. See
  the :doc:`installation` section.

* This error message may also be triggered whenever the communication port is
  already in use.


"Error: The device has no langid"
.................................

* On Linux, it usually comes from the same installation issue as the
  ``Access denied`` error: the current user is not granted the permissions to
  access the FTDI device, therefore pyusb cannot read the FTDI registers. Check
  out the :doc:`installation` section.


"serial.serialutil.SerialException: Unable to open USB port"
............................................................

May be caused by a conflict with the FTDI virtual COM port (VCOM). Try
uninstalling the driver. On macOS, refer to this `FTDI macOS guide`_.


Slow initialisation on OS X El Capitan
......................................

It may take several seconds to open or enumerate FTDI devices.

If you run libusb <= v1.0.20, be sure to read the `Libusb issue on macOS`_
with OS X 10.11+.
