.. include:: defs.rst

Troubleshooting
---------------

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
  ``pyftdi/tools/uphy.sh``

  Please note that the system automatically reloads the driver, so it may be
  useful to move the kernel extension so that the system never loads it.

* On Linux: it may indicate a missing or invalid udev configuration. See
  the :doc:`installation` section.

* This error message may also be triggered whenever the communication port is
  already in use.


"serial.serialutil.SerialException: Unable to open USB port"
............................................................

May be caused by a conflict with the FTDI virtual COM port (VCOM). Try
uninstalling the driver. On macOS, refer to this `FTDI macOS guide`_.

Slow initialisation on OS X El Capitan
......................................

It may take several seconds to open or enumerate FTDI devices.

If you run libusb <= v1.0.20, be sure to read the `Libusb issue on macOS`_
with OS X 10.11+.

