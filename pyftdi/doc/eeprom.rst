.. include:: defs.rst

EEPROM management
-----------------

.. warning::
   Writing to the EEPROM can cause very **undesired** effects if the wrong
   value is written in the wrong place. You can even essentially **brick** your
   FTDI device. Use this function only with **extreme** caution.

   If using a Hi-Speed Mini Module and you brick for FTDI device, see
   FTDI_Recovery_


Supported features
~~~~~~~~~~~~~~~~~~

EEPROM support is experimental. For now, PyFtdi_ mostly supports read-only
feature to report EEPROM configuration.

Some features may be wrongly decoded, as each FTDI model implements a different
feature map, and more test/validation is required.

Write access is limited to modifying the variable length strings of the EEPROM,
that is:

 * manufacturer name, *e.g.* `FTDI`
 * product description string, *e.g.* ``FT2232H MiniModule``
 * serial number, *e.g.* ``FT123ABC``

Changing the product description may help to distinguish a specific FTDI device
when several of them are connected to a host, while changing the serial number
may help selecting a specific FTDI device from the command line, as the serial
number may be specified in the :doc:`URL <urlscheme>`.

The :doc:`EEPROM API <api/eeprom>` implements the upper API to access the
EEPROM content.


``ftconf.py``
~~~~~~~~~~~~~

``pyftdi/bin/ftconf.py`` is a companion script to help managing the content of
the FTDI EEPROM from the command line.

  $ pyftdi/bin/ftconf.py

  usage: ftconf.py [-h] [-x] [-o OUTPUT] [-s SERIAL_NUMBER] [-m MANUFACTURER]
                   [-p PRODUCT] [-e] [-u] [-v] [-d]
                   [device]

  Simple FTDI EEPROM configurator.

  positional arguments:
    device                serial port device name

  optional arguments:
    -h, --help            show this help message and exit
    -x, --hexdump         dump EEPROM content as ASCII
    -o OUTPUT, --output OUTPUT
                          output ini file to save EEPROM content
    -s SERIAL_NUMBER, --serial-number SERIAL_NUMBER
                          set serial number
    -m MANUFACTURER, --manufacturer MANUFACTURER
                          set manufacturer name
    -p PRODUCT, --product PRODUCT
                          set product name
    -e, --erase           erase the whole EEPROM content
    -u, --update          perform actual update, use w/ care
    -v, --verbose         increase verbosity
    -d, --debug           enable debug mode

**Again, please read the** :doc:`licenses` **terms before using the EEPROM API
or this script. You may brick your device if something goes wrong, and there
may be no way to recover your device.**

Note that to protect the EEPROM content of unexpected modification, it is
mandatory to specify the ``-u`` flag along any alteration/change of the EEPROM
content. Without this flag, the script performs a dry-run execution of the
changes, *i.e.* all actions but the write request to the EEPROM are executed.

Once updated, you need to unplug/plug back the device to use the new EEPROM
configuration.

It is recommended to first save the current content of the EEPROM, using the
``-o`` flag, to have a working copy of the EEPROM data before any attempt to
modify it. It can help restoring the EEPROM if something gets wrong during a
subsequence update.

Most FTDI device can run without an EEPROM. If something goes wrong, try to
erase the EEPROM content, then restore the original content. For now,
``ftconf.py`` does not support EEPROM restoration, but a Windows-only
application FT_PROG_ is available from FTDI web site.
