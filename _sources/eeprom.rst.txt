.. include:: defs.rst

EEPROM management
-----------------

.. warning::
   Writing to the EEPROM can cause very **undesired** effects if the wrong
   value is written in the wrong place. You can even essentially **brick** your
   FTDI device. Use this function only with **extreme** caution.

   It is not recommended to use this application with devices that use an
   internal EEPROM such as FT232R or FT-X series, as if something goes wrong,
   recovery options are indeed limited. FT232R internal EEPROM seems to be
   unstable, even the official FT_PROG_ tool from FTDI may fail to fix it on
   some conditions.

   If using a Hi-Speed Mini Module and you brick for FTDI device, see
   FTDI_Recovery_


Supported features
~~~~~~~~~~~~~~~~~~

EEPROM support is under active development.

Some features may be wrongly decoded, as each FTDI model implements a different
feature map, and more test/validation are required.

The :doc:`EEPROM API <api/eeprom>` implements the upper API to access the
EEPROM content.

.. _ftconf:

EEPROM configuration tool
~~~~~~~~~~~~~~~~~~~~~~~~~

``ftconf.py`` is a companion script to help managing the content of the FTDI
EEPROM from the command line. See the :ref:`tools` chapter to locate this tool.

::

  ftconf.py [-h] [-x] [-X HEXBLOCK] [-i INPUT] [-l {all,raw,values}]
                   [-o OUTPUT] [-s SERIAL_NUMBER] [-m MANUFACTURER] [-p PRODUCT]
                   [-c CONFIG] [-e] [-u] [-P VIDPID] [-V VIRTUAL] [-v] [-d]
                   [device]

  Simple FTDI EEPROM configurator.

  positional arguments:
    device                serial port device name

  optional arguments:
    -h, --help            show this help message and exit
    -x, --hexdump         dump EEPROM content as ASCII
    -X HEXBLOCK, --hexblock HEXBLOCK
                          dump EEPROM as indented hexa blocks
    -i INPUT, --input INPUT
                          input ini file to load EEPROM content
    -l {all,raw,values}, --load {all,raw,values}
                          section(s) to load from input file
    -o OUTPUT, --output OUTPUT
                          output ini file to save EEPROM content
    -s SERIAL_NUMBER, --serial-number SERIAL_NUMBER
                          set serial number
    -m MANUFACTURER, --manufacturer MANUFACTURER
                          set manufacturer name
    -p PRODUCT, --product PRODUCT
                          set product name
    -c CONFIG, --config CONFIG
                          change/configure a property as key=value pair
    -e, --erase           erase the whole EEPROM content
    -u, --update          perform actual update, use w/ care
    -P VIDPID, --vidpid VIDPID
                          specify a custom VID:PID device ID, may be repeated
    -V VIRTUAL, --virtual VIRTUAL
                          use a virtual device, specified as YaML
    -v, --verbose         increase verbosity
    -d, --debug           enable debug mode


**Again, please read the** :doc:`license` **terms before using the EEPROM API
or this script. You may brick your device if something goes wrong, and there
may be no way to recover your device.**

Note that to protect the EEPROM content of unexpected modification, it is
mandatory to specify the :ref:`-u <option_u>` flag along any alteration/change
of the EEPROM content. Without this flag, the script performs a dry-run
execution of the changes, *i.e.* all actions but the write request to the
EEPROM are executed.

Once updated, you need to unplug/plug back the device to use the new EEPROM
configuration.

It is recommended to first save the current content of the EEPROM, using the
:ref:`-o <option_o>` flag, to have a working copy of the EEPROM data before any
attempt to modify it. It can help restoring the EEPROM if something gets wrong
during a subsequence update, thanks to the :ref:`-i <option_i>` option switch.

Most FTDI device can run without an EEPROM. If something goes wrong, try to
erase the EEPROM content, then restore the original content.


Option switches
```````````````
In addition to the :ref:`common_option_switches` for  PyFtdi_ tools,
``ftconf.py`` support the following arguments:

.. _option_c:

``-c name=value``
  Change a configuration in the EEPROM. This flag can be repeated as many times
  as required to change several configuration parameter at once. Note that
  without option ``-u``, the EEPROM content is not actually modified, the
  script runs in dry-run mode.

  The name should be separated from the value with an equal ``=`` sign or
  alternatively a full column ``:`` character.

  * To obtain the list of supported name, use the `?` wildcard: ``-c ?``.
  * To obtain the list of supported values for a namw, use the `?` wildcard:
    ``-c name=?``, where *name* is a supported name.

  See :ref:`cbus_func` table for the alternate function associated with each
  name.

.. _option_e:

``-e``
  Erase the whole EEPROM. This may be useful to recover from a corrupted
  EEPROM, as when no EEPROM or a blank EEPROM is detected, the FTDI falls back
  to a default configuration. Note that without option :ref:`-u <option_u>`,
  the EEPROM content is not actually modified, the script runs in dry-run mode.

.. _option_i:

``-i``
  Load a INI file (as generated with the :ref:`-o <option_o>` option switch. It
  is possible to select which section(s) from the INI file are loaded, using
  :ref:`-l <option_l>` option switch. The ``values`` section may be modified,
  as it takes precedence over the ``raw`` section. Note that without option
  :ref:`-u <option_u>`, the EEPROM content is not actually modified, the script
  runs in dry-run mode.

.. _option_l:

``-l <all|raw|values>``
  Define which section(s) of the INI file are used to update the EEPROM content
  along with the :ref:`-i <option_i>` option switch. Defaults to ``all``.

  The supported feature set of the ``values`` is the same as the one exposed
  through the :ref:`-c <option_c>` option switch. Unsupported feature are
  ignored, and a warning is emitted for each unsupported feature.

.. _option_m:

``-m <manufacturer>``
  Assign a new manufacturer name to the device. Note that without option
  :ref:`-u <option_u>`, the EEPROM content is not actually modified, the script
  runs in dry-run mode. Manufacturer names with ``/`` or ``:`` characters are
  rejected, to avoid parsing issues with FTDI :ref:`URLs <url_scheme>`.


.. _option_o:

``-o <output>``
  Generate and write to the specified file the EEPROM content as decoded
  values and a hexa dump. The special ``-`` file can be used as the output file
  to print to the standard output. The output file contains two sections:

  * ``[values]`` that contain the decoded EEPROM configuration as key, value
    pair. Note that the keys and values can be used as configuration input, see
    option :ref:`-c <option_c>`.
  * ``[raw]`` that contains a compact representation of the EEPROM raw content,
    encoded as hexadecimal strings.

.. _option_p:

``-p <product>``
  Assign a new product name to the device. Note that without option :ref:`-u
  <option_u>`, the EEPROM content is not actually modified, the script runs in
  dry-run mode. Product names with ``/`` or ``:`` characters are rejected, to
  avoid parsing issues with FTDI :ref:`URLs <url_scheme>`.

.. _option_s:

``-s <serial>``
  Assign a new serial number to the device. Note that without option :ref:`-u
  <option_u>`, the EEPROM content is not actually modified, the script runs in
  dry-run mode. Serial number with ``/`` or ``:`` characters are rejected, to
  avoid parsing issues with FTDI :ref:`URLs <url_scheme>`.

.. _option_u:

``-u``
  Update the EEPROM with the new settings. Without this flag, the script runs
  in dry-run mode, so no change is made to the EEPROM. Whenever this flag is
  used, the EEPROM is actually updated and its checksum regenerated. If
  something goes wrong at this point, you may brick you board, you've been
  warned. PyFtdi_ offers neither guarantee whatsoever than altering the EEPROM
  content is safe, nor that it is possible to recover from a bricked device.

.. _option_x:

``-x``
  Generate and print a hexadecimal raw dump of the EEPROM content, similar to
  the output of the `hexdump -Cv` tool.


.. _cbus_func:

CBUS function
`````````````

The following table describes the CBUS pin alternate functions. Note that
depending on the actual device, some alternate function may not be available.

+-----------------+--------+--------------------------------------------------------------------------------+
| Name            | Active | Description                                                                    |
+=================+========+================================================================================+
| ``TRISTATE``    | Hi-Z   | IO Pad is tri-stated                                                           |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``TXLED``       | Low    | TX activity, can be used as status for LED                                     |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``RXLED``       | Low    | RX activity, can be used as status for LED                                     |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``TXRXLED``     | Low    | TX & RX activity, can be used as status for LED                                |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``PWREN``       | Low    | USB configured, USB suspend: high                                              |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``SLEEP``       | Low    | USB suspend, typically used to power down external devices.                    |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``DRIVE0``      | Low    | Drive a constant (FT232H and FT-X only)                                        |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``DRIVE1``      | High   | Drive a constant (FT232H and FT-X only)                                        |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``GPIO``        |        | IO port for CBUS bit bang mode                                                 |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``TXDEN``       | High   | Enable transmit for RS485 mode                                                 |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``CLK48``       |        | Output 48 MHz clock (FT232R only)                                              |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``CLK30``       |        | Output 30 MHz clock (FT232H only)                                              |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``CLK24``       |        | Output 24 MHz clock (FT232R and FT-X only)                                     |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``CLK15``       |        | Output 12 MHz clock (FT232H only)                                              |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``CLK12``       |        | Output 12 MHz clock (FT232R and FT-X only)                                     |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``CLK7_5``      |        | Output 7.5 MHz clock (FT232H only)                                             |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``CLK6``        |        | Output 6 MHz clock (FT232R and FT-X only)                                      |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``BAT_DETECT``  | High   | Battery Charger Detect (FT-X only)                                             |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``BAT_NDETECT`` | Low    | Inverse signal of BAT_DETECT (FT-X only)                                       |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``I2C_TXE``     | Low    | Transmit buffer empty (FT-X only)                                              |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``I2C_RXF``     | Low    | Receive buffer full  (FT-X only)                                               |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``VBUS_SENSE``  | High   | Detect when VBUS is present via the appropriate AC IO pad (FT-X only)          |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``BB_WR``       | Low    | Synchronous Bit Bang Write strobe (FT232R and FT-X only)                       |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``BB_RD``       | Low    | Synchronous Bit Bang Read strobe (FT232R and FT-X only)                        |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``TIMESTAMP``   |        | Toggle signal each time a USB SOF is received (FT-X only)                      |
+-----------------+--------+--------------------------------------------------------------------------------+
| ``AWAKE``       | Low    | Do not suspend when unplugged/disconnect/suspsend (FT-X only)                  |
+-----------------+--------+--------------------------------------------------------------------------------+


Examples
````````

* Change product name and serial number

  ::

    pyftdi/bin/ftconf.py ftdi:///1 -p UartBridge -s abcd1234 -u

* List supported configuration parameters

  ::

    pyftdi/bin/ftconf.py ftdi:///1 -c ?
      cbus_func_0, cbus_func_1, cbus_func_2, cbus_func_3, cbus_func_4,
      cbus_func_5, cbus_func_6, cbus_func_7, cbus_func_8, cbus_func_9,
      channel_a_driver, channel_a_type, chip, clock_polarity,
      flow_control, group_0_drive, group_0_schmitt, group_0_slew,
      group_1_drive, group_1_schmitt, group_1_slew, has_serial,
      has_usb_version, in_isochronous, lsb_data, out_isochronous,
      power_max, powersave, product_id, remote_wakeup, self_powered,
      suspend_pull_down, type, usb_version, vendor_id

* List supported configuration values for CBUS0

  ::

    pyftdi/bin/ftconf.py ftdi:///1 -c cbus_func_0:?
      AWAKE, BAT_DETECT, BAT_NDETECT, BB_RD, BB_WR, CLK12, CLK24, CLK6,
      DRIVE0, DRIVE1, I2C_RXF, I2C_TXE, GPIO, PWREN, RXLED, SLEEP,
      TIME_STAMP, TRISTATE, TXDEN, TXLED, TXRXLED, VBUS_SENSE

.. _eeprom_cbus:

* Configure CBUS: 0 and 3 as GPIOs, then show the device configuration

  ::

   pyftdi/bin/ftconf.py ftdi:///1 -v
      -c cbus_func_0:GPIO -c cbus_func_3:GPIO
