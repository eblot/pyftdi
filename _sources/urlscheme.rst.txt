.. include:: defs.rst

URL Scheme
----------

There are two ways to open a connection to an `Ftdi` object.

The recommended way to open a connection is to specify connection details
using a URL. The URL scheme is defined as:

::

    ftdi://[vendor][:[product][:serial|:bus:address|:index]]/interface

where:

* vendor: the USB vendor ID of the manufacturer

  * ex: ``ftdi`` or ``0x403``

* product: the USB product ID of the device

  * ex: ``232h`` or ``0x6014``
  * Supported product IDs: ``0x6001``, ``0x6010``, ``0x6011``, ``0x6014``, ``0x6015``
  * Supported product aliases:

    * ``232``, ``232r``, ``232h``, ``2232d``, ``2232h``, ``4232h``, ``230x``
    * ``ft`` prefix for all aliases is also accepted, as for example ``ft232h``

* ``serial``: the serial number as a string. This is the preferred method to
  uniquely identify a specific FTDI device. However, some FTDI device are not
  fitted with an EEPROM, or the EEPROM is either corrupted or erased. In this
  case, FTDI devices report no serial number

  Examples:
     * ``ftdi://ftdi:232h:FT0FMF6V/1``
     * ``ftdi://:232h:FT0FMF6V/1``
     * ``ftdi://::FT0FMF6V/1``

* ``bus:addess``: it is possible to select a FTDI device through a bus:address
  pair, specified as *hexadecimal* integer values.

  Examples:
     * ``ftdi://ftdi:232h:10:22/1``
     * ``ftdi://ftdi:232h:10:22/1``
     * ``ftdi://::10:22/1``

  Here, bus ``(0x)10`` = 16 (decimal) and address ``(0x)22`` = 34 (decimal)

* ``index``: an integer - not particularly useful, as it depends on the
  enumeration order on the USB buses, and may vary from on session to another.

* ``interface``: the interface of FTDI device, starting from 1

  * ``1`` for 230x and 232\* devices,
  * ``1`` or ``2`` for 2232\* devices,
  * ``1``, ``2``, ``3`` or ``4`` for 4232\* devices

All parameters but the interface are optional, PyFtdi tries to find the best
match. Therefore, if you have a single FTDI device connected to your system,
``ftdi:///1`` should be enough.

You can also ask PyFtdi to enumerate all the compatible devices with the
special ``ftdi:///?`` syntax. This syntax is useful to retrieve the available
FTDI URLs with serial number and/or bus:address selectors.

Note that if there's only one FTDI device connected to the host, the FTDI URL
can be as simple as ``ftdi:///n``, where n is the FTDI port to use, starting
from 1.

URL-based methods to open a connection
......................................

.. code-block:: python

   open_from_url()
   open_mpsse_from_url()
   open_bitbang_from_url()


Device-based methods to open a connection
.........................................

You may also open an Ftdi device from an existing PyUSB_ device, with the help
of the ``open_from_device()`` helper method.

.. code-block:: python

   open_from_device()
   open_mpsse_from_device()
   open_bitbang_from_device()


Legacy methods to open a connection
...................................

The old, deprecated method to open a connection is to use the ``open()``
methods without the ``_from_url`` suffix, which accept VID, PID, and serial
parameters (among others).

.. code-block:: python

   open()
   open_mpsse()
   open_bitbang()


Tools
~~~~~

The ``bin/`` directory contains a tiny script ``ftdu_urls.py`` to list the
available FTDIs connected to the host, and the URLs than can be used to open a
``Fdti()`` instance with the ``Ftdi.open_from_url()`` method.


Supporting custom USB vendor and product IDs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

PyFtdi only recognizes FTDI official vendor and product IDs.

If you have an FTDI device with an EEPROM with customized IDs, you need to tell
PyFtdi to support those custom USB identifiers.

Custom PID
..........

To support a custom product ID (16-bit integer) with the official FTDI ID, add
the following code **before** any call to an FTDI ``open()`` method.

.. code-block:: python

   from pyftdi.ftdi import Ftdi

   Ftdi.add_custom_product(Ftdi.DEFAULT_VENDOR, product_id)

Custom VID
..........

To support a custom vendor ID and product ID (16-bit integers), add the
following code **before** any call to an FTDI ``open()`` method.

.. code-block:: python

   from pyftdi.ftdi import Ftdi

   Ftdi.add_custom_vendor(vendor_id)
   Ftdi.add_custom_product(vendor_id, product_id)

You may also specify an arbitrary string to each method if you want to specify
a URL by custom vendor and product names instead of their numerical values:

.. code-block:: python

   from pyftdi.ftdi import Ftdi

   Ftdi.add_custom_vendor(0x1234, 'myvendor')
   Ftdi.add_custom_product(0x1234, 0x5678, 'myproduct')

   f1 = Ftdi.create_from_url('ftdi://0x1234:0x5678/1')
   f2 = Ftdi.create_from_url('ftdi://myvendor:myproduct/2')
