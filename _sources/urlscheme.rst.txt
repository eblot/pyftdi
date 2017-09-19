.. include:: defs.rst

URL Scheme
----------

There are two ways to open a connection to an `Ftdi` object.

The recommended way to open a connection is to specify connection details
using a URL. The URL scheme is defined as:

::

    protocol://[vendor][:[product][:index|:serial]]/interface

where:

* protocol: always ``ftdi``
* vendor: the USB vendor ID of the manufacturer

  * ex: ``ftdi`` or ``0x403``

* product: the USB product ID of the device

  * ex: ``232h`` or ``0x6014``
  * Supported product IDs: ``0x6001``, ``0x6010``, ``0x6011``, ``0x6014``, ``0x6015``
  * Supported product aliases:

    * ``232``, ``232r``, ``232h``, ``2232d``, ``2232h``, ``4232h``, ``230x``
    * ``ft`` prefix for all aliases is also accepted, as for example ``ft232h``

* serial: the serial number as a string
* index: an integer (not particularly useful, as it depends on the enumeration
  order on the USB buses)
* interface: the interface of FTDI device, starting from 1

  * ex: ``1`` for 232\*, ``1`` or ``2`` for 2232\*, ``1``-``4`` for 4232\* devices

All parameters but the interface are optional, PyFtdi tries to find the best
match. Therefore, if you have a single FTDI device connected to your system,
``ftdi:///1`` should be enough.

You can also ask PyFtdi to enumerate all the compatible devices with the
special ``ftdi:///?`` syntax.

Note that if there's only one FTDI device connected to the host, the FTDI URL
can be as simple as ``ftdi:///n``, where n is the FTDI port to use, starting
from 1.

You may also select a FTDI device by its sole serial number, *e.g.*
`ftdi://::serial/1`.

URL-based methods to open a connection:

.. code-block:: python

   open_from_url()
   open_mpsse_from_url()
   open_bitbang_from_url()


The old, deprecated method to open a connection is to use the ``open()``
methods without the ``_from_url`` suffix, which accept VID, PID, and serial
parameters (among others).

.. code-block:: python

   open()
   open_mpsse()
   open_bitbang()


.. warning:: API break
   ``open()``, ``open_mpsse()`` and ``open_bitbang`` arguments have changed in
   v0.22.0, be sure to update your code.


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
