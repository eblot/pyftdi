.. include:: defs.rst

URL Scheme
----------

There are two ways to open a connection to an `Ftdi` object.

The recommended way to open a connection is to specify connection details
using a URL. The URL scheme is defined as:

::

    protocol://[vendor[:product[:index|:serial]]]/interface

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
