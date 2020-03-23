.. include:: defs.rst

.. _tools:

Tools
-----

Overview
~~~~~~~~

PyFtdi_ comes with a couple of scripts designed to help using PyFtdi_ APIs,
and can be useful to quick start working with PyFtdi_.

Scripts
~~~~~~~

.. _ftdi_urls:

``ftdi_urls``
`````````````

This tiny script ``ftdu_urls.py`` to list the available, *i.e.* detected,
FTDI devices connected to the host, and the URLs than can be used to open a
:py:class:`pyftdi.ftdi.Ftdi` instance with the
:py:class:`pyftdi.ftdi.Ftdi.open_from_url` family and ``configure`` methods.


``ftconf``
``````````

``ftconf.py`` is a companion script to help managing the content of
the FTDI EEPROM from the command line. See the :ref:`ftconf` documentation.


.. _i2cscan:

``i2cscan``
```````````

The ``i2cscan.py`` script helps to discover which I2C devices
are connected to the FTDI I2C bus.


.. _pyterm.py:

``pyterm``
``````````

``pyterm.py`` is a simple serial terminal that can be used to test the serial
port feature, see the :ref:`pyterm` documentation.


Where to find these tools?
~~~~~~~~~~~~~~~~~~~~~~~~~~

These scripts can be downloaded from PyFtdiTools_, and are also installed along
with the PyFtdi_ module on the local host.

The location of the scripts depends on how PyFtdi_ has been installed and the
type of hosts:

* on linux and macOS, there are located in the ``bin/`` directory, that is the
  directory where the Python interpreter is installed.

* on Windows, there are located in the ``Scripts/`` directory, which is a
  subdirectory of the directory where the Python interpreter is installed.

