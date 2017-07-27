.. include:: ../defs.rst

:mod:`spi` - SPI API
--------------------

.. module :: pyftdi.spi

Quickstart
~~~~~~~~~~

Example: communication with a SPI data flash

.. code-block:: python

    # Instanciate a SPI controller
    spi = SpiController()

    # Configure the first interface (IF/1) of the FTDI device as a SPI master
    spi.configure('ftdi://ftdi:2232h/1')

    # Get a port to a SPI slave w/ /CS on A*BUS3 and SPI mode 0 @ 12MHz
    slave = spi.get_port(cs=0, freq=12E6, mode=0)

    # Request the JEDEC ID from the SPI slave
    jedec_id = slave.exchange([0x9f], 3).tobytes()


See also pyspiflash_ module and `tests/spi.py`_


Classes
~~~~~~~

.. autoclass :: SpiController
 :members:

.. autoclass :: SpiPort
 :members:


Exceptions
~~~~~~~~~~

.. autoexception :: SpiIOError

Tests
~~~~~

SPI sample tests expect:
  * MX25L1606E device on /CS 0, SPI mode 0
  * ADXL345 device on /CS 1, SPI mode 3
  * RFDA2125 device on /CS 2, SPI mode 0

Checkout a fresh copy from PyFtdi_ github repository.

See :doc:`../pinout` for FTDI wiring.

.. code-block:: shell

   # optional: specify an alternative FTDI device
   export FTDI_DEVICE=ftdi://ftdi:2232h/1
   # optional: increase log level
   export FTDI_LOGLEVEL=DEBUG
   # be sure to connect the appropriate SPI slaves to the FTDI SPI bus and run
   PYTHONPATH=. python3 pyftdi/tests/spi.py
