.. include:: ../defs.rst

:mod:`spi` - SPI API
--------------------

.. module :: pyftdi.spi

Quickstart
~~~~~~~~~~

Example: communication with a SPI data flash (half-duplex example)

.. code-block:: python

    # Instanciate a SPI controller
    spi = SpiController()

    # Configure the first interface (IF/1) of the FTDI device as a SPI master
    spi.configure('ftdi://ftdi:2232h/1')

    # Get a port to a SPI slave w/ /CS on A*BUS3 and SPI mode 0 @ 12MHz
    slave = spi.get_port(cs=0, freq=12E6, mode=0)

    # Request the JEDEC ID from the SPI slave
    jedec_id = slave.exchange([0x9f], 3).tobytes()


Example: communication with a remote SPI device using full-duplex mode

.. code-block:: python

    # Instanciate a SPI controller
    # We need want to use A*BUS4 for /CS, so at least 2 /CS lines should be
    # reserved for SPI, the remaining IO are available as GPIOs.
    spi = SpiController(cs_count=2)

    # Configure the first interface (IF/1) of the FTDI device as a SPI master
    spi.configure('ftdi://ftdi:2232h/1')

    # Get a port to a SPI slave w/ /CS on A*BUS4 and SPI mode 3 @ 10MHz
    slave = spi.get_port(cs=1, freq=10E6, mode=3)

    # Synchronous exchange with the remote SPI slave
    write_buf = b'\x01\x02\x03'
    read_buf = slave.exchange(write_buf, duplex=True).tobytes()

See also pyspiflash_ module and `tests/spi.py`_

Example: communication with a SPI device and an extra GPIO

.. code-block:: python

    # Instanciate a SPI controller
    spi = SpiController()

    # Configure the first interface (IF/1) of the first FTDI device as a
    # SPI master
    spi.configure('ftdi://::/1')

    # Get a SPI port to a SPI slave w/ /CS on A*BUS3 and SPI mode 0 @ 12MHz
    slave = spi.get_port(cs=0, freq=12E6, mode=0)

    # Get GPIO port to manage extra pins, use A*BUS4 as GPO, A*BUS4 as GPI
    gpio = spi.get_gpio()
    gpio.set_direction(0x30, 0x10)

    # Assert GPO pin
    gpio.write(0x10)
    # Write to SPI slace
    slave.write(b'hello world!')
    # Release GPO pin
    gpio.write(0x00)
    # Test GPI pin
    pin = bool(gpio.read() & 0x20)


Classes
~~~~~~~

.. autoclass :: SpiController
 :members:

.. autoclass :: SpiPort
 :members:

.. autoclass :: SpiGpioPort
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

Caveats
~~~~~~~

* Due to the MPSSE engine limitation, it is not possible to achieve
  time-controlled request sequence. In other words, if the SPI slave needs to
  receive command sequences at precise instants - for example ADC or DAC
  devices - PyFtdi_ use is not recommended. This limitation is likely to apply
  to any library that relies on FTDI device. The USB bus latency and the lack
  of timestamped commands always add jitter and delays, with no easy known
  workaround.

* FTDI devices are documented to only support SPI mode 0 and mode 2. Mode 1
  may work in some cases, but mode 3 is known to fail.
