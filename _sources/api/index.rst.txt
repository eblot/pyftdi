PyFtdi API documentation
========================

.. include:: ../defs.rst

|release|
---------

.. toctree::
   :maxdepth: 1
   :glob:

   ftdi
   gpio
   i2c
   spi
   usbtools
   misc

Quickstart
----------

UART
~~~~

.. code-block:: python

    # Enable pyserial extensions
    import pyftdi.serialext

    # Open a serial port on the second FTDI device interface (IF/2) @ 3Mbaud
    port = pyftdi.serialext.serial_for_url('ftdi://ftdi:2232h/2', baudrate=3000000)

    # Send bytes
    port.write(b'Hello World')

    # Receive bytes
    data = port.read(1024)

See also `Mini serial terminal`_


SPI
~~~

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

|I2C|
~~~~~

Example: communication with an |I2C| GPIO expander

.. code-block:: python

    # Instanciate an I2C controller
    i2c = I2cController()

    # Configure the first interface (IF/1) of the FTDI device as an I2C master
    i2c.configure('ftdi://ftdi:2232h/1')

    # Get a port to an I2C slave device
    slave = i2c.get_port(0x21)

    # Send one byte, then receive one byte
    slave.exchange([0x04], 1)

    # Write a register to the I2C slave
    slave.write_to(0x06, b'\x00')

    # Read a register from the I2C slave
    slave.read_from(0x00, 1)

See also pyi2cflash_ module and `tests/i2c.py`_

GPIO
~~~~

See `tests/gpio.py`_

