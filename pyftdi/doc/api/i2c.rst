
.. include:: ../defs.rst

:mod:`i2c` - |I2C| API
----------------------

.. module :: pyftdi.i2c


Quickstart
~~~~~~~~~~

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

Example: creating an |I2C| controller from an existing FTDI device

.. code-block:: python

    # Open an FTDI device
    ftdi = Ftdi.create_from_url('ftdi://ftdi:2232h/1')

    # Instanciate an I2C controller from an existing FTDI device
    i2c = I2cController(ftdi)

    # Configure the FTDI device as an I2C master
    i2c.configure()

Example: mastering the |I2C| bus with a complex transaction

.. code-block:: python

   from time import sleep

   port = I2cController().get_port(0x56)

   # emit a START sequence is read address, but read no data and keep the bus
   # busy
   port.read(0, relax=False)

   # wait for ~1ms
   sleep(0.001)

   # write 4 bytes, without neither emitting the start or stop sequence
   port.write(b'\x00\x01', relax=False, start=False)

   # read 4 bytes, without emitting the start sequence, and release the bus
   port.read(4, start=False)

See also pyi2cflash_ module and `tests/i2c.py`_


Classes
~~~~~~~

.. autoclass :: I2cController
 :members:

.. autoclass :: I2cPort
 :members:


Exceptions
~~~~~~~~~~

.. autoexception :: I2cIOError
.. autoexception :: I2cNackError


Tests
~~~~~

|I2C| sample tests expect:
  * TCA9555 device on slave address 0x21
  * ADXL345 device on slave address 0x53

Checkout a fresh copy from PyFtdi_ github repository.

See :doc:`../pinout` for FTDI wiring.

.. code-block:: shell

   # optional: specify an alternative FTDI device
   export FTDI_DEVICE=ftdi://ftdi:2232h/1
   # optional: increase log level
   export FTDI_LOGLEVEL=DEBUG
   # be sure to connect the appropriate I2C slaves to the FTDI I2C bus and run
   PYTHONPATH=. python3 pyftdi/tests/i2c.py
