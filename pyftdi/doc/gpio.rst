.. include:: defs.rst

GPIOs
-----

Overview
~~~~~~~~

Many PyFtdi APIs give direct access to the IO pins of the FTDI devices:

  * `GpioController` (see :doc:`api/gpio`) gives full access to the FTDI pins as raw I/O pins,
  * `SpiGpioPort` (see :doc:`api/spi`) gives access to all free pins of an FTDI interface, a.k.a. port, which are not reserved for the SPI feature,
  * `I2cGpioPort` (see :doc:`api/i2c`) gives access to all free pins of an FTDI interface, a.k.a. port, which are not reserved for the I2C feature

.. hint:: Gpio raw access is not yet supported with JTAG feature

This document presents the common definitions for these APIs and explain how to drive those pins.


Definitions
~~~~~~~~~~~

Ports
`````

Each port can be accessed as raw input/output pins. At a given time, a pin is either configured as an input or an output function.

The width of a port, that is the number of pins of the interface, depending on the actual hardware, *i.e.* the FTDI model:

* FT232R features a single port, which is 8-bit wide: `DBUS`
* FT232H features a single port, which is 16-bit wide: `ADBUS/ACBUS`
* FT2232D features two ports, which are 12-bit wide each: `ADBUS/ACBUS` and
  `BDBUS/BCBUS`
* FT2232H features two ports, which are 16-bit wide each: `ADBUS/ACBUS` and
  `BDBUS/BCBUS`
* FT4232H features four ports, which are 8-bit wide each: `ADBUS`, `BDBUS`,
  `CDBUS` and `DDBUS`.
* FT230X features a single port, which is 4-bit wide
* FT231X feature a single port, which is 8-bit wide

.. note::

   FT232R and FT230X/FT231X support an additional port denoted CBUS:

   * FT232R provides an additional 5-bit wide port
   * FT230X/FT231X provides an additional 4-bit wide port

   Accessing this extra port requires a specific EEPROM configuration, *i.e.*
   it cannot be able with a software request till a specific configuration is
   defined in the EEPROM map. This feature and access to this port is not yet
   supported by PyFTDI.

For historical reasons, 16-bit ports used to be named *wide* ports and 8-bit
ports used to be called *narrow*. This terminology - and API - is no longer
used, but preserved to prevent API break. Please only use port ``width``
rather than port types.

GPIO value
``````````

* A logical ``0`` bit represents a low level value on a pin, that is *GND*
* A logical ``1`` bit represents a high level value on a pin, that is *Vdd*
  which is typically 3.3 volts on most FTDIs

Please refers to the FTDI datasheet of your device for the tolerance and
supported analog levels for more details

.. hint::

   FT232H supports a specific feature, which is dedicated to better supporting
   the |I2C| feature. This specific devices enables an open-collector mode:

   * Setting a pin to a low level drains it to *GND*
   * Setting a pin to a high level sets the pin as High-Z

   This feature is automatically activated when |I2C| feature is enabled on a
   port, for the two first pins, i.e. `SCL` and `SDA out`.

   However., PyFTDI does not yet provide an API to enable this mode to the
   other pins of a port, *i.e.* for the pins used as GPIOs.


Direction
`````````

An FTDI pin should either be configured as an input or an ouput. It is
mandatory to (re)configure the direction of a pin before changing the way it is
used.

* A logical ``0`` bit represents an input pin, *i.e.* a pin whose value can be
  sampled and read via the PyFTDI APIs
* A logical ``1`` bit represents an output pin, *i.e.* a pin whose value can be
  set/written with the PyFTDI APIs


Configuration
~~~~~~~~~~~~~

GPIO bitmap
```````````

The GPIO pins of a port are always accessed as an integer, whose supported
width depends on the width of the port. These integers should be considered as
a bitmap of pins, and are always assigned the same mapping, whatever feature is
enabled:

* b\ :sub:`0`\  (``0x01``) represents the first pin of a port, *i.e.* AD0/BD0
* b\ :sub:`1`\  (``0x02``) represents the second pin of a port, *i.e.* AD1/BD1
* ...
* b\ :sub:`7`\  (``0x80``) represents the seventh pin of a port, *i.e.* AD7/BD7
* b\ :sub:`N`\  represents the highest pin of a port, *i.e.* AD7/BD7 for an
  8-bit port, AD15/BD15 for a 16-bit port, etc.

Pins reserved for a specific feature (|I2C|, SPI, ...) cannot be accessed as
a regular GPIO. They cannot be arbitrarily written and should be masked out
when the GPIO output value is set. See :ref:`reserved_pins` for details.


Direction bitmap
````````````````

Before using a port as GPIO, the port must be configured as GPIO. This is
achieved by either instanciating a ``GpioController`` or by requesting the
GPIO port from a specific controller: ``I2cController.get_gpio()`` and
``SpiController.get_gpio()``. All instances provide a similar API (duck typing
API) to configure, read and write to GPIO pins.

Once a GPIO port is instanciated, the direction of each pin should be defined.
The direction can be changed at any time. It is not possible to write to /
read from a pin before the proper direction has been defined.

To configure the direction, use the `set_direction` API with a bitmap integer
value that defines the direction to use of each pin.

Direction example
.................

A 8-bit port, dedicated to GPIO, is configured as follows:

 * BD0, BD3, BD7: input, `I` for short
 * BD1-BD2, BD4-BD6: output, `O` for short

That is, MSB to LSB: *I O O O I O O I*.

This translates to 0b ``0111 0110`` as output is ``1`` and input is ``0``,
that is ``0x76`` as an hexa value. This is the direction value to use to
``configure()`` the port.

See also the ``set_direction()`` API to reconfigure the direction of GPIO pins
at any time. This method accepts two arguments. This first arguments,
``pins``, defines which pins - the ones with the maching bit set - to consider in the second ``direction`` argument, so there is no need to preserve/read-modify-copy the configuration of other pins. Pins with their matching bit
reset are not reconfigured, whatever their direction bit.

.. code-block:: python

    gpio = GpioController()
    gpio.configure('ftdi:///1', direction=0x76)
    # later, reconfigure BD2 as input and BD7 as output
    gpio.set_direction(0x84, 0x80)


Using GPIO APIs
~~~~~~~~~~~~~~~

.. warning::

  GpioController (see :doc:`api/gpio`) relies on legacy bit-band mode of
  FTDI device. This mode only gives access to the lower 8 bit of each FTDI
  port. This means that this API cannot be used to access the upport 4- or
  8- bit of respectively 12- or 16- bit capable ports.

  To access those upper pins, please use the GPIO mode of either |I2C| or SPI
  APIs. Unfortenately these APIs do not give access to the LSB (3- or 4- bits)
  of the port, as they are dedicated to |I2C| and SPI signalling.


Setting GPIO pin state
``````````````````````

To write to a GPIO, use the `write()` method. The caller needs to mask out
the bits configured as input, or an exception is triggered:

* writing ``0`` to an input pin is ignored
* writing ``1`` to an input pin raises an exception

.. code-block:: python

    gpio = GpioController()
    gpio.configure('ftdi:///1', direction=0x76)
    # all output set low
    gpio.write(0x00)
    # all output set high
    gpio.write(0x76)
    # all output set high, apply direction mask
    gpio.write(0xFF & gpio.direction)
    # all output forced to high, writing to input pins is illegal
    gpio.write(0xFF)  # raises an IOError


Retrieving GPIO pin state
`````````````````````````

To read a GPIO, use the `read()` method.

.. code-block:: python

    gpio = GpioController()
    gpio.configure('ftdi:///1', direction=0x76)
    # read whole port
    pins = gpio.read()
    # ignore output values (optional)
    pins &= ~gpio.direction


Modifying GPIO pin state
````````````````````````

A read-modify-write sequence is required.

.. code-block:: python

    gpio = GpioController()
    gpio.configure('ftdi:///1', direction=0x76)
    # read whole port
    pins = gpio.read()
    # clearing out AD1 and AD2
    pins &= ~((1 << 1) | (1 << 2))  # or 0x06
    # want AD2=0, AD1=1
    pins |= 1 << 1
    # update GPIO output
    gpio.write(pins)


.. _reserved_pins:


Reserved pins
~~~~~~~~~~~~~

GPIO pins vs. feature pins
``````````````````````````

It is important to note that the reserved pins do not change the pin
assignment, *i.e.* the lowest pins of a port may become unavailable as regular
GPIO when the feature is enabled:

Example
.......

|I2C| feature reserves
the three first pins, as *SCL*, *SDA output*, *SDA input* (w/o clock stretching
feature which also reserves another pin). This means that AD0, AD1 and AD2,
that is b\ :sub:`0`\ , b\ :sub:`1`\ , b\ :sub:`2`\  cannot be directly
accessed.

The first accessible GPIO pin in this case is no longer AD0 but AD3, which
means that b\ :sub:`3`\ becomes the lowest bit which can be read/written.

.. code-block:: python

    # use I2C feature
    i2c = I2cController()
    # configure the I2C feature, and predefines the direction of the GPIO pins
    i2c.configure('ftdi:///1', direction=0x78)
    gpio = i2c.get_gpio()
    # read whole port
    pins = gpio.read()
    # clearing out I2C bits (SCL, SDAo, SDAi)
    pins &= 0x07
    # set AD4
    pins |= 1 << 4
    # update GPIO output
    gpio.write(pins)
