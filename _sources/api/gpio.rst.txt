.. -*- coding: utf-8 -*-

.. include:: ../defs.rst

:mod:`gpio` - GPIO API
----------------------

.. module :: pyftdi.gpio

Direct drive GPIO pins of FTDI device.

.. note::

  This mode is mutually exclusive with advanced serial MPSSE features, such as
  |I2C|, SPI, JTAG, ...

  If you need to use GPIO pins and MPSSE interface on the same port, you need
  to use the dedicated API. For now, this shared mode is only supported with
  the :doc:`SPI API <spi>`.

Quickstart
~~~~~~~~~~

See ``tests/gpio.py`` example


Classes
~~~~~~~

.. autoclass :: GpioController
 :members:


Exceptions
~~~~~~~~~~

.. autoexception :: GpioException
