.. include:: defs.rst

FTDI device pinout
------------------

============ ============= ====== ============== ======= ======
 IF/1         IF/2 [#if2]_  UART   |I2C|          SPI     JTAG
============ ============= ====== ============== ======= ======
 ``ADBUS0``   ``BDBUS0``    TxD    SCK            SCLK    TCK
 ``ADBUS1``   ``BDBUS1``    RxD    SDA/O [#i2c]_  MOSI    TDI
 ``ADBUS2``   ``BDBUS2``    RTS    SDA/I [#i2c]_  MISO    TDO
 ``ADBUS3``   ``BDBUS3``    CTS                   CS0     TMS
 ``ADBUS4``   ``BDBUS4``                          CS1
 ``ADBUS5``   ``BDBUS5``                          CS2
 ``ADBUS6``   ``BDBUS6``                          CS3
============ ============= ====== ============== ======= ======

.. [#i2c] FTDI pins are either configured as input or output. As |I2C| SDA line
          is bi-directional, two FTDI pins are required to provide the SDA
          feature, and they should be connected together and to the SDA |I2C|
          bus line. Pull-up resistors on SCK and SDA lines should be used.
.. [#if2] FT232H_ does not support a secondary MPSSE port, only FT2232H_ and
          FT4232H_ do. Note that FT4232H_ has 4 serial ports, but only the
          first two interfaces are MPSSE-capable.
