.. include:: defs.rst

FTDI device pinout
------------------

============ ============= ====== ============== ========== ====== ============
 IF/1 [#ih]_ IF/2 [#if2]_  UART   |I2C|          SPI        JTAG   C232HD cable
============ ============= ====== ============== ========== ====== ============
 ``ADBUS0``   ``BDBUS0``    TxD    SCK            SCLK       TCK   Orange
 ``ADBUS1``   ``BDBUS1``    RxD    SDA/O [#i2c]_  MOSI       TDI   Yellow
 ``ADBUS2``   ``BDBUS2``    RTS    SDA/I [#i2c]_  MISO       TDO   Green
 ``ADBUS3``   ``BDBUS3``    CTS                   CS0        TMS   Brown
 ``ADBUS4``   ``BDBUS4``                          CS1/GPIO4        Grey
 ``ADBUS5``   ``BDBUS5``                          CS2/GPIO5        Purple
 ``ADBUS6``   ``BDBUS6``                          CS3/GPIO6        White
 ``ADBUS7``   ``BDBUS7``           RSCK [#rck]_   CS4/GPIO7  RCLK  Blue
 ``ACBUS0``   ``BCBUS0``                          GPIO8
 ``ACBUS1``   ``BCBUS1``                          GPIO9
 ``ACBUS2``   ``BCBUS2``                          GPIO10
 ``ACBUS3``   ``BCBUS3``                          GPIO11
 ``ACBUS4``   ``BCBUS4``                          GPIO12
 ``ACBUS5``   ``BCBUS5``                          GPIO13
 ``ACBUS6``   ``BCBUS6``                          GPIO14
 ``ACBUS7``   ``BCBUS7``                          GPIO15
============ ============= ====== ============== ========== ====== ============

.. [#ih]  16-bit port (ACBUS, BCBUS) is not available with FT4232H_ series
.. [#i2c] FTDI pins are either configured as input or output. As |I2C| SDA line
          is bi-directional, two FTDI pins are required to provide the SDA
          feature, and they should be connected together and to the SDA |I2C|
          bus line. Pull-up resistors on SCK and SDA lines should be used.
.. [#if2] FT232H_ does not support a secondary MPSSE port, only FT2232H_ and
          FT4232H_ do. Note that FT4232H_ has 4 serial ports, but only the
          first two interfaces are MPSSE-capable. C232HD cable only exposes
          IF/1 (ADBUS).
.. [#rck] In order to support I2C clock stretch mode, ADBUS7 should be
          connected to SCK.