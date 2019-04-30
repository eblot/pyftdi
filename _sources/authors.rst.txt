Authors
-------

Main developers
~~~~~~~~~~~~~~~

 * Emmanuel Blot <emmanuel.blot@free.fr>
 * Emmanuel Bouaziz <ebouaziz@free.fr>

Contributors
~~~~~~~~~~~~

 * Nikus-V:

  * Added open device by description, USB timeout support, RTS bug fix

 * Dave McCoy:

  * Fix a long-lasting syntax issue with bus hi-speed management

 * Adam Feuer:

  * Add support for FT232H FTDI device

 * endlesscoil:

  * Add support for FT232R FTDI device
  * Add support for FT232x FTDI device

 * humm (Fabien Benureau):

  * Fix implementation for PyUSB 1.0.0b2 (API break)

 *  dlharmon:

  * JTAG speed optimisation for long transfer

 * DavidWC:

  * Update PyUSB API adaptation code

 * Sebastian:

  * Fix bitmask definition

 * Anders (anders-code):

  * Remove the double-layer directory structure, an historical mess :-)

 * Andrea Concil:

  * Fix SPI implementation so that any SPI /CS can be addressed
  * Documentation hints for installation on Windows hosts

 * Darren Garnier:

  * Fix open() arguments (issue #40)

 * Michael Leonard:

  * Document FTDI URL schemes

 * nopeppermint (Stefan):

  * Documentation reviewer and editor

 * hannesweisbach:

  * I2C poll mode (read/write)
  * I2C soft tristate for FT2232H/FT4232H devices

 * Vianney le Clément de Saint-Marcq:

  * Fix timeout and remove useless delay in UART mode:

 * Pete Schwamb:

  * Fix issue with SPI duplex mode

 * Will Richey:

  * makes write() compatible with write() from pySerial.

 * sgoadhouse:

  * JTAG constant definitions debunking and engine instanciation fixes.

 * tavip (Octavian Purdila):

  * Allow simulatenous usage with kernel drivers (per FTDI port management)

 * Tim Legrand:

  * Fix invalid package configuration

 * vestom:

  * Fix divisor calculation for frequency generation and improve frequency
    selection.
