# Copyright (c) 2014-2019, Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2016, Emmanuel Bouaziz <ebouaziz@free.fr>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Neotion nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL NEOTION BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""GPIO/BitBang support for PyFdti"""

from struct import pack as spack
from .ftdi import Ftdi


class GpioException(IOError):
    """Base class for GPIO errors.
    """


class GpioController:
    """GPIO controller for an FTDI port, in bit-bang legacy mode.
    """

    MASK = 0xFF

    def __init__(self):
        self._ftdi = Ftdi()
        self._direction = 0

    @property
    def is_connected(self) -> bool:
        """Reports whether a connection exists with the FTDI interface.

           :return: the FTDI slave connection status
        """
        return self._ftdi.is_connected

    def configure(self, url, direction=0, **kwargs):
        """Open a new interface to the specified FTDI device in bitbang mode.

           :param str url: a FTDI URL selector
           :param int direction: a bitfield specifying the FTDI GPIO direction,
                where high level defines an output, and low level defines an
                input
        """
        for k in ('direction',):
            if k in kwargs:
                del kwargs[k]
        try:
            self._ftdi.open_bitbang_from_url(url, direction=direction,
                                             **kwargs)
        except IOError as ex:
            raise GpioException('Unable to open USB port: %s' % str(ex))
        self._direction = direction & self.MASK
        if 'initial' in kwargs:
            self.write(kwargs['initial'] & self.MASK)

    def close(self):
        """Close the FTDI interface.
        """
        if self._ftdi.is_connected:
            self._ftdi.close()

    @property
    def direction(self) -> int:
        """Reports the GPIO direction.

          :return: a bitfield specifying the FTDI GPIO direction, where high
                level reports an output pin, and low level reports an input pin
        """
        return self._direction

    @property
    def pins(self) -> int:
        """Report the configured GPIOs as a bitfield.

           A true bit represents a GPIO, a false bit a reserved or not
           configured pin.

           :return: always 0xFF.
        """
        return self.MASK

    @property
    def all_pins(self) -> int:
        """Report the addressable GPIOs as a bitfield.

           A true bit represents a pin which may be used as a GPIO, a false bit
           a reserved pin

           :return: always 0xFF.
        """
        return self.MASK

    @property
    def width(self) -> int:
        """Report the FTDI count of addressable pins.

           :return: always 8.
        """
        return 8

    def set_direction(self, pins: int, direction: int) -> None:
        """Update the GPIO pin direction.

           :param pins: which GPIO pins should be reconfigured
           :param direction: a bitfield of GPIO pins. Each bit represent a
                GPIO pin, where a high level sets the pin as output and a low
                level sets the pin as input/high-Z.
        """
        if direction > self.MASK:
            raise GpioException("Invalid direction mask")
        self._direction &= ~pins
        self._direction |= (pins & direction)
        self._ftdi.set_bitmode(self._direction, Ftdi.BITMODE_BITBANG)

    def read(self) -> int:
        """Read the GPIO input pin electrical level.

           :param value: a bitfield of GPIO pins. Each bit represent a GPIO
                pin, matching the logical input level of the pin.
        """
        if not self.is_connected:
            raise GpioException('Not connected')
        return self._ftdi.read_pins()

    def write(self, value: int) -> None:
        """Set the GPIO output pin electrical level.

           :param int value: a bitfield of GPIO pins.
        """
        if not self.is_connected:
            raise GpioException('Not connected')
        if value > self.MASK:
            raise GpioException("Invalid value")
        self._ftdi.write_data(spack('<B', value))

    # old API names
    open_from_url = configure
    read_port = read
    write_port = write
