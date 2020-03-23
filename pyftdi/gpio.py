# Copyright (c) 2014-2020, Emmanuel Blot <emmanuel.blot@free.fr>
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

#pylint: disable-msg=too-few-public-methods

from struct import calcsize as scalc, unpack as sunpack
from typing import Iterable, Tuple, Union
from .ftdi import Ftdi, FtdiError
from .misc import is_iterable


class GpioException(FtdiError):
    """Base class for GPIO errors.
    """


class GpioPort:
    """Duck-type GPIO port for GPIO controller.
    """


class GpioBaseController(GpioPort):
    """GPIO controller for an FTDI port, in bit-bang legacy mode.

       GPIO bit-bang mode is limited to the 8 lower pins of each GPIO port.
    """

    def __init__(self):
        self._ftdi = Ftdi()
        self._direction = 0
        self._width = 0
        self._mask = 0
        self._frequency = 0.0

    @property
    def ftdi(self) -> Ftdi:
        """Return the Ftdi instance.

           :return: the Ftdi instance
        """
        return self._ftdi

    @property
    def is_connected(self) -> bool:
        """Reports whether a connection exists with the FTDI interface.

           :return: the FTDI slave connection status
        """
        return self._ftdi.is_connected

    def configure(self, url: str, direction: int = 0,
                  **kwargs):
        """Open a new interface to the specified FTDI device in bitbang mode.

           :param str url: a FTDI URL selector
           :param int direction: a bitfield specifying the FTDI GPIO direction,
                where high level defines an output, and low level defines an
                input
           :param initial: optional initial GPIO output value
           :param frequency: optional frequency in Hz
           :return: actual bitbang frequency in Hz
        """
        if self.is_connected:
            raise FtdiError('Already connected')
        self._frequency = self._configure(url, direction, **kwargs)
        self._mask = (1 << self._width) - 1
        self._direction = direction & self._mask

    def close(self):
        """Close the FTDI interface.
        """
        if self._ftdi.is_connected:
            self._ftdi.close()

    def get_gpio(self) -> GpioPort:
        """Retrieve the GPIO port.

           This method is mostly useless, it is a wrapper to duck type other
           GPIO APIs (I2C, SPI, ...)

           :return: GPIO port
        """
        return self

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

           :return: always 0xFF for GpioController instance.
        """
        return self._mask

    @property
    def all_pins(self) -> int:
        """Report the addressable GPIOs as a bitfield.

           A true bit represents a pin which may be used as a GPIO, a false bit
           a reserved pin

           :return: always 0xFF for GpioController instance.
        """
        return self._mask

    @property
    def width(self) -> int:
        """Report the FTDI count of addressable pins.

           :return: the width of the GPIO port.
        """
        return self._width

    def set_direction(self, pins: int, direction: int) -> None:
        """Update the GPIO pin direction.

           :param pins: which GPIO pins should be reconfigured
           :param direction: a bitfield of GPIO pins. Each bit represent a
                GPIO pin, where a high level sets the pin as output and a low
                level sets the pin as input/high-Z.
        """
        if direction > self._mask:
            raise GpioException("Invalid direction mask")
        self._direction &= ~pins
        self._direction |= (pins & direction)
        self._ftdi.set_bitmode(self._direction, Ftdi.BITMODE_BITBANG)

    def _configure(self, url, direction, **kwargs):
        raise NotImplementedError('GpioBaseController cannot be instanciated')


class GpioAsyncController(GpioBaseController):
    """GPIO controller for an FTDI port, in bit-bang asynchronous mode.

       GPIO accessible pins are limited to the 8 lower pins of each GPIO port.
    """

    def _configure(self, url, direction, **kwargs):
        for k in ('direction', 'sync'):
            if k in kwargs:
                del kwargs[k]
        frequency = self._ftdi.open_bitbang_from_url(url,
                                                     direction=direction,
                                                     sync=False,
                                                     **kwargs)
        if 'initial' in kwargs:
            self.write(kwargs['initial'] & self._mask)
        self._width = 8
        return frequency

    def read(self, direct: bool = True, readlen: int = 1) -> \
             Union[int, bytes]:
        """Read the GPIO input pin electrical level.

           :param direct: whether to peak current value from port, or to use
                          the HW FIFO. When direct mode is selected, readlen
                          should be 1. This matches the behaviour of the legacy
                          API.
           :param readlen: how many GPIO samples to retrieve. Each sample if
                           8-bit wide.
           :return: a 8-bit wide integer if direct mode is used, or
                    a :py:type:`bytes`` buffer otherwise.
        """
        if not self.is_connected:
            raise GpioException('Not connected')
        if direct:
            if readlen != 1:
                raise ValueError('Invalid read length with direct mode')
        if direct:
            return self._ftdi.read_pins()
        return self._ftdi.read_data(readlen)

    def write(self, out: Union[bytes, bytearray, int]) -> None:
        """Set the GPIO output pin electrical level, or output a sequence of
           bytes @ constant frequency to GPIO output pins.

           :param out: a bitfield of GPIO pins, or a sequence of them
        """
        if not self.is_connected:
            raise GpioException('Not connected')
        if isinstance(out, (bytes, bytearray)):
            pass
        else:
            if isinstance(out, int):
                out = bytes([out])
            elif not is_iterable(out):
                raise TypeError('Invalid output value')
            for val in out:
                if val > self._mask:
                    raise GpioException("Invalid value")
        self._ftdi.write_data(out)

    # old API names
    open_from_url = GpioBaseController.configure
    read_port = read
    write_port = write


# old API compatibility
GpioController = GpioAsyncController


class GpioSyncController(GpioBaseController):
    """GPIO controller for an FTDI port, in bit-bang synchronous mode.

       GPIO accessible pins are limited to the 8 lower pins of each GPIO port.
    """

    def exchange(self, out: Union[bytes, bytearray]) -> bytes:
        """Set the GPIO output pin electrical level, or output a sequence of
           bytes @ constant frequency to GPIO output pins.

           :param out: the byte buffer to output as GPIO
           :return: a byte buffer of the same length as out buffer.
        """
        if not self.is_connected:
            raise GpioException('Not connected')
        if isinstance(out, (bytes, bytearray)):
            pass
        else:
            if isinstance(out, int):
                out = bytes([out])
            elif not is_iterable(out):
                raise TypeError('Invalid output value')
            for val in out:
                if val > self._mask:
                    raise GpioException("Invalid value")
        self._ftdi.write_data(out)
        return self._ftdi.read_data(len(out))

    def _configure(self, url, direction, **kwargs):
        for k in ('direction', 'sync'):
            if k in kwargs:
                del kwargs[k]
        frequency = self._ftdi.open_bitbang_from_url(url,
                                                     direction=direction,
                                                     sync=True,
                                                     **kwargs)
        if 'initial' in kwargs:
            self.exchange(kwargs['initial'] & self._mask)
        self._width = 8
        return frequency


class GpioMpsseController(GpioBaseController):
    """GPIO controller for an FTDI port, in MPSSE mode.

       All GPIO pins are reachable, but MPSSE mode is slower than other modes.
    """


    MPSSE_PAYLOAD_MAX_LENGTH = 0xFF00  # 16 bits max (- spare for control)

    def read(self, direct: bool = True, readlen: int = 1) -> \
             Union[int, bytes, Tuple[int]]:
        """Read the GPIO input pin electrical level.

           :param direct: whether to peak current value from port, or to use
                          the HW FIFO. When direct mode is selected, readlen
                          should be 1. This matches the behaviour of the legacy
                          API.
           :param readlen: how many GPIO samples to retrieve. Each sample if
                           :py:meth:`width` bit wide.
           :return: a :py:meth:`width` bit wide integer if direct mode is used,
                    a :py:type:`bytes`` buffer if :py:meth:`width` is a byte,
                    a list of integer otherwise (MPSSE mode only).
        """
        if not self.is_connected:
            raise GpioException('Not connected')
        if direct:
            if readlen != 1:
                raise ValueError('Invalid read length with direct mode')
        if direct:
            return self._ftdi.read_pins()
        return self._read_mpsse(readlen)

    def write(self, out: Union[bytes, bytearray, Iterable[int], int]) -> None:
        """Set the GPIO output pin electrical level, or output a sequence of
           bytes @ constant frequency to GPIO output pins.

           :param out: a bitfield of GPIO pins, or a sequence of them
        """
        if not self.is_connected:
            raise GpioException('Not connected')
        if isinstance(out, (bytes, bytearray)):
            pass
        else:
            if isinstance(out, int):
                out = bytes([out])
            elif not is_iterable(out):
                raise TypeError('Invalid output value')
            for val in out:
                if val > self._mask:
                    raise GpioException("Invalid value")
        self._write_mpsse(out)

    def _configure(self, url, direction, **kwargs):
        frequency = self._ftdi.open_mpsse_from_url(url,
                                                   direction=direction,
                                                   **kwargs)
        self._width = self._ftdi.port_width
        return frequency

    def _read_mpsse(self, count: int) -> Tuple[int]:
        if self._width > 8:
            cmd = bytearray([Ftdi.GET_BITS_LOW, Ftdi.GET_BITS_HIGH] * count)
            fmt = '<%dH' % count
        else:
            cmd = bytearray([Ftdi.GET_BITS_LOW] * count)
            fmt = None
        cmd.append(Ftdi.SEND_IMMEDIATE)
        if len(cmd) > self.MPSSE_PAYLOAD_MAX_LENGTH:
            raise ValueError('Too many samples')
        self._ftdi.write_data(cmd)
        size = scalc(fmt) if fmt else count
        data = self._ftdi.read_data_bytes(size, 4)
        if len(data) != size:
            raise FtdiError('Cannot read GPIO')
        if fmt:
            return sunpack(fmt, data)
        return data

    def _write_mpsse(self,
                     out: Union[bytes, bytearray, Iterable[int], int]) -> None:
        cmd = []
        low_dir = self._direction & 0xFF
        if self._width > 8:
            high_dir = (self._direction >> 8) & 0xFF
            for data in out:
                low_data = data & 0xFF
                high_data = (data >> 8) & 0xFF
                cmd.extend([Ftdi.SET_BITS_LOW, low_data, low_dir,
                            Ftdi.SET_BITS_HIGH, high_data, high_dir])
        else:
            for data in out:
                cmd.extend([Ftdi.SET_BITS_LOW, data, low_dir])
        self._ftdi.write_data(bytes(cmd))
