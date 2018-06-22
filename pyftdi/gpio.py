"""GPIO/BitBang support for PyFdti"""

# Copyright (c) 2014-2018, Emmanuel Blot <emmanuel.blot@free.fr>
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

from array import array
from pyftdi.ftdi import Ftdi
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from threading import Lock
from logging import getLogger


__all__ = ['GpioController']


class GpioException(IOError):
    """Base class for GPIO errors"""


class GpioController:
    """GPIO controller for an FTDI port

       Bitfield size depends on the FTDI device: 4432H series use 8-bit GPIO
       ports, while 232H and 2232H series use wide 16-bit ports.

       """

    def __init__(self):
        self._ftdi = None
        self.log = getLogger('pyftdi.gpio')
        self._direction = 0
        self._wide_port = False
        self._lock = Lock()
        self._gpio_mask = 0

    @property
    def pins(self):
        """Report the addressable GPIOs as a bitfield."""
        return self._gpio_mask

    @property
    def direction(self):
        """Reports the GPIO direction.

          :return: a bitfield specifying the FTDI GPIO direction, where high
                level reports an output pin, and low level reports an input pin
          :rtype: int
        """
        return self._direction

    @property
    def is_connected(self):
        """Reports whether a connection exists with the FTDI interface."""
        return bool(self._ftdi)

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
            ftdi = Ftdi()
            ftdi.open_bitbang_from_url(url, direction=direction, **kwargs)
            self._ftdi = ftdi
        except IOError as ex:
            raise GpioException('Unable to open USB port: %s' % str(ex))
        self._direction = direction

        with self._lock:
            if (self._ftdi.has_wide_port and self._ftdi.has_mpsse):
                # If this device supports the wide 16-bit GPIO port,
                # must open as MPSSE to access the 16-bits. So close
                # ftdi and re-open it as MPSSE.
                try:
                    self._ftdi.close()
                    ftdi = Ftdi()
                    ftdi.open_mpsse_from_url(url, direction=direction, **kwargs)
                    self._ftdi = ftdi
                except IOError as ex:
                    raise GpioException('Unable to open USB port: %s' % str(ex))

            self._wide_port = self._ftdi.has_wide_port
            
            gpio_width = self._wide_port and 16 or 8
            gpio_mask = (1 << gpio_width) - 1
            self._gpio_mask =  gpio_mask
        
    def close(self):
        """Close the FTDI interface."""
        if self._ftdi:
            self._ftdi.close()
            self._ftdi = None

    def set_direction(self, pins, direction):
        """Update the GPIO pin direction.

           :param int pins: which GPIO pins should be reconfigured
           :param int direction: a bitfield of GPIO pins. Each bit represent a
                GPIO pin, where a high level sets the pin as output and a low
                level sets the pin as input/high-Z.
        """
        gpio_width = self._wide_port and 16 or 8
        gpio_mask = (1 << gpio_width) - 1
        if (pins & gpio_mask) != pins:
            raise GpioException("Invalid direction mask")
        self._direction &= ~pins
        self._direction |= (pins & direction)
        self._gpio_mask = gpio_mask & pins

        if (not self._wide_port):
            self._ftdi.set_bitmode(self._direction, Ftdi.BITMODE_BITBANG)

    def read(self):
        """Read the GPIO input pin electrical level.

           :param int value: a bitfield of GPIO pins. Each bit represent a GPIO
                pin, matching the logical input level of the pin.
        """
        if not self.is_connected:
            raise GpioException('Not connected')

        with self._lock:
            data = self._read_raw(self._wide_port)
        value = data & self._gpio_mask

        dbg = ('read: wide: 0x{:x}  data: 0x{:x}  mask: 0x{:x}  value: 0x{:x}'
                 .format(self._wide_port, data, self._gpio_mask, value))
        self.log.debug(dbg)

        return value
    
    def write(self, value):
        """Set the GPIO output pin electrical level.

           :param int value: a bitfield of GPIO pins.
        """
        if not self.is_connected:
            raise GpioException('Not connected')

        with self._lock:
            if (value & self._direction) != value:
                raise GpioException('No such GPO pins: %04x/%04x' %
                                        (self._direction, value))
            # perform read-modify-write
            use_high = self._wide_port and (self._direction & 0xff00)
            data = self._read_raw(use_high)

            dbg = ('write: use_high: 0x{:x}  data: 0x{:x}  mask: 0x{:x}'
                       .format(use_high, data, self._gpio_mask))
            self.log.debug(dbg)
            
            data &= ~self._gpio_mask
            data |= value

            dbg = 'write: use_high: 0x{:x}  data: 0x{:x}'.format(use_high, data)
            self.log.debug(dbg)
            
            self._write_raw(data, use_high)
        
    def _read_raw(self, read_high):
        if read_high:
            cmd = array('B', [Ftdi.GET_BITS_LOW,
                              Ftdi.GET_BITS_HIGH,
                              Ftdi.SEND_IMMEDIATE])
            fmt = '<H'
            self._ftdi.write_data(cmd)
            size = scalc(fmt)
            data = self._ftdi.read_data_bytes(size, 4)
            if len(data) != size:
                raise GpioException('Cannot read GPIO')
            value, = sunpack(fmt, data)
        else:
            # If not using read_high method, then also means this is
            # BIT BANG and not MPSSE, so just use read_pins()
            value = self._ftdi.read_pins()
        
        return value

    def _write_raw(self, data, write_high):
        direction = self.direction
        low_data = data & 0xFF
        low_dir = direction & 0xFF
        if write_high:
            high_data = (data >> 8) & 0xFF
            high_dir = (direction >> 8) & 0xFF
            cmd = array('B', [Ftdi.SET_BITS_LOW, low_data, low_dir,
                              Ftdi.SET_BITS_HIGH, high_data, high_dir])
        else:
            # If not using read_high method, then also means this is
            # BIT BANG and not MPSSE, so just write the data - no CMD
            # needed
            cmd = spack('<B', low_data)
            
        self._ftdi.write_data(cmd)
        
    # old API names
    open_from_url = configure
    read_port = read
    write_port = write
