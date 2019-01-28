"""SPI support for PyFdti"""

# Copyright (c) 2010-2019, Emmanuel Blot <emmanuel.blot@free.fr>
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
from logging import getLogger
from pyftdi.ftdi import Ftdi, FtdiError
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from threading import Lock


class SpiIOError(FtdiError):
    """SPI I/O error"""


class SpiPort:
    """SPI port

       An SPI port is never instanciated directly: use SpiController.get_port()
       method to obtain an SPI port.

       Example:

       >>> ctrl = SpiController(silent_clock=False)
       >>> ctrl.configure('ftdi://ftdi:232h/1')
       >>> spi = ctrl.get_port(1)
       >>> spi.set_frequency(1000000)
       >>> # send 2 bytes
       >>> spi.exchange([0x12, 0x34])
       >>> # send 2 bytes, then receive 2 bytes
       >>> out = spi.exchange([0x12, 0x34], 2)
       >>> # send 2 bytes, then receive 4 bytes, manage the transaction
       >>> out = spi.exchange([0x12, 0x34], 2, True, False)
       >>> out.extend(spi.exchange([], 2, False, True))
    """

    def __init__(self, controller, cs, cs_hold=3, spi_mode=0):
        self.log = getLogger('pyftdi.spi.port')
        self._controller = controller
        self._cpol = spi_mode & 0x1
        self._cpha = spi_mode & 0x2
        cs_clock = 0xFF & ~((int(not self._cpol) and SpiController.SCK_BIT) |
                            SpiController.DO_BIT)
        cs_select = 0xFF & ~((SpiController.CS_BIT << cs) |
                             (int(not self._cpol) and SpiController.SCK_BIT) |
                             SpiController.DO_BIT)
        self._cs_prolog = bytes([cs_clock, cs_select])
        self._cs_epilog = bytes([cs_select] + [cs_clock] * int(cs_hold))
        self._frequency = self._controller.frequency

    def exchange(self, out=b'', readlen=0, start=True, stop=True,
                 duplex=False):
        """Perform an exchange or a transaction with the SPI slave

           .. note:: Exchange is a dual half-duplex transmission: output bytes
                     are sent to the slave, then bytes are received from the
                     slave. It is not possible to perform a full duplex
                     exchange for now, although this feature could be easily
                     implemented.

           :param out: data to send to the SPI slave, may be empty to read out
                       data from the slave with no write.
           :type out: array or bytes or list(int)
           :param int readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param bool start: whether to start an SPI transaction, i.e.
                        activate the /CS line for the slave. Use False to
                        resume a previously started transaction
           :param bool stop: whether to desactivete the /CS line for the slave.
                       Use False if the transaction should complete with a
                       further call to exchange()
           :param duplex: perform a full-duplex exchange (vs. half-duplex),
                    i.e. bits are clocked in and out at once.
           :return: an array of bytes containing the data read out from the
                    slave
           :rtype: array
        """
        return self._controller.exchange(self._frequency, out, readlen,
                                         start and self._cs_prolog,
                                         stop and self._cs_epilog,
                                         self._cpol, self._cpha,
                                         duplex=duplex)

    def read(self, readlen=0, start=True, stop=True):
        """Read out bytes from the slave

           :param int readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param bool start: whether to start an SPI transaction, i.e.
                        activate the /CS line for the slave. Use False to
                        resume a previously started transaction
           :param bool stop: whether to desactivete the /CS line for the slave.
                       Use False if the transaction should complete with a
                       further call to exchange()
           :return: an array of bytes containing the data read out from the
                    slave
           :rtype: array
        """
        return self._controller.exchange(self._frequency, [], readlen,
                                         start and self._cs_prolog,
                                         stop and self._cs_epilog,
                                         self._cpol, self._cpha)

    def write(self, out, start=True, stop=True):
        """Write bytes to the slave

           :param out: data to send to the SPI slave, may be empty to read out
                       data from the slave with no write.
           :type out: array or bytes or list(int)
           :param bool start: whether to start an SPI transaction, i.e.
                        activate the /CS line for the slave. Use False to
                        resume a previously started transaction
           :param bool stop: whether to desactivete the /CS line for the slave.
                       Use False if the transaction should complete with a
                       further call to exchange()
        """
        return self._controller.exchange(self._frequency, out, 0,
                                         start and self._cs_prolog,
                                         stop and self._cs_epilog,
                                         self._cpol, self._cpha)

    def flush(self):
        """Force the flush of the HW FIFOs"""
        self._controller._flush()

    def set_frequency(self, frequency):
        """Change SPI bus frequency

           :param float frequency: the new frequency in Hz
        """
        self._frequency = min(frequency, self._controller.frequency_max)

    @property
    def frequency(self):
        """Return the current SPI bus block"""
        return self._frequency


class SpiGpioPort:
    """GPIO port

       A SpiGpioPort instance enables to drive GPIOs wich are not reseerved for
       SPI feature as regular GPIOs.

       GPIO are managed as a bitfield. The LSBs are reserved for the SPI
       feature, which means that the lowest pin that can be used as a GPIO is
       b4:

       * b0: SPI SCLK
       * b1: SPI MOSI
       * b2: SPI MISO
       * b3: SPI CS0
       * b4: SPI CS1 or first GPIO

       If more than one SPI device is used, less GPIO pins are available, see
       the cs_count argument of the SpiController constructor.

       There is no offset bias in GPIO bit position, *i.e.* the first available
       GPIO can be reached from as ``0x10``.

       Bitfield size depends on the FTDI device: 4432H series use 8-bit GPIO
       ports, while 232H and 2232H series use wide 16-bit ports.

       An SpiGpio port is never instanciated directly: use
       SpiController.get_gpio() method to obtain the GPIO port.
    """
    def __init__(self, controller):
        self.log = getLogger('pyftdi.spi.gpio')
        self._controller = controller

    @property
    def pins(self):
        """Report the configured GPIOs as a bitfield."""
        return self._controller.gpio_pins

    @property
    def all_pins(self):
        """Report the addressable GPIOs as a bitfield"""
        return self._controller.gpio_all_pins

    @property
    def width(self):
        """Report the FTDI count of addressable pins.

           Note that all pins, including reserved SPI ones, are reported.

           :return: the count of IO pins (including SPI ones).
        """
        return self._controller.width

    @property
    def direction(self):
        """Provide the FTDI GPIO direction"""
        return self._controller.direction

    def read(self, with_output=False):
        """Read GPIO port.

           :param bool with_output: set to unmask output pins
           :return: the GPIO port pins as a bitfield
           :rtype: int
        """
        return self._controller.read_gpio(with_output)

    def write(self, value):
        """Write GPIO port.

           :param int value: the GPIO port pins as a bitfield
        """
        return self._controller.write_gpio(value)

    def set_direction(self, pins, direction):
        """Change the direction of the GPIO pins.

           :param int pins: which GPIO pins should be reconfigured
           :param int direction: direction bitfield (high level for output)
        """
        self._controller.set_gpio_direction(pins, direction)


class SpiController:
    """SPI master.

        :param silent_clock: deprecated.
        :param int cs_count: is the number of /CS lines (one per device to
            drive on the SPI bus)
        :param boolean turbo: to be documented
    """

    SCK_BIT = 0x01
    DO_BIT = 0x02
    DI_BIT = 0x04
    CS_BIT = 0x08
    SPI_BITS = DI_BIT | DO_BIT | SCK_BIT
    PAYLOAD_MAX_LENGTH = 0x10000  # 16 bits max

    def __init__(self, silent_clock=False, cs_count=4, turbo=True):
        self.log = getLogger('pyftdi.spi.ctrl')
        self._ftdi = Ftdi()
        self._lock = Lock()
        self._gpio_port = None
        self._gpio_dir = 0
        self._gpio_mask = 0
        self._gpio_low = 0
        self._wide_port = False
        self._cs_count = cs_count
        self._turbo = turbo
        self._immediate = bytes((Ftdi.SEND_IMMEDIATE,))
        self._frequency = 0.0
        self._clock_phase = False

    @property
    def direction(self):
        """Provide the FTDI GPIO direction"""
        return self._spi_dir | self._gpio_dir

    def configure(self, url, **kwargs):
        """Configure the FTDI interface as a SPI master

           :param str url: FTDI URL string, such as 'ftdi://ftdi:232h/1'
           :param kwargs: options to configure the SPI bus

           Accepted options:

           * ``frequency`` the SPI bus frequency in Hz. Note that each slave
                          may reconfigure the SPI bus with a specialized
                          frequency.
           * ``cs_count`` count of chip select signals dedicated to select
                          SPI slave devices
           * ``turbo`` whether to enable or disable turbo mode
           * ``debug`` for extra debug output
        """
        # it is better to specify CS and turbo in configure, but the older
        # API where these parameters are specified at instanciation has been
        # preserved
        self._cs_count = int(kwargs.get('cs_count', self._cs_count))
        if not (1 <= self._cs_count <= 5):
            raise ValueError('Unsupported CS line count: %d' % self._cs_count)
        self._turbo = bool(kwargs.get('turbo', self._turbo))
        for k in ('direction', 'initial', 'cs_count', 'turbo'):
            if k in kwargs:
                del kwargs[k]
        with self._lock:
            if self._frequency > 0.0:
                raise SpiIOError('Already configured')
            self._cs_bits = (((SpiController.CS_BIT << self._cs_count) - 1) &
                             ~(SpiController.CS_BIT - 1))
            self._spi_ports = [None] * self._cs_count
            self._spi_dir = (self._cs_bits |
                             SpiController.DO_BIT |
                             SpiController.SCK_BIT)
            self._spi_mask = self._cs_bits | self.SPI_BITS
            self._frequency = self._ftdi.open_mpsse_from_url(
                # /CS all high
                url, direction=self._spi_dir, initial=self._cs_bits, **kwargs)
            self._ftdi.enable_adaptive_clock(False)
            self._wide_port = self._ftdi.has_wide_port

    def terminate(self):
        """Close the FTDI interface"""
        if self._ftdi:
            self._ftdi.close()
        self._frequency = 0.0

    def get_port(self, cs, freq=None, mode=0):
        """Obtain a SPI port to drive a SPI device selected by Chip Select.

           :note: SPI mode 2 is not supported.

           :param int cs: chip select slot, starting from 0
           :param float freq: SPI bus frequency for this slave in Hz
           :param int mode: SPI mode [0,1,3]
           :rtype: SpiPort
        """
        with self._lock:
            if not self._ftdi:
                raise SpiIOError("FTDI controller not initialized")
            if cs >= len(self._spi_ports):
                raise SpiIOError("No such SPI port")
            if not (0 <= mode <= 3):
                raise SpiIOError("Invalid SPI mode")
            if (mode & 0x2) and not self._ftdi.is_H_series:
                raise SpiIOError("SPI with CPHA high is not supported by "
                                 "this FTDI device")
            if mode == 2:
                raise SpiIOError("SPI mode 2 has no known workaround with "
                                 "FTDI devices")
            if not self._spi_ports[cs]:
                freq = min(freq or self._frequency, self.frequency_max)
                hold = freq and (1+int(1E6/freq))
                self._spi_ports[cs] = SpiPort(self, cs, cs_hold=hold,
                                              spi_mode=mode)
                self._spi_ports[cs].set_frequency(freq)
                self._flush()
            return self._spi_ports[cs]

    def get_gpio(self):
        with self._lock:
            if not self._ftdi:
                raise SpiIOError("FTDI controller not initialized")
            if not self._gpio_port:
                self._gpio_port = SpiGpioPort(self)
            return self._gpio_port

    @property
    def frequency_max(self):
        """Returns the maximum SPI clock"""
        return self._ftdi.frequency_max

    @property
    def frequency(self):
        """Returns the current SPI clock"""
        return self._frequency

    @property
    def gpio_pins(self):
        """Report the configured GPIOs as a bitfield"""
        with self._lock:
            return self._gpio_mask

    @property
    def gpio_all_pins(self):
        """Report the addressable GPIOs as a bitfield"""
        mask = (1 << self.width) - 1
        with self._lock:
            return mask & ~self._spi_mask

    @property
    def width(self):
        """Report the FTDI count of addressable pins.

           :return: the count of IO pins (including SPI ones).
        """
        return 16 if self._wide_port else 8

    def exchange(self, frequency, out, readlen,
                 cs_prolog=None, cs_epilog=None,
                 cpol=False, cpha=False, duplex=False):
        if duplex:
            if readlen > len(out):
                tmp = array('B', out)
                tmp.extend([0] * (readlen - len(out)))
                out = tmp
            elif not readlen:
                readlen = len(out)
        with self._lock:
            if duplex:
                data = self._exchange_full_duplex(frequency, out,
                                                  cs_prolog, cs_epilog,
                                                  cpol, cpha)
                return data[:readlen]
            else:
                return self._exchange_half_duplex(frequency, out, readlen,
                                                  cs_prolog, cs_epilog,
                                                  cpol, cpha)

    def read_gpio(self, with_output=False):
        """Read GPIO port

           :param bool with_output: set to unmask output pins
           :return: the GPIO port pins as a bitfield
           :rtype: int
        """
        with self._lock:
            data = self._read_raw(self._wide_port)
        value = data & self._gpio_mask
        if not with_output:
            value &= ~self._gpio_dir
        return value

    def write_gpio(self, value):
        """Write GPIO port

           :param int value: the GPIO port pins as a bitfield
        """
        with self._lock:
            if (value & self._gpio_dir) != value:
                raise SpiIOError('No such GPO pins: %04x/%04x' %
                                 (self._gpio_dir, value))
            # perform read-modify-write
            use_high = self._wide_port and (self.direction & 0xff00)
            data = self._read_raw(use_high)
            data &= ~self._gpio_mask
            data |= value
            self._write_raw(data, use_high)
            self._gpio_low = data & 0xFF & ~self._spi_mask

    def set_gpio_direction(self, pins, direction):
        """Change the direction of the GPIO pins

           :param int pins: which GPIO pins should be reconfigured
           :param int direction: direction bitfield (on for output)
        """
        with self._lock:
            if pins & self._spi_mask:
                raise SpiIOError('Cannot access SPI pins as GPIO')
            gpio_width = self._wide_port and 16 or 8
            gpio_mask = (1 << gpio_width) - 1
            gpio_mask &= ~self._spi_mask
            if (pins & gpio_mask) != pins:
                raise SpiIOError('No such GPIO pin(s)')
            self._gpio_dir &= ~pins
            self._gpio_dir |= (pins & direction)
            self._gpio_mask = gpio_mask & pins

    def _read_raw(self, read_high):
        if read_high:
            cmd = array('B', [Ftdi.GET_BITS_LOW,
                              Ftdi.GET_BITS_HIGH,
                              Ftdi.SEND_IMMEDIATE])
            fmt = '<H'
        else:
            cmd = array('B', [Ftdi.GET_BITS_LOW,
                              Ftdi.SEND_IMMEDIATE])
            fmt = 'B'
        self._ftdi.write_data(cmd)
        size = scalc(fmt)
        data = self._ftdi.read_data_bytes(size, 4)
        if len(data) != size:
            raise SpiIOError('Cannot read GPIO')
        value, = sunpack(fmt, data)
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
            cmd = array('B', [Ftdi.SET_BITS_LOW, low_data, low_dir])
        self._ftdi.write_data(cmd)

    def _exchange_half_duplex(self, frequency, out, readlen,
                              cs_prolog, cs_epilog, cpol, cpha):
        if not self._ftdi:
            raise SpiIOError("FTDI controller not initialized")
        if len(out) > SpiController.PAYLOAD_MAX_LENGTH:
            raise SpiIOError("Output payload is too large")
        if readlen > SpiController.PAYLOAD_MAX_LENGTH:
            raise SpiIOError("Input payload is too large")
        if cpha:
            # to enable CPHA, we need to use a workaround with FTDI device,
            # that is enable 3-phase clocking (which is usually dedicated to
            # I2C support). This mode use use 3 clock period instead of 2,
            # which implies the FTDI frequency should be fixed to match the
            # requested one.
            frequency = (3*frequency)//2
        if self._frequency != frequency:
            self._ftdi.set_frequency(frequency)
            # store the requested value, not the actual one (best effort),
            # to avoid setting unavailable values on each call.
            self._frequency = frequency
        direction = self.direction & 0xFF  # low bits only
        cmd = array('B')
        for ctrl in cs_prolog or []:
            ctrl &= self._spi_mask
            ctrl |= self._gpio_low
            cmd.extend((Ftdi.SET_BITS_LOW, ctrl, direction))
        epilog = array('B')
        if cs_epilog:
            for ctrl in cs_epilog:
                ctrl &= self._spi_mask
                ctrl |= self._gpio_low
                epilog.extend((Ftdi.SET_BITS_LOW, ctrl, direction))
            # Restore idle state
            cs_high = [Ftdi.SET_BITS_LOW, self._cs_bits | self._gpio_low,
                       direction]
            if not self._turbo:
                cs_high.append(Ftdi.SEND_IMMEDIATE)
            epilog.extend(cs_high)
        writelen = len(out)
        if self._clock_phase != cpha:
            self._ftdi.enable_3phase_clock(cpha)
            self._clock_phase = cpha
        if writelen:
            wcmd = (cpol ^ cpha) and \
                Ftdi.WRITE_BYTES_PVE_MSB or Ftdi.WRITE_BYTES_NVE_MSB
            write_cmd = spack('<BH', wcmd, writelen-1)
            cmd.frombytes(write_cmd)
            cmd.extend(out)
        if readlen:
            rcmd = (cpol ^ cpha) and \
                Ftdi.READ_BYTES_PVE_MSB or Ftdi.READ_BYTES_NVE_MSB
            read_cmd = spack('<BH', rcmd, readlen-1)
            cmd.frombytes(read_cmd)
            cmd.extend(self._immediate)
            if self._turbo:
                if epilog:
                    cmd.extend(epilog)
                self._ftdi.write_data(cmd)
            else:
                self._ftdi.write_data(cmd)
                if epilog:
                    self._ftdi.write_data(epilog)
            # USB read cycle may occur before the FTDI device has actually
            # sent the data, so try to read more than once if no data is
            # actually received
            data = self._ftdi.read_data_bytes(readlen, 4)
        else:
            if writelen:
                if self._turbo:
                    if epilog:
                        cmd.extend(epilog)
                    self._ftdi.write_data(cmd)
                else:
                    self._ftdi.write_data(cmd)
                    if epilog:
                        self._ftdi.write_data(epilog)
            data = array('B')
        return data

    def _exchange_full_duplex(self, frequency, out,
                              cs_prolog, cs_epilog, cpol, cpha):
        if not self._ftdi:
            raise SpiIOError("FTDI controller not initialized")
        if len(out) > SpiController.PAYLOAD_MAX_LENGTH:
            raise SpiIOError("Output payload is too large")
        if cpha:
            # to enable CPHA, we need to use a workaround with FTDI device,
            # that is enable 3-phase clocking (which is usually dedicated to
            # I2C support). This mode use use 3 clock period instead of 2,
            # which implies the FTDI frequency should be fixed to match the
            # requested one.
            frequency = (3*frequency)//2
        if self._frequency != frequency:
            self._ftdi.set_frequency(frequency)
            # store the requested value, not the actual one (best effort),
            # to avoid setting unavailable values on each call.
            self._frequency = frequency
        direction = self.direction & 0xFF  # low bits only
        cmd = array('B')
        for ctrl in cs_prolog or []:
            ctrl &= self._spi_mask
            ctrl |= self._gpio_low
            cmd.extend((Ftdi.SET_BITS_LOW, ctrl, direction))
        epilog = array('B')
        if cs_epilog:
            for ctrl in cs_epilog:
                ctrl &= self._spi_mask
                ctrl |= self._gpio_low
                epilog.extend((Ftdi.SET_BITS_LOW, ctrl, direction))
            # Restore idle state
            cs_high = [Ftdi.SET_BITS_LOW, self._cs_bits | self._gpio_low,
                       direction]
            if not self._turbo:
                cs_high.append(Ftdi.SEND_IMMEDIATE)
            epilog.extend(cs_high)
        writelen = len(out)
        if self._clock_phase != cpha:
            self._ftdi.enable_3phase_clock(cpha)
            self._clock_phase = cpha
        wcmd = (cpol ^ cpha) and \
            Ftdi.RW_BYTES_NVE_PVE_MSB or Ftdi.RW_BYTES_PVE_NVE_MSB
        write_cmd = spack('<BH', wcmd, writelen-1)
        cmd.frombytes(write_cmd)
        cmd.extend(out)
        cmd.extend(self._immediate)
        if self._turbo:
            if epilog:
                cmd.extend(epilog)
            self._ftdi.write_data(cmd)
        else:
            self._ftdi.write_data(cmd)
            if epilog:
                self._ftdi.write_data(epilog)
        # USB read cycle may occur before the FTDI device has actually
        # sent the data, so try to read more than once if no data is
        # actually received
        data = self._ftdi.read_data_bytes(len(out), 4)
        return data

    def _flush(self):
        """Flush the HW FIFOs"""
        self._ftdi.write_data(self._immediate)
        self._ftdi.purge_buffers()
