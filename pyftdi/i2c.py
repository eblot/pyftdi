# Copyright (c) 2017-2019, Emmanuel Blot <emmanuel.blot@free.fr>
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

"""I2C support for PyFdti"""

from array import array
from binascii import hexlify
from collections import namedtuple
from logging import getLogger
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from threading import Lock
from pyftdi.ftdi import Ftdi, FtdiFeatureError

# pylint: disable-msg=too-many-locals,too-many-instance-attributes
# pylint: disable-msg=too-many-arguments
# pylint: disable-msg=consider-using-ternary


class I2cIOError(IOError):
    """I2c I/O error"""


class I2cNackError(I2cIOError):
    """I2c NACK receive from slave"""


class I2cTimeoutError(TimeoutError):
    """I2c timeout on polling"""


class I2cPort:
    """I2C port.

       An I2C port is never instanciated directly: use I2cController.get_port()
       method to obtain an I2C port.

       ``relax`` parameter in I2cPort methods may be used to prevent the master
       from releasing the I2C bus, if some further data should be exchanged
       with the slave device. Note that in case of any error, the I2C bus is
       released and the ``relax`` parameter is ignored in such an event.

       Example:

       >>> ctrl = I2cController()
       >>> ctrl.configure('ftdi://ftdi:232h/1')
       >>> i2c = ctrl.get_port(0x21)
       >>> # send 2 bytes
       >>> i2c.write([0x12, 0x34])
       >>> # send 2 bytes, then receive 2 bytes
       >>> out = i2c.exchange([0x12, 0x34], 2)
    """
    FORMATS = {scalc(fmt): fmt for fmt in 'BHI'}

    def __init__(self, controller, address):
        self._controller = controller
        self._address = address
        self._shift = 0
        self._endian = '<'
        self._format = 'B'

    def configure_register(self, bigendian=False, width=1):
        """Reconfigure the format of the slave address register (if any)

            :param bool bigendian: True for a big endian encoding,
                                   False otherwise
            :param int width: width, in bytes, of the register
        """
        try:
            self._format = self.FORMATS[width]
        except KeyError:
            raise I2cIOError('Unsupported integer width')
        self._endian = bigendian and '>' or '<'

    def shift_address(self, offset):
        """Tweak the I2C slave address, as required with some devices
        """
        I2cController.validate_address(self._address+offset)
        self._shift = offset

    def read(self, readlen=0, relax=True, start=True):
        """Read one or more bytes from a remote slave

           :param int readlen: count of bytes to read out.
           :param bool relax: whether to relax the bus (emit STOP) or not
           :param bool start: whether to emit a start sequence (w/ address)
           :return: byte sequence of read out bytes
           :rtype: array
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.read(
            self._address+self._shift if start else None,
            readlen=readlen, relax=relax)

    def write(self, out, relax=True, start=True):
        """Write one or more bytes to a remote slave

           :param out: the byte buffer to send
           :type out: array or bytes or list(int)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :param bool start: whether to emit a start sequence (w/ address)
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.write(
            self._address+self._shift if start else None,
            out, relax=relax)

    def read_from(self, regaddr, readlen=0, relax=True, start=True):
        """Read one or more bytes from a remote slave

           :param int regaddr: slave register address to read from
           :param int readlen: count of bytes to read out.
           :param bool relax: whether to relax the bus (emit STOP) or not
           :param bool start: whether to emit a start sequence (w/ address)
           :return: data read out from the slave
           :rtype: array
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.exchange(
            self._address+self._shift if start else None,
            out=self._make_buffer(regaddr), readlen=readlen, relax=relax)

    def write_to(self, regaddr, out, relax=True, start=True):
        """Read one or more bytes from a remote slave

           :param int regaddr: slave register address to write to
           :param out: the byte buffer to send
           :type out: array or bytes or list(int)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :param bool start: whether to emit a start sequence (w/ address)
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.write(
            self._address+self._shift if start else None,
            out=self._make_buffer(regaddr, out), relax=relax)

    def exchange(self, out=b'', readlen=0, relax=True, start=True):
        """Perform an exchange or a transaction with the I2c slave

           :param out: an array of bytes to send to the I2c slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param bool relax: whether to relax the bus (emit STOP) or not
           :param bool start: whether to emit a start sequence (w/ address)
           :return: data read out from the slave
           :rtype: array
        """
        return self._controller.exchange(
            self._address+self._shift if start else None, out,
            readlen, relax=relax)

    def poll(self, write=False, relax=True, start=True):
        """Poll a remote slave, expect ACK or NACK.

           :param bool write: poll in write mode (vs. read)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :param bool start: whether to emit a start sequence (w/ address)
           :return: True if the slave acknowledged, False otherwise
           :rtype: bool
        """
        return self._controller.poll(
            self._address+self._shift if start else None, write,
            relax=relax)

    def poll_cond(self, width, mask, value, count, relax=True, start=True):
        """Poll a remove slave, watching for condition to satisfy.
           On each poll cycle, a repeated start condition is emitted, without
           releasing the I2C bus, and an ACK is returned to the slave.

           If relax is set, this method releases the I2C bus however it leaves.

           :param int width: count of bytes to poll for the condition check,
                that is the size of the condition register
           :param int mask: binary mask to apply on the condition register
                before testing for the value
           :param int value: value to test the masked condition register
                against. Condition is satisfied when register & mask == value
           :param int count: maximum poll count before raising a timeout
           :param bool relax: whether to relax the bus (emit STOP) or not
           :param bool start: whether to emit a start sequence (w/ address)
           :return: the polled register value
           :rtype: array
           :raise I2cTimeoutError: if poll condition is not satisified
        """
        try:
            fmt = ''.join((self._endian, self.FORMATS[width]))
        except KeyError:
            raise I2cIOError('Unsupported integer width')
        return self._controller.poll_cond(
            self._address+self._shift if start else None,
            fmt, mask, value, count, relax=relax)

    def flush(self):
        """Force the flush of the HW FIFOs.
        """
        self._controller.flush()

    @property
    def frequency(self):
        """Provide the current I2c bus frequency.
        """
        return self._controller.frequency

    def _make_buffer(self, regaddr, out=None):
        data = array('B')
        data.extend(spack('%s%s' % (self._endian, self._format), regaddr))
        if out:
            data.extend(out)
        return data.tobytes()


class I2cGpioPort:
    """GPIO port

       A I2cGpioPort instance enables to drive GPIOs wich are not reserved for
       I2c feature as regular GPIOs.

       GPIO are managed as a bitfield. The LSBs are reserved for the I2c
       feature, which means that the lowest pin that can be used as a GPIO is
       b4:

       * b0: I2C SCL
       * b1: I2C SDA_O
       * b2: I2C SDA_I
       * b3: first GPIO
       * b7: reserved for I2C clock stretching if this mode is enabled

       There is no offset bias in GPIO bit position, *i.e.* the first available
       GPIO can be reached from as ``0x08``.

       Bitfield size depends on the FTDI device: 4432H series use 8-bit GPIO
       ports, while 232H and 2232H series use wide 16-bit ports.

       An I2cGpio port is never instanciated directly: use
       I2cController.get_gpio() method to obtain the GPIO port.
    """
    def __init__(self, controller):
        self.log = getLogger('pyftdi.i2c.gpio')
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

           Note that all pins, including reserved I2C ones, are reported.

           :return: the count of IO pins (including I2C ones).
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


I2CTimings = namedtuple('I2CTimings', 't_hd_sta t_su_sta t_su_sto t_buf')
"""I2C standard timings.
"""


class I2cController:
    """I2c master.

       An I2c master should be instanciated only once for each FTDI port that
       supports MPSSE (one or two ports, depending on the FTDI device).

       Once configured, :py:func:`get_port` should be invoked to obtain an I2c
       port for each I2c slave to drive. I2cport should handle all I/O requests
       for its associated HW slave.

       It is not recommended to use I2cController :py:func:`read`,
       :py:func:`write` or :py:func:`exchange` directly.

       * ``SCK`` should be connected to ``A*BUS0``, and ``A*BUS7`` if clock
           stretching mode is enavbled
       * ``SDA`` should be connected to ``A*BUS1`` **and** ``A*BUS2``
    """

    LOW = 0x00
    HIGH = 0xff
    BIT0 = 0x01
    IDLE = HIGH
    SCL_BIT = 0x01  #AD0
    SDA_O_BIT = 0x02  #AD1
    SDA_I_BIT = 0x04  #AD2
    SCL_FB_BIT = 0x80  #AD7
    PAYLOAD_MAX_LENGTH = 0x10000  # 16 bits max
    HIGHEST_I2C_ADDRESS = 0x78
    DEFAULT_BUS_FREQUENCY = 100000.0
    HIGH_BUS_FREQUENCY = 400000.0
    RETRY_COUNT = 3

    I2C_MASK = SCL_BIT | SDA_O_BIT | SDA_I_BIT
    I2C_MASK_CS = SCL_BIT | SDA_O_BIT | SDA_I_BIT | SCL_FB_BIT
    I2C_DIR = SCL_BIT | SDA_O_BIT

    I2C_100K = I2CTimings(4.0E-6, 4.7E-6, 4.0E-6, 4.7E-6)
    I2C_400K = I2CTimings(0.6E-6, 0.6E-6, 0.6E-6, 1.3E-6)
    I2C_1M = I2CTimings(0.26E-6, 0.26E-6, 0.26E-6, 0.5E-6)

    def __init__(self):
        self._ftdi = Ftdi()
        self._lock = Lock()
        self.log = getLogger('pyftdi.i2c')
        self._gpio_port = None
        self._gpio_dir = 0
        self._gpio_low = 0
        self._gpio_mask = 0
        self._i2c_mask = 0
        self._wide_port = False
        self._slaves = {}
        self._retry_count = self.RETRY_COUNT
        self._frequency = 0.0
        self._immediate = (Ftdi.SEND_IMMEDIATE,)
        self._read_bit = (Ftdi.READ_BITS_PVE_MSB, 0)
        self._read_byte = (Ftdi.READ_BYTES_PVE_MSB, 0, 0)
        self._write_byte = (Ftdi.WRITE_BYTES_NVE_MSB, 0, 0)
        self._nack = (Ftdi.WRITE_BITS_NVE_MSB, 0, self.HIGH)
        self._ack = (Ftdi.WRITE_BITS_NVE_MSB, 0, self.LOW)
        self._ck_delay = 1
        self._tristate = None
        self._tx_size = 1
        self._rx_size = 1
        self._ck_hd_sta = 0
        self._ck_su_sto = 0
        self._ck_idle = 0

    def set_retry_count(self, count):
        """Change the default retry count when a communication error occurs,
           before bailing out.
           :param int count: count of retries
        """
        if not isinstance(count, int) or not 0 < count <= 16:
            raise ValueError('Invalid retry count')
        self._retry_count = count

    def configure(self, url, **kwargs):
        """Configure the FTDI interface as a I2c master.

           :param str url: FTDI URL string, such as 'ftdi://ftdi:232h/1'
           :param kwargs: options to configure the I2C bus

           Accepted options:

           * ``frequency`` float value the I2C bus frequency in Hz
           * ``clockstretching`` boolean value to enable clockstreching.
             xD7 (GPIO7) pin should be connected back to xD0 (SCK)
        """
        for k in ('direction', 'initial'):
            if k in kwargs:
                del kwargs[k]
        if 'frequency' in kwargs:
            frequency = kwargs['frequency']
            del kwargs['frequency']
        else:
            frequency = self.DEFAULT_BUS_FREQUENCY
        # Fix frequency for 3-phase clock
        if frequency <= 100E3:
            timings = self.I2C_100K
        elif frequency <= 400E3:
            timings = self.I2C_100K
        else:
            timings = self.I2C_100K
        if 'clockstretching' in kwargs:
            clkstrch = bool(kwargs['clockstretching'])
            del kwargs['clockstretching']
        else:
            clkstrch = False
        self._ck_hd_sta = self._compute_delay_cycles(timings.t_hd_sta)
        self._ck_su_sto = self._compute_delay_cycles(timings.t_su_sto)
        ck_su_sta = self._compute_delay_cycles(timings.t_su_sta)
        ck_buf = self._compute_delay_cycles(timings.t_buf)
        self._ck_idle = max(ck_su_sta, ck_buf)
        self._ck_delay = ck_buf
        # as 3-phase clock frequency mode is required for I2C mode, the
        # FTDI clock should be adapted to match the required frequency.
        frequency = self._ftdi.open_mpsse_from_url(
            url, direction=self.I2C_DIR, initial=self.IDLE,
            frequency=(3.0*frequency)/2.0, **kwargs)
        self._frequency = (2.0*frequency)/3.0
        self._tx_size, self._rx_size = self._ftdi.fifo_sizes
        self._ftdi.enable_adaptive_clock(clkstrch)
        self._ftdi.enable_3phase_clock(True)
        try:
            self._ftdi.enable_drivezero_mode(self.SCL_BIT |
                                             self.SDA_O_BIT |
                                             self.SDA_I_BIT)
        except FtdiFeatureError:
            self._tristate = (Ftdi.SET_BITS_LOW, self.LOW, self.SCL_BIT)
        self._wide_port = self._ftdi.has_wide_port
        if clkstrch:
            self._i2c_mask = self.I2C_MASK_CS
        else:
            self._i2c_mask = self.I2C_MASK


    def terminate(self):
        """Close the FTDI interface.
        """
        with self._lock:
            if self._ftdi:
                self._ftdi.close()
                self._ftdi = None

    def get_port(self, address):
        """Obtain an I2cPort to drive an I2c slave.

           :param int address: the address on the I2C bus
           :return: an I2cPort instance
           :rtype: :py:class:`I2cPort`
        """
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address not in self._slaves:
            self._slaves[address] = I2cPort(self, address)
        return self._slaves[address]

    def get_gpio(self):
        """Retrieve the GPIO port.

           :return GPIO port
        """
        with self._lock:
            if not self._ftdi:
                raise I2cIOError("FTDI controller not initialized")
            if not self._gpio_port:
                self._gpio_port = I2cGpioPort(self)
            return self._gpio_port

    @property
    def configured(self):
        """Test whether the device has been properly configured.

           :return: True if configured
        """
        return bool(self._ftdi) and bool(self._start)

    @classmethod
    def validate_address(cls, address):
        """Assert an I2C slave address is in the supported range.
           None is a special bypass address.

           :param address: the address on the I2C bus
           :type address: int or None
           :raise I2cIOError: if the I2C slave address is not supported
        """
        if address is None:
            return
        if address > cls.HIGHEST_I2C_ADDRESS:
            raise I2cIOError("No such I2c slave: 0x%02x" % address)

    @property
    def frequency_max(self):
        """Provides the maximum I2c clock frequency.
        """
        return self._ftdi.frequency_max

    @property
    def frequency(self):
        """Provides the current I2c clock frequency.

           :return: the I2C bus clock
           :rtype: float
        """
        return self._frequency

    @property
    def direction(self):
        """Provide the FTDI GPIO direction"""
        return self.I2C_DIR | self._gpio_dir

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
            return mask & ~self._i2c_mask

    @property
    def width(self):
        """Report the FTDI count of addressable pins.

           :return: the count of IO pins (including I2C ones).
        """
        return 16 if self._wide_port else 8

    def read(self, address, readlen=1, relax=True):
        """Read one or more bytes from a remote slave

           :param address: the address on the I2C bus, or None to discard start
           :type address: int or None
           :param int readlen: count of bytes to read out.
           :param bool relax: not used
           :return: read bytes
           :rtype: array
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid

           Address is a logical slave address (0x7f max)

           Most I2C devices require a register address to read out
           check out the exchange() method.
        """
        if not self.configured:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            i2caddress = None
        else:
            i2caddress = (address << 1) & self.HIGH
            i2caddress |= self.BIT0
        retries = self._retry_count
        do_epilog = True
        with self._lock:
            while True:
                try:
                    self._do_prolog(i2caddress)
                    data = self._do_read(readlen)
                    do_epilog = relax
                    return data
                except I2cNackError:
                    retries -= 1
                    if not retries:
                        raise
                    self.log.warning('Retry read')
                finally:
                    if do_epilog:
                        self._do_epilog()

    def write(self, address, out, relax=True):
        """Write one or more bytes to a remote slave

           :param address: the address on the I2C bus, or None to discard start
           :type address: int or None
           :param out: the byte buffer to send
           :type out: array or bytes or list(int)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid

           Address is a logical slave address (0x7f max)

           Most I2C devices require a register address to write into. It should
           be added as the first (byte)s of the output buffer.
        """
        if not self.configured:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            i2caddress = None
        else:
            i2caddress = (address << 1) & self.HIGH
        retries = self._retry_count
        do_epilog = True
        with self._lock:
            while True:
                try:
                    self._do_prolog(i2caddress)
                    self._do_write(out)
                    do_epilog = relax
                    return
                except I2cNackError:
                    retries -= 1
                    if not retries:
                        raise
                    self.log.warning('Retry write')
                finally:
                    if do_epilog:
                        self._do_epilog()

    def exchange(self, address, out, readlen=0, relax=True):
        """Send a byte sequence to a remote slave followed with
           a read request of one or more bytes.

           This command is useful to tell the slave what data
           should be read out.

           :param address: the address on the I2C bus, or None to discard start
           :type address: int or None
           :param out: the byte buffer to send
           :type out: array or bytes or list(int)
           :param int readlen: count of bytes to read out.
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: read bytes
           :rtype: array
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid

           Address is a logical slave address (0x7f max)
        """
        if not self.configured:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if readlen < 1:
            raise I2cIOError('Nothing to read')
        if readlen > (self.PAYLOAD_MAX_LENGTH/3-1):
            raise I2cIOError("Input payload is too large")
        if address is None:
            i2caddress = None
        else:
            i2caddress = (address << 1) & self.HIGH
        retries = self._retry_count
        do_epilog = True
        with self._lock:
            while True:
                try:
                    self._do_prolog(i2caddress)
                    self._do_write(out)
                    self._do_prolog(i2caddress | self.BIT0)
                    if readlen:
                        data = self._do_read(readlen)
                    do_epilog = relax
                    return data
                except I2cNackError:
                    retries -= 1
                    if not retries:
                        raise
                    self.log.warning('Retry exchange')
                finally:
                    if do_epilog:
                        self._do_epilog()

    def poll(self, address, write=False, relax=True):
        """Poll a remote slave, expect ACK or NACK.

           :param address: the address on the I2C bus, or None to discard start
           :type address: int or None
           :param bool write: poll in write mode (vs. read)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: True if the slave acknowledged, False otherwise
           :rtype: bool
        """
        if not self.configured:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            i2caddress = None
        else:
            i2caddress = (address << 1) & self.HIGH
            if not write:
                i2caddress |= self.BIT0
        do_epilog = True
        with self._lock:
            try:
                self._do_prolog(i2caddress)
                do_epilog = relax
                return True
            except I2cNackError:
                self.log.info('Not ready')
                return False
            finally:
                if do_epilog:
                    self._do_epilog()

    def poll_cond(self, address, fmt, mask, value, count, relax=True):
        """Poll a remove slave, watching for condition to satisfy.
           On each poll cycle, a repeated start condition is emitted, without
           releasing the I2C bus, and an ACK is returned to the slave.

           If relax is set, this method releases the I2C bus however it leaves.

           :param address: the address on the I2C bus, or None to discard start
           :type address: int or None
           :param str fmt: struct format for poll register
           :param int mask: binary mask to apply on the condition register
                before testing for the value
           :param int value: value to test the masked condition register
                against. Condition is satisfied when register & mask == value
           :param int count: maximum poll count before raising a timeout
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: the polled register value, or None if poll failed
           :rtype: array or None
        """
        if not self.configured:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            i2caddress = None
        else:
            i2caddress = (address << 1) & self.HIGH
            i2caddress |= self.BIT0
        do_epilog = True
        with self._lock:
            try:
                retry = 0
                while retry < count:
                    retry += 1
                    size = scalc(fmt)
                    self._do_prolog(i2caddress)
                    data = self._do_read(size)
                    self.log.debug("Poll data: %s", hexlify(data).decode())
                    cond, = sunpack(fmt, data)
                    if (cond & mask) == value:
                        self.log.debug('Poll condition matched')
                        break
                    else:
                        data = None
                        self.log.debug('Poll condition not fulfilled: %x/%x',
                                       cond & mask, value)
                do_epilog = relax
                if not data:
                    self.log.warning('Poll condition failed')
                return data
            except I2cNackError:
                self.log.info('Not ready')
                return None
            finally:
                if do_epilog:
                    self._do_epilog()

    def flush(self):
        """Flush the HW FIFOs
        """
        if not self.configured:
            raise I2cIOError("FTDI controller not initialized")
        with self._lock:
            self._ftdi.write_data(self._immediate)
            self._ftdi.purge_buffers()

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
                raise I2cIOError('No such GPO pins: %04x/%04x' %
                                 (self._gpio_dir, value))
            # perform read-modify-write
            use_high = self._wide_port and (self.direction & 0xff00)
            data = self._read_raw(use_high)
            data &= ~self._gpio_mask
            data |= value
            self._write_raw(data, use_high)
            self._gpio_low = data & 0xFF & ~self._i2c_mask

    def set_gpio_direction(self, pins, direction):
        """Change the direction of the GPIO pins

           :param int pins: which GPIO pins should be reconfigured
           :param int direction: direction bitfield (on for output)
        """
        with self._lock:
            if pins & self._i2c_mask:
                raise I2cIOError('Cannot access I2C pins as GPIO')
            gpio_width = self._wide_port and 16 or 8
            gpio_mask = (1 << gpio_width) - 1
            gpio_mask &= ~self._i2c_mask
            if (pins & gpio_mask) != pins:
                raise I2cIOError('No such GPIO pin(s)')
            self._gpio_dir &= ~pins
            self._gpio_dir |= (pins & direction)
            self._gpio_mask = gpio_mask & pins

    @property
    def _data_lo(self):
        return (Ftdi.SET_BITS_LOW,
                self.SCL_BIT | self._gpio_low,
                self.I2C_DIR | (self._gpio_dir & 0xFF))

    @property
    def _clk_lo_data_hi(self):
        return (Ftdi.SET_BITS_LOW,
                self.SDA_O_BIT | self._gpio_low,
                self.I2C_DIR | (self._gpio_dir & 0xFF))

    @property
    def _clk_lo_data_lo(self):
        return (Ftdi.SET_BITS_LOW,
                self._gpio_low,
                self.I2C_DIR | (self._gpio_dir & 0xFF))

    @property
    def _idle(self):
        return (Ftdi.SET_BITS_LOW,
                self.I2C_DIR | self._gpio_low,
                self.I2C_DIR | (self._gpio_dir & 0xFF))

    @property
    def _start(self):
        return self._data_lo * self._ck_hd_sta + \
               self._clk_lo_data_lo * self._ck_hd_sta

    @property
    def _stop(self):
        return self._clk_lo_data_hi * self._ck_hd_sta + \
               self._data_lo * self._ck_su_sto + \
               self._idle * self._ck_idle

    def _compute_delay_cycles(self, value):
        # approx ceiling without relying on math module
        # the bit delay is far from being precisely known anyway
        bit_delay = self._ftdi.mpsse_bit_delay
        return max(1, int((value + bit_delay) / bit_delay))

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
            raise I2cIOError('Cannot read GPIO')
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

    def _do_prolog(self, i2caddress):
        if i2caddress is None:
            return
        self.log.debug('   prolog 0x%x', i2caddress >> 1)
        cmd = array('B', self._idle)
        cmd.extend(self._start)
        cmd.extend(self._write_byte)
        cmd.append(i2caddress)
        if self._tristate:
            cmd.extend(self._tristate)
            cmd.extend(self._read_bit)
            cmd.extend(self._clk_lo_data_hi)
        else:
            cmd.extend(self._clk_lo_data_hi)
            cmd.extend(self._read_bit)
        cmd.extend(self._immediate)
        self._ftdi.write_data(cmd)
        ack = self._ftdi.read_data_bytes(1, 4)
        if not ack:
            raise I2cIOError('No answer from FTDI')
        if ack[0] & self.BIT0:
            self.log.warning('NACK @ 0x%02x', (i2caddress>>1))
            raise I2cNackError('NACK from slave')

    def _do_epilog(self):
        self.log.debug('   epilog')
        cmd = array('B', self._stop)
        self._ftdi.write_data(cmd)
        # be sure to purge the MPSSE reply
        self._ftdi.read_data_bytes(1, 1)

    def _do_read(self, readlen):
        self.log.debug('- read %d byte(s)', readlen)
        if not readlen:
            # force a real read request on device, but discard any result
            cmd = array('B')
            cmd.extend(self._immediate)
            self._ftdi.write_data(cmd)
            self._ftdi.read_data_bytes(0, 4)
            return array('B')
        if self._tristate:
            read_byte = self._tristate + \
                        self._read_byte + \
                        self._clk_lo_data_hi
            read_not_last = \
                read_byte + self._ack + self._clk_lo_data_lo * self._ck_delay
            read_last = \
                read_byte + self._nack + self._clk_lo_data_hi * self._ck_delay
        else:
            read_not_last = \
                self._read_byte + self._ack + \
                self._clk_lo_data_hi * self._ck_delay
            read_last = \
                self._read_byte + self._nack + \
                self._clk_lo_data_hi * self._ck_delay
        # maximum RX size to fit in FTDI FIFO, minus 2 status bytes
        chunk_size = self._rx_size-2
        cmd_size = len(read_last)
        # limit RX chunk size to the count of I2C packable commands in the FTDI
        # TX FIFO (minus one byte for the last 'send immediate' command)
        tx_count = (self._tx_size-1) // cmd_size
        chunk_size = min(tx_count, chunk_size)
        chunks = []
        cmd = None
        rem = readlen
        while rem:
            if rem > chunk_size:
                if not cmd:
                    cmd = array('B')
                    cmd.extend(read_not_last * chunk_size)
                    size = chunk_size
            else:
                cmd = array('B')
                cmd.extend(read_not_last * (rem-1))
                cmd.extend(read_last)
                cmd.extend(self._immediate)
                size = rem
            self._ftdi.write_data(cmd)
            buf = self._ftdi.read_data_bytes(size, 4)
            self.log.debug('- read %d byte(s): %s',
                           len(buf), hexlify(buf).decode())
            chunks.append(buf)
            rem -= size
        return array('B', b''.join(chunks))

    def _do_write(self, out):
        if not isinstance(out, array):
            out = array('B', out)
        if not out:
            return
        self.log.debug('- write %d byte(s): %s',
                       len(out), hexlify(out).decode())
        for byte in out:
            cmd = array('B', self._write_byte)
            cmd.append(byte)
            if self._tristate:
                cmd.extend(self._tristate)
                cmd.extend(self._read_bit)
                cmd.extend(self._clk_lo_data_hi)
            else:
                cmd.extend(self._clk_lo_data_hi)
                cmd.extend(self._read_bit)
            cmd.extend(self._immediate)
            self._ftdi.write_data(cmd)
            ack = self._ftdi.read_data_bytes(1, 4)
            if not ack:
                msg = 'No answer from FTDI'
                self.log.critical(msg)
                raise I2cIOError(msg)
            if ack[0] & self.BIT0:
                msg = 'NACK from slave'
                self.log.warning(msg)
                raise I2cNackError(msg)
