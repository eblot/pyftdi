# Copyright (c) 2017, Emmanuel Blot <emmanuel.blot@free.fr>
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
from binascii import hexlify
from logging import getLogger
from pyftdi.ftdi import Ftdi, FtdiFeatureError
from struct import calcsize as scalc, pack as spack, unpack as sunpack


__all__ = ['I2cPort', 'I2cController']


class I2cIOError(IOError):
    """I2c I/O error"""


class I2cNackError(I2cIOError):
    """I2c NACK receive from slave"""


class I2cTimeoutError(TimeoutError):
    """I2c timeout on polling"""


class I2cPort(object):
    """I2C port.

       An I2C port is never instanciated directly: use I2cController.get_port()
       method to obtain an I2C port.

       ``relax`` parameter may be used to prevent the master from releasing
       the I2C bus, if some further data should be exchanged with the slave
       device. Note that in case of any error, the I2C bus is released and
       the ``relax`` parameter is ignored in such an event.

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

    def read(self, readlen=0, relax=True):
        """Read one or more bytes from a remote slave

           :param int readlen: count of bytes to read out.
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: byte sequence of read out bytes
           :rtype: array
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.read(self._address+self._shift,
                                     readlen=readlen, relax=relax)

    def write(self, out, relax=True):
        """Write one or more bytes to a remote slave

           :param out: the byte buffer to send
           :type out: array or bytes or list(int)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.write(self._address+self._shift, out,
                                      relax=relax)

    def read_from(self, regaddr, readlen=0, relax=True):
        """Read one or more bytes from a remote slave

           :param int regaddr: slave register address to read from
           :param int readlen: count of bytes to read out.
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: data read out from the slave
           :rtype: array
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.exchange(self._address+self._shift,
                                         out=self._make_buffer(regaddr),
                                         readlen=readlen, relax=relax)

    def write_to(self, regaddr, out, relax=True):
        """Read one or more bytes from a remote slave

           :param int regaddr: slave register address to write to
           :param out: the byte buffer to send
           :type out: array or bytes or list(int)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.write(self._address+self._shift,
                                      out=self._make_buffer(regaddr, out),
                                      relax=relax)

    def exchange(self, out=b'', readlen=0, relax=True):
        """Perform an exchange or a transaction with the I2c slave

           :param out: an array of bytes to send to the I2c slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: data read out from the slave
           :rtype: array
        """
        return self._controller.exchange(self._address+self._shift, out,
                                         readlen, relax=relax)

    def poll(self, write=False, relax=True):
        """Poll a remote slave, expect ACK or NACK.

           :param bool write: poll in write mode (vs. read)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: True if the slave acknowledged, False otherwise
           :rtype: bool
        """
        return self._controller.poll(self._address+self._shift, write,
                                     relax=relax)

    def poll_cond(self, width, mask, value, count, relax=True):
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
           :return: the polled register value
           :rtype: array
           :raise I2cTimeoutError: if poll condition is not satisified
        """
        try:
            fmt = ''.join((self._endian, self.FORMATS[width]))
        except KeyError:
            raise I2cIOError('Unsupported integer width')
        return self._controller.poll_cond(self._address+self._shift,
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


class I2cController(object):
    """I2c master.

       An I2c master should be instanciated only once for each FTDI port that
       supports MPSSE (one or two ports, depending on the FTDI device).

       Once configured, :py:func:`get_port` should be invoked to obtain an I2c
       port for each I2c slave to drive. I2cport should handle all I/O requests
       for its associated HW slave.

       It is not recommended to use I2cController :py:func:`read`,
       :py:func:`write` or :py:func:`exchange` directly.
    """

    LOW = 0x00
    HIGH = 0xff
    BIT0 = 0x01
    IDLE = HIGH
    SCL_BIT = 0x01
    SDA_O_BIT = 0x02
    SDA_I_BIT = 0x04
    PAYLOAD_MAX_LENGTH = 0x10000  # 16 bits max
    HIGHEST_I2C_ADDRESS = 0x7f
    DEFAULT_BUS_FREQUENCY = 100000.0
    HIGH_BUS_FREQUENCY = 400000.0
    RETRY_COUNT = 3

    def __init__(self):
        self._ftdi = Ftdi()
        self.log = getLogger('pyftdi.i2c')
        self._slaves = {}
        self._frequency = 0.0
        self._direction = I2cController.SCL_BIT | I2cController.SDA_O_BIT
        self._immediate = (Ftdi.SEND_IMMEDIATE,)
        self._idle = (Ftdi.SET_BITS_LOW, self.IDLE, self._direction)
        data_low = (Ftdi.SET_BITS_LOW,
                    self.IDLE & ~self.SDA_O_BIT, self._direction)
        clock_low_data_low = (Ftdi.SET_BITS_LOW,
                              self.IDLE & ~(self.SDA_O_BIT | self.SCL_BIT),
                              self._direction)
        self._clock_low_data_high = (Ftdi.SET_BITS_LOW,
                                     self.IDLE & ~self.SCL_BIT,
                                     self._direction)
        self._read_bit = (Ftdi.READ_BITS_PVE_MSB, 0)
        self._read_byte = (Ftdi.READ_BYTES_PVE_MSB, 0, 0)
        self._write_byte = (Ftdi.WRITE_BYTES_NVE_MSB, 0, 0)
        self._nack = (Ftdi.WRITE_BITS_NVE_MSB, 0, self.HIGH)
        self._ack = (Ftdi.WRITE_BITS_NVE_MSB, 0, self.LOW)
        self._start = data_low*4 + clock_low_data_low*4
        self._stop = clock_low_data_low*4 + data_low*4 + self._idle*4
        self._tristate = None
        self._tx_size = 1
        self._rx_size = 1

    def configure(self, url, **kwargs):
        """Configure the FTDI interface as a I2c master.

           :param str url: FTDI URL string, such as 'ftdi://ftdi:232h/1'
           :param kwargs: options to configure the I2C bus

           Accepted options:

           * ``frequency`` the I2C bus frequency in Hz
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
        frequency = (3.0*frequency)/2.0
        self._frequency = self._ftdi.open_mpsse_from_url(
            url, direction=self._direction, initial=self.IDLE,
            frequency=frequency, **kwargs)
        self._tx_size, self._rx_size = self._ftdi.fifo_sizes
        self._ftdi.enable_adaptive_clock(False)
        self._ftdi.enable_3phase_clock(True)
        try:
            self._ftdi.enable_drivezero_mode(self.SCL_BIT |
                                             self.SDA_O_BIT |
                                             self.SDA_I_BIT)
        except FtdiFeatureError:
            self._tristate = (Ftdi.SET_BITS_LOW, self.LOW, self.SCL_BIT)

    def terminate(self):
        """Close the FTDI interface.
        """
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

    @classmethod
    def validate_address(cls, address):
        """Assert an I2C slave address is in the supported range

           :param int address: I2C slave address
           :raise I2cIOError: if the I2C slave address is not supported
        """
        if address > cls.HIGHEST_I2C_ADDRESS:
            raise I2cIOError("No such I2c slave")

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

    def read(self, address, readlen=1, relax=True):
        """Read one or more bytes from a remote slave

           :param int address: the address on the I2C bus
           :param int readlen: count of bytes to read out.
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: read bytes
           :rtype: array
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid

           Address is a logical slave address (0x7f max)

           Most I2C devices require a register address to read out
           check out the exchange() method.
        """
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if readlen < 1:
            raise I2cIOError('Nothing to read')
        i2caddress = (address << 1) & self.HIGH
        i2caddress |= self.BIT0
        retries = self.RETRY_COUNT
        while True:
            try:
                self._do_prolog(i2caddress)
                data = self._do_read(readlen)
                return data
            except I2cNackError:
                retries -= 1
                if not retries:
                    raise
                self.log.warning('Retry read')
            finally:
                self._do_epilog()

    def write(self, address, out, relax=True):
        """Write one or more bytes to a remote slave

           :param int address: the address on the I2C bus
           :param out: the byte buffer to send
           :type out: array or bytes or list(int)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :raise I2cIOError: if device is not configured or input parameters
                              are invalid

           Address is a logical slave address (0x7f max)

           Most I2C devices require a register address to write into. It should
           be added as the first (byte)s of the output buffer.
        """
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if not out or len(out) < 1:
            raise I2cIOError('Nothing to write')
        i2caddress = (address << 1) & self.HIGH
        retries = self.RETRY_COUNT
        do_epilog = True
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

           :param int address: the address on the I2C bus
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
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        if not out or not len(out):
            raise I2cIOError('Nothing to write')
        if readlen < 1:
            raise I2cIOError('Nothing to read')
        if readlen > (I2cController.PAYLOAD_MAX_LENGTH/3-1):
            raise I2cIOError("Input payload is too large")
        i2caddress = (address << 1) & self.HIGH
        retries = self.RETRY_COUNT
        do_epilog = True
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

           :param int address: the address on the I2C bus
           :param bool write: poll in write mode (vs. read)
           :param bool relax: whether to relax the bus (emit STOP) or not
           :return: True if the slave acknowledged, False otherwise
           :rtype: bool
        """
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        i2caddress = (address << 1) & self.HIGH
        if not write:
            i2caddress |= self.BIT0
        self.log.debug('- poll 0x%x', i2caddress >> 1)
        do_epilog = True
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

           :param int address: the address on the I2C bus
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
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        self.validate_address(address)
        i2caddress = (address << 1) & self.HIGH
        i2caddress |= self.BIT0
        self.log.debug('- cond poll 0x%x', i2caddress >> 1)
        do_epilog = True
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
        self._ftdi.write_data(self._immediate)
        self._ftdi.purge_buffers()

    def _do_prolog(self, i2caddress):
        self.log.debug('   prolog 0x%x', i2caddress >> 1)
        cmd = array('B', self._idle)
        cmd.extend(self._start)
        cmd.extend(self._write_byte)
        cmd.append(i2caddress)
        if self._tristate:
            cmd.extend(self._tristate)
            cmd.extend(self._read_bit)
            cmd.extend(self._clock_low_data_high)
        else:
            cmd.extend(self._clock_low_data_high)
            cmd.extend(self._read_bit)
        cmd.extend(self._immediate)
        self._ftdi.write_data(cmd)
        ack = self._ftdi.read_data_bytes(1, 4)
        if not ack:
            raise I2cIOError('No answer from FTDI')
        if ack[0] & self.BIT0:
            self.log.warning('NACK')
            raise I2cNackError('NACK from slave')

    def _do_epilog(self):
        self.log.debug('   epilog')
        cmd = array('B', self._stop)
        self._ftdi.write_data(cmd)
        # be sure to purge the MPSSE reply
        self._ftdi.read_data_bytes(1, 1)

    def _do_read(self, readlen,):
        self.log.debug('- read %d byte(s)', readlen)
        if self._tristate:
            read_byte = self._tristate + self._read_byte + \
                        self._clock_low_data_high
            read_not_last = read_byte + self._ack
            read_last = read_byte + self._nack
        else:
            read_not_last = self._read_byte + self._ack + \
                self._clock_low_data_high
            read_last = self._read_byte + self._nack + \
                self._clock_low_data_high
        chunk_size = self._rx_size-2
        cmd_size = len(read_last)
        # limit RX chunk size to the count of I2C packable ommands in the FTDI
        # TX FIFO (minus one byte for the last 'send immediate' command)
        tx_count = (self._tx_size-1) // cmd_size
        chunk_size = min(tx_count, chunk_size)
        chunks = []
        last_count = 0
        last_cmd = None
        count = readlen
        while count:
            block_count = min(count, chunk_size)
            count -= block_count
            if last_count != block_count:
                cmd = array('B')
                cmd.extend(read_not_last * (block_count-1))
                cmd.extend(read_last)
                last_cmd = cmd
            else:
                cmd = last_cmd
            if count <= 0:
                # only force immediate read out on last chunk
                cmd.extend(self._immediate)
            self._ftdi.write_data(cmd)
            buf = self._ftdi.read_data_bytes(block_count, 4)
            chunks.append(buf)
        return array('B', b''.join(chunks))

    def _do_write(self, out):
        if not isinstance(out, array):
            out = array('B', out)
        self.log.debug('- write %d byte(s): %s',
                       len(out), hexlify(out).decode())
        for byte in out:
            cmd = array('B', self._write_byte)
            cmd.append(byte)
            if self._tristate:
                cmd.extend(self._tristate)
                cmd.extend(self._read_bit)
                cmd.extend(self._clock_low_data_high)
            else:
                cmd.extend(self._clock_low_data_high)
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
