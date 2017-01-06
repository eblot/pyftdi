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

import struct
from array import array as Array
from pyftdi.ftdi import Ftdi


__all__ = ['I2cPort', 'I2cController']


class I2cIOError(IOError):
    """I2c I/O error"""


class I2cPort(object):
    """I2C port

       An I2C port is never instanciated directly.

       Use I2cController.get_port() method to obtain an I2C port

       :Example:

            ctrl = I2cController()
            ctrl.configure('ftdi://ftdi:232h/1')
            i2c = ctrl.get_port(1)
            i2c.set_frequency(1000000)
            # send 2 bytes
            i2c.exchange([0x12, 0x34])
            # send 2 bytes, then receive 2 bytes
            out = i2c.exchange([0x12, 0x34], 2)
    """

    def __init__(self, controller, address):
        self._controller = controller
        self._address = address

    def exchange(self, out='', readlen=0):
        """Perform an exchange or a transaction with the I2c slave

           :param out: an array of bytes to send to the I2c slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :return: an array of bytes containing the data read out from the
                    slave
        """
        return self._controller._exchange(self._address, out, readlen)

    def read(self, readlen=0):
        """Read out bytes from the slave"""
        return self._controller._exchange(self._address, readlen=readlen)

    def write(self, out):
        """Write bytes to the slave"""
        return self._controller._exchange(self._address, out)

    def flush(self):
        """Force the flush of the HW FIFOs"""
        self._controller._flush()

    @property
    def frequency(self):
        """Return the current I2c bus"""
        return self._controller.frequency


class I2cController(object):
    """I2c master.
    """

    LOW = 0x00
    HIGH = 0xff
    IDLE = HIGH
    SCL_BIT = 0x01
    SDA_O_BIT = 0x02
    SDA_I_BIT = 0x04
    PAYLOAD_MAX_LENGTH = 0x10000  # 16 bits max

    def __init__(self):
        self._ftdi = Ftdi()
        self._slaves = {}
        self._frequency = 0.0
        self._direction = I2cController.SCL_BIT | I2cController.SDA_O_BIT
        self._immediate = (Ftdi.SEND_IMMEDIATE,)
        idle = (Ftdi.SET_BITS_LOW, self.IDLE, self._direction)
        data_low = (Ftdi.SET_BITS_LOW,
            self.IDLE & ~self.SDA_O_BIT, self._direction)
        clock_low_data_low = (Ftdi.SET_BITS_LOW,
            self.IDLE & ~(self.SDA_O_BIT|self.SCL_BIT), self._direction)
        self._clock_low_data_high = (Ftdi.SET_BITS_LOW,
            self.IDLE & ~self.SCL_BIT, self._direction)
        self._read_bit = (Ftdi.READ_BITS_PVE_MSB, 0)
        self._read_byte = (Ftdi.READ_BYTES_PVE_MSB, 0, 0)
        self._write_byte = (Ftdi.WRITE_BYTES_NVE_MSB, 0, 0)
        self._nack = (Ftdi.WRITE_BITS_NVE_MSB, 0, self.HIGH)
        self._ack = (Ftdi.WRITE_BITS_NVE_MSB, 0, self.LOW)
        self._start = data_low*4 + clock_low_data_low*4
        self._stop = clock_low_data_low*4 + data_low*4 + idle*4
        self._ftdi.write_data(Array('B', idle))

    def configure(self, url, **kwargs):
        """Configure the FTDI interface as a I2c master"""
        for k in ('direction', 'initial'):
            if k in kwargs:
                del kwargs[k]
        if 'frequency' in kwargs:
            frequency = kwargs['frequency']
            del kwargs['frequency']
        else:
            frequency = 100000.0
        # Fix frequency for 3-phase clock
        frequency = (3.0*frequency)/2.0
        self._frequency = \
            self._ftdi.open_mpsse_from_url(
                # /CS all high
                url, direction=self._direction, initial=self._cs_bits,
                frequency=frequency, **kwargs)

    def terminate(self):
        """Close the FTDI interface"""
        if self._ftdi:
            self._ftdi.close()
            self._ftdi = None

    def get_port(self, address):
        """Obtain a I2cPort to to drive a I2c slave"""
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        if address > 0x7f:
            raise I2cIOError("No such I2c slave")
        if address not in self._slaves:
            self._slaves[address] = I2cPort(self, cs_cmd)
        return self._slaves[address]

    @property
    def frequency_max(self):
        """Returns the maximum I2c clock"""
        return self._ftdi.frequency_max

    @property
    def frequency(self):
        """Returns the current I2c clock"""
        return self._frequency

    def _read_bytes(self, readlen=1):
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        if readlen > (I2cController.PAYLOAD_MAX_LENGTH/3-1):
            raise I2cIOError("Input payload is too large")
        cmd = Array('B')
        count = readlen
        while count:
            count -= 1
            cmd.extend(self._read_byte)
            cmd.extend(count and self._ack or self._nack)
            cmd.extend(self._clock_low_data_high)
        cmd.extend(self._immediate)
        self._ftdi.write_data(cmd)
        data = self._ftdi.read_data_bytes(readlen, 4)
        return data

    def _write_bytes(self, out):
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        cmd = Array('B')
        for byte in out:
            cmd.extend(self._write_byte)
            cmd.append(byte)
            cmd.extend(self._clock_low_data_high) 
            cmd.extend(self._read_bit)
            cmd.extend(self._immediate)
            self._ftdi.write_data(cmd)
            ack = self._ftdi.read_data_bytes(1, 4)
            if ack[0] & 0x01:
                raise FtdiError('NACK from slave')

    def _send_address(self, address, write_req=True):
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        address <<= 1
        address &= 0xfe
        address |= int(not bool(write))
        self._write_bytes([address])

    def _exchange(self, address, out=None, readlen=0):
        if out:
            self._set_idle()
            self._start()
            self._send_address(address, True)
            self._write_bytes(out)
        if readlen:
            self._set_idle()
            self._start()
            data = self._read_bytes(readlen)
        else:
            data = Array('B')
        self._stop()
        return data

    def _flush(self):
        """Flush the HW FIFOs"""
        self._ftdi.write_data(self._immediate)
        self._ftdi.purge_buffers()
