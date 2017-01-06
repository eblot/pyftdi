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
        self._frequency = self._controller.frequency

    def exchange(self, out='', readlen=0):
        """Perform an exchange or a transaction with the I2c slave

           .. note:: Exchange is a dual half-duplex transmission: output bytes
                     are sent to the slave, then bytes are received from the
                     slave. It is not possible to perform a full duplex
                     exchange for now, although this feature could be easily
                     implemented.

           :param out: an array of bytes to send to the I2c slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param start: whether to start an I2c transaction, i.e. activate
                         the /CS line for the slave. Use False to resume a
                         previously started transaction
           :param stop: whether to desactivete the /CS line for the slave. Use
                       False if the transaction should complete with a further
                       call to exchange()
           :return: an array of bytes containing the data read out from the
                    slave
        """
        return self._controller._exchange(self._frequency, out, readlen,
                                          self._address)

    def read(self, readlen=0):
        """Read out bytes from the slave"""
        return self._controller._exchange(self._frequency, [], readlen,
                                          self._address)

    def write(self, out):
        """Write bytes to the slave"""
        return self._controller._exchange(self._frequency, out, 0,
                                          self._address)

    def flush(self):
        """Force the flush of the HW FIFOs"""
        self._controller._flush()

    def set_frequency(self, frequency):
        """Change I2c bus frequency"""
        self._frequency = min(frequency, self._controller.frequency_max)

    @property
    def frequency(self):
        """Return the current I2c bus block"""
        return self._frequency


class I2cController(object):
    """I2c master.
    """

    IDLE = 0xff
    SCL_BIT = 0x01
    SDA_O_BIT = 0x02
    SDA_I_BIT = 0x04
    PAYLOAD_MAX_LENGTH = 0x10000  # 16 bits max

    def __init__(self):
        self._ftdi = Ftdi()
        self._frequency = 0.0
        self._direction = I2cController.SCL_BIT | I2cController.SDA_O_BIT
        self._immediate = Array('B', 
            (Ftdi.SEND_IMMEDIATE,))
        idle = (Ftdi.SET_BITS_LOW, self.IDLE, self._direction)
        data_low = (Ftdi.SET_BITS_LOW,
            self.IDLE & ~self.SDA_O_BIT, self._direction)
        clock_data_low = (Ftdi.SET_BITS_LOW,
            self.IDLE & ~(self.SDA_O_BIT|self.SCL_BIT), self._direction)
        self._start = Array('B')
        self._start.extend(data_low*4)
        self._start.extend(clock_data_low*4)
        self._stop = Array('B')
        self._stop.extend(clock_data_low*4)
        self._stop.extend(data_low*4)
        self._stop.extend(idle*4)
        self._idle = Array('B', idle)
        # self._cs_bits = (((I2cController.CS_BIT << cs_count) - 1) &
        #                  ~(I2cController.CS_BIT - 1))
        # self._ports = [None] * cs_count
        # self._direction = (self._cs_bits |
        #                    I2cController.DO_BIT |
        #                    I2cController.SCK_BIT)
        # self._turbo = turbo
        # self._cs_high = Array('B')
        # if self._turbo:
        #     if silent_clock:
        #         # Set SCLK as input to avoid emitting clock beats
        #         self._cs_high.extend((Ftdi.SET_BITS_LOW, self._cs_bits,
        #                               self._direction & ~I2cController.SCK_BIT))
        #     # /CS to SCLK delay, use 8 clock cycles as a HW tempo
        #     self._cs_high.extend((Ftdi.WRITE_BITS_TMS_NVE, 8-1, 0xff))
        # # Restore idle state
        # self._cs_high.extend((Ftdi.SET_BITS_LOW, self._cs_bits,
        #                       self._direction))
        # if not self._turbo:
        #     self._cs_high.append(Ftdi.SEND_IMMEDIATE)
        # self._frequency = 0.0
        self._ftdi.write_data(self._idle)

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
        """Obtain a I2c port to drive a I2c device selected by cs"""
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        if address > 0x7f:
            raise I2cIOError("No such I2c slave")
        # if not self._ports[cs]:
        #     cs_state = 0xFF & ~((I2cController.CS_BIT << cs) |
        #                         I2cController.SCK_BIT |
        #                         I2cController.DO_BIT)
        #     cs_cmd = Array('B', (Ftdi.SET_BITS_LOW,
        #                          cs_state,
        #                          self._direction))
        #     self._ports[cs] = I2cPort(self, cs_cmd)
        #     self._flush()
        return self._ports[address]

    @property
    def frequency_max(self):
        """Returns the maximum I2c clock"""
        return self._ftdi.frequency_max

    @property
    def frequency(self):
        """Returns the current I2c clock"""
        return self._frequency

    def _exchange(self, frequency, out, readlen, address):
        """Perform a half-duplex exchange or transaction with the I2c slave

           :param frequency: I2c bus clock
           :param out: an array of bytes to send to the I2c slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param address: the slave address
           :return: an array of bytes containing the data read out from the
                    slave
        """
        if not self._ftdi:
            raise I2cIOError("FTDI controller not initialized")
        if len(out) > I2cController.PAYLOAD_MAX_LENGTH:
            raise I2cIOError("Output payload is too large")
        if readlen > I2cController.PAYLOAD_MAX_LENGTH:
            raise I2cIOError("Input payload is too large")
        if self._frequency != frequency:
            self._ftdi.set_frequency(frequency)
            # store the requested value, not the actual one (best effort)
            self._frequency = frequency
        cmd = cs_cmd and Array('B', cs_cmd) or Array('B')
        writelen = len(out)
        if writelen:
            write_cmd = struct.pack('<BH', Ftdi.WRITE_BYTES_NVE_MSB,
                                    writelen-1)
            cmd.frombytes(write_cmd)
            cmd.extend(out)
        if readlen:
            read_cmd = struct.pack('<BH', Ftdi.READ_BYTES_NVE_MSB,
                                   readlen-1)
            cmd.frombytes(read_cmd)
            cmd.extend(self._immediate)
            if self._turbo:
                if complete:
                    cmd.extend(self._cs_high)
                self._ftdi.write_data(cmd)
            else:
                self._ftdi.write_data(cmd)
                if complete:
                    self._ftdi.write_data(self._cs_high)
            # USB read cycle may occur before the FTDI device has actually
            # sent the data, so try to read more than once if no data is
            # actually received
            data = self._ftdi.read_data_bytes(readlen, 4)
        elif writelen:
            if self._turbo:
                if complete:
                    cmd.extend(self._cs_high)
                self._ftdi.write_data(cmd)
            else:
                self._ftdi.write_data(cmd)
                if complete:
                    self._ftdi.write_data(self._cs_high)
            data = Array('B')
        return data

    def _flush(self):
        """Flush the HW FIFOs"""
        self._ftdi.write_data(self._immediate)
        self._ftdi.purge_buffers()
