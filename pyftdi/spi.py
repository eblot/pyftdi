# Copyright (c) 2010-2017, Emmanuel Blot <emmanuel.blot@free.fr>
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

import struct
from array import array as Array
from pyftdi.ftdi import Ftdi


__all__ = ['SpiPort', 'SpiController']


class SpiIOError(IOError):
    """SPI I/O error"""


class SpiPort(object):
    """SPI port

       An SPI port is never instanciated directly.

       Use SpiController.get_port() method to obtain an SPI port

       :Example:

            ctrl = SpiController(silent_clock=False)
            ctrl.configure('ftdi://ftdi:232h/1')
            spi = ctrl.get_port(1)
            spi.set_frequency(1000000)
            # send 2 bytes
            spi.exchange([0x12, 0x34])
            # send 2 bytes, then receive 2 bytes
            out = spi.exchange([0x12, 0x34], 2)
            # send 2 bytes, then receive 4 bytes, manage the transaction
            out = spi.exchange([0x12, 0x34], 2, True, False)
            out.extend(spi.exchange([], 2, False, True))
    """

    def __init__(self, controller, cs_cmd):
        self._controller = controller
        self._cs_cmd = cs_cmd
        self._frequency = self._controller.frequency

    def exchange(self, out='', readlen=0, start=True, stop=True):
        """Perform an exchange or a transaction with the SPI slave

           .. note:: Exchange is a dual half-duplex transmission: output bytes
                     are sent to the slave, then bytes are received from the
                     slave. It is not possible to perform a full duplex
                     exchange for now, although this feature could be easily
                     implemented.

           :param out: an array of bytes to send to the SPI slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param start: whether to start an SPI transaction, i.e. activate
                         the /CS line for the slave. Use False to resume a
                         previously started transaction
           :param stop: whether to desactivete the /CS line for the slave. Use
                       False if the transaction should complete with a further
                       call to exchange()
           :return: an array of bytes containing the data read out from the
                    slave
        """
        return self._controller._exchange(self._frequency, out, readlen,
                                          start and self._cs_cmd, stop)

    def read(self, readlen=0, start=True, stop=True):
        """Read out bytes from the slave"""
        return self._controller._exchange(self._frequency, [], readlen,
                                          start and self._cs_cmd, stop)

    def write(self, out, start=True, stop=True):
        """Write bytes to the slave"""
        return self._controller._exchange(self._frequency, out, 0,
                                          start and self._cs_cmd, stop)

    def flush(self):
        """Force the flush of the HW FIFOs"""
        self._controller._flush()

    def set_frequency(self, frequency):
        """Change SPI bus frequency"""
        self._frequency = min(frequency, self._controller.frequency_max)

    @property
    def frequency(self):
        """Return the current SPI bus block"""
        return self._frequency


class SpiController(object):
    """SPI master.

        :param silent_clock: should be set to avoid clocking out SCLK when all
                             /CS signals are released. This clock beat is used
                             to enforce a delay between /CS signal activation.

                             When weird SPI devices are used, SCLK beating may
                             cause trouble. In this case, silent_clock should
                             be set but beware that SCLK line should be fitted
                             with a pull-down resistor, as SCLK is high-Z
                             during this short period of time.
        :param cs_count: is the number of /CS lines (one per device to drive on
                         the SPI bus)
    """

    SCK_BIT = 0x01
    DO_BIT = 0x02
    DI_BIT = 0x04
    CS_BIT = 0x08
    PAYLOAD_MAX_LENGTH = 0x10000  # 16 bits max

    def __init__(self, silent_clock=False, cs_count=4, turbo=True):
        self._ftdi = Ftdi()
        self._cs_bits = (((SpiController.CS_BIT << cs_count) - 1) &
                         ~(SpiController.CS_BIT - 1))
        self._ports = [None] * cs_count
        self._direction = (self._cs_bits |
                           SpiController.DO_BIT |
                           SpiController.SCK_BIT)
        self._turbo = turbo
        self._cs_high = Array('B')
        if self._turbo:
            if silent_clock:
                # Set SCLK as input to avoid emitting clock beats
                self._cs_high.extend((Ftdi.SET_BITS_LOW, self._cs_bits,
                                      self._direction & ~SpiController.SCK_BIT))
            # /CS to SCLK delay, use 8 clock cycles as a HW tempo
            self._cs_high.extend((Ftdi.WRITE_BITS_TMS_NVE, 8-1, 0xff))
        # Restore idle state
        self._cs_high.extend((Ftdi.SET_BITS_LOW, self._cs_bits,
                              self._direction))
        if not self._turbo:
            self._cs_high.append(Ftdi.SEND_IMMEDIATE)
        self._immediate = Array('B', (Ftdi.SEND_IMMEDIATE,))
        self._frequency = 0.0

    def configure(self, url, **kwargs):
        """Configure the FTDI interface as a SPI master"""
        for k in ('direction', 'initial'):
            if k in kwargs:
                del kwargs[k]
        self._frequency = \
            self._ftdi.open_mpsse_from_url(
                # /CS all high
                url, direction=self._direction, initial=self._cs_bits,
                **kwargs)

    def terminate(self):
        """Close the FTDI interface"""
        if self._ftdi:
            self._ftdi.close()
            self._ftdi = None

    def get_port(self, cs):
        """Obtain a SPI port to drive a SPI device selected by cs"""
        if not self._ftdi:
            raise SpiIOError("FTDI controller not initialized")
        if cs >= len(self._ports):
            raise SpiIOError("No such SPI port")
        if not self._ports[cs]:
            cs_state = 0xFF & ~((SpiController.CS_BIT << cs) |
                                SpiController.SCK_BIT |
                                SpiController.DO_BIT)
            cs_cmd = Array('B', (Ftdi.SET_BITS_LOW,
                                 cs_state,
                                 self._direction))
            self._ports[cs] = SpiPort(self, cs_cmd)
            self._flush()
        return self._ports[cs]

    @property
    def frequency_max(self):
        """Returns the maximum SPI clock"""
        return self._ftdi.frequency_max

    @property
    def frequency(self):
        """Returns the current SPI clock"""
        return self._frequency

    def _exchange(self, frequency, out, readlen, cs_cmd=None, complete=True):
        """Perform a half-duplex exchange or transaction with the SPI slave

           :param frequency: SPI bus clock
           :param out: an array of bytes to send to the SPI slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param cs_cmd: the prolog sequence to activate the /CS line on the
                       SPI bus. May be empty to resume a previously started
                       transaction
           :param complete: whether to send the epilog sequence to move the
                       /CS line back to the idle state. May be force to False
                       if another part of a transaction is expected
           :return: an array of bytes containing the data read out from the
                    slave
        """
        if not self._ftdi:
            raise SpiIOError("FTDI controller not initialized")
        if len(out) > SpiController.PAYLOAD_MAX_LENGTH:
            raise SpiIOError("Output payload is too large")
        if readlen > SpiController.PAYLOAD_MAX_LENGTH:
            raise SpiIOError("Input payload is too large")
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
