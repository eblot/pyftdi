# Copyright (c) 2010-2011, Emmanuel Blot <emmanuel.blot@free.fr>
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
import time
from array import array as Array
from pyftdi import Ftdi

__all__ = ['SpiPort', 'SpiController']


class SpiIOError(IOError):
    """SPI I/O error"""


class SpiPort(object):
    """SPI port"""
    def __init__(self, controller, cs_cmd):
        """Instanciate a new SPI port"""
        self._controller = controller
        self._cs_cmd = cs_cmd
        self._frequency = self._controller.frequency

    def exchange(self, out='', readlen=0):
        """Perform a half-duplex transaction with the SPI slave"""
        return self._controller._exchange(self._frequency, self._cs_cmd,
                                          out, readlen)

    def flush(self):
        """Force the flush of the HW FIFOs"""
        self._controller._flush()

    def set_frequency(self, frequency):
        """Change SPI bus frequency"""
        self._frequency = min(frequency, Ftdi.BUS_CLOCK_MAX)

    @property
    def frequency(self):
        return self._frequency


class SpiController(object):
    """SPI master"""

    SCK_BIT = 0x01
    DO_BIT = 0x02
    DI_BIT = 0x04
    CS_BIT = 0x08
    CS_COUNT = 4
    PAYLOAD_MAX_LENGTH = 0x10000 # 16 bits max

    def __init__(self):
        self._ftdi = Ftdi()
        self._cs_bits = ((SpiController.CS_BIT<<SpiController.CS_COUNT)-1) - \
                         (SpiController.CS_BIT-1)
        self._ports = [None] * SpiController.CS_COUNT
        self._direction = self._cs_bits | \
                          SpiController.DO_BIT | \
                          SpiController.SCK_BIT
        self._cs_high = Array('B', [Ftdi.SET_BITS_LOW,
                                    self._cs_bits, self._direction,
                                    # /CS to SCLK delay
                                    Ftdi.WRITE_BITS_TMS_NVE, 7, 0xff])
        self._immediate = Array('B', [Ftdi.SEND_IMMEDIATE])
        self._frequency = 0.0

    def configure(self, vendor, product, interface, frequency=6.0E6):
        """Configure the FTDI interface as a SPI master"""
        self._frequency = self._ftdi.open_mpsse(vendor, product, interface,
                                                self._direction, frequency)

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
            cs_bits = SpiController.SCK_BIT | \
                      SpiController.DO_BIT | \
                      (SpiController.CS_BIT<<cs)
            cs_cmd = struct.pack('<BBB', Ftdi.SET_BITS_LOW,
                                 (~cs_bits)&0xff, self._direction)
            self._ports[cs] = SpiPort(self, cs_cmd)
            self._release_bus()
        return self._ports[cs]

    @property
    def frequency(self):
        return self._frequency

    def _release_bus(self):
        self._ftdi.write_data(self._cs_high)
        self._flush()

    def _exchange(self, frequency, cs_cmd, out, readlen):
        """Perform a half-duplex transaction with the SPI slave"""
        if not self._ftdi:
            raise SpiIOError("FTDI controller not initialized")
        if len(out) > SpiController.PAYLOAD_MAX_LENGTH:
            raise SpiIOError("Output payload is too large")
        if readlen > SpiController.PAYLOAD_MAX_LENGTH:
            raise SpiIOError("Input payload is too large")
        if self._frequency != frequency:
            freq = self._ftdi.set_frequency(frequency)
            # store the requested value, not the actual one (best effort)
            self._frequency = frequency
        write_cmd = struct.pack('<BH', Ftdi.WRITE_BYTES_NVE_MSB, len(out)-1)
        if readlen:
            read_cmd = struct.pack('<BH', Ftdi.READ_BYTES_NVE_MSB, readlen-1)
            cmd = Array('B', cs_cmd)
            cmd.fromstring(write_cmd)
            cmd.extend(out)
            cmd.fromstring(read_cmd)
            cmd.extend(self._immediate)
            cmd.extend(self._cs_high)
            self._ftdi.write_data(cmd)
            # USB read cycle may occur before the FTDI device has actually
            # sent the data, so try to read more than once if no data is
            # actually received
            data = self._ftdi.read_data_bytes(readlen, 4)
        else:
            cmd = Array('B', cs_cmd)
            cmd.fromstring(write_cmd)
            cmd.extend(out)
            cmd.extend(self._cs_high)
            self._ftdi.write_data(cmd)
            data = Array('B')
        return data

    def _flush(self):
        """Flush the HW FIFOs"""
        self._ftdi.write_data(self._immediate)
        self._ftdi.purge_buffers()
