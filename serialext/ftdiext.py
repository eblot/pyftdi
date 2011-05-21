# Copyright (c) 2008-2011, Neotion
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

import re
import time

from pyftdi.ftdi import Ftdi

BACKEND = 'pyftdi'

__all__ = ['SerialFtdi']


class SerialFtdi:
    """Serial port implementation for FTDI compatible with pyserial API"""

    SCHEME = 'ftdi://'
    VENDOR_IDS = { 'ftdi': 0x0403 }
    PRODUCT_IDS = { 0x0403 : \
                      { '232': 0x6001,
                        '2232': 0x6010,
                        '4232': 0x6011,
                        'ft232': 0x6001,
                        'ft2232': 0x6010,
                        'ft4232': 0x6011
                      }
                  }
    INTERFACES = { 0x403 : { 0x6001 : 1, 0x6010 : 2, 0x6011 : 4 } }
    DEFAULT_VENDOR = 0x403

    @property
    def fifoSizes(self):
        """Return the (TX, RX) tupple of hardware FIFO sizes"""
        try:
            # Note that the FTDI datasheets contradict themselves, so
            # the following values may not be the right ones...
            fifo_sizes = { 0x6001: (128,  256),   # TX: 128, RX: 256
                           0x6010: (4096, 4096),  # TX: 4KB, RX: 4KB
                           0x6011: (2048, 2048) } # TX: 2KB, RX: 2KB
            return fifo_sizes[self._product]
        except KeyError:
            return (128, 128) # unknown product

    def open(self):
        super(self.__class__, self).open(Ftdi, 
                                         SerialFtdi.SCHEME,
                                         SerialFtdi.VENDOR_IDS,
                                         SerialFtdi.PRODUCT_IDS,
                                         SerialFtdi.INTERFACES,
                                         SerialFtdi.DEFAULT_VENDOR)

    def _reconfigurePort(self):
        import serial
        BYTESIZES = { 7 : Ftdi.BITS_7,
                      8 : Ftdi.BITS_8 }
        PARITIES  = { 'N' : Ftdi.PARITY_NONE,
                      'O' : Ftdi.PARITY_ODD,
                      'E' : Ftdi.PARITY_EVEN,
                      'M' : Ftdi.PARITY_MARK,
                      'S' : Ftdi.PARITY_SPACE }
        STOPBITS  = { 1 : Ftdi.STOP_BIT_1,
                      1.5 : Ftdi.STOP_BIT_15,
                      2 : Ftdi.STOP_BIT_2 }
        if self._parity not in PARITIES:
            raise serial.SerialException("Unsupported parity")
        if self._bytesize not in BYTESIZES:
            raise serial.SerialException("Unsupported byte size")
        if self._stopbits not in STOPBITS:
            raise serial.SerialException("Unsupported stop bits")
        try:
            self.udev.set_baudrate(self._baudrate)
            self.udev.set_line_property(BYTESIZES[self._bytesize],
                                        STOPBITS[self._stopbits],
                                        PARITIES[self._parity])
            if self._rtscts:
                self.udev.set_flowctrl(Ftdi.SIO_RTS_CTS_HS)
            elif self._xonxoff:
                self.udev.set_flowctrl(Ftdi.SIO_XON_XOFF_HS)
            else:
                self.udev.set_flowctrl(Ftdi.SIO_DISABLE_FLOW_CTRL)
            try:
                self.udev.set_dynamic_latency(2, 200, 400)
            except AttributeError:
                # backend does not support this feature
                pass
        except UsbError, e:
            err = self.udev.get_error_string()
            raise serial.SerialException("%s (%s)" % (str(e), err))

    def inWaiting(self):
        """Return the number of characters currently in the input buffer.
           It seems that Prolific does not offer a way to check if some data
           are already available in the input FIFO, so this method is pretty
           useless. This method only reports if data are ready and the RX FIFO
           buffer is already full. A workaround is to actually read data and
           count the received bytes. In any case, USB polling is required.
        """
        try:
            status = self.udev.poll_modem_status()
        except UsbError, e:
            raise IOError('Prolific communication error: %s' % str(e))
        if (status & Ftdi.MODEM_DR):
            return 1
        return 0

