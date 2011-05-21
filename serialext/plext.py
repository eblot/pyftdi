# Copyright (c) 2008-2011, Neotion
# Copyright (c) 2011, Emmanuel Blot <emmanuel.blot@free.fr>
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

from pyprolific.prolific import Prolific

BACKEND = 'pyprolific'

__all__ = ['SerialProlific']


class SerialProlific:
    """Serial port implementation for Prolific compatible with pyserial API"""

    SCHEME = 'prolific://'
    VENDOR_IDS = { 'prolific': 0x067b }
    PRODUCT_IDS = { 0x067b : \
                      { '2303': 0x2303,
                        'pl2303': 0x2303
                      }
                  }
    INTERFACES = { 0x067b : { 0x2303 : 1 } }
    DEFAULT_VENDOR = 0x067b

    @property
    def fifoSizes(self):
        """Return the (TX, RX) tupple of hardware FIFO sizes"""
        return (128, 128) # random value for now

    def open(self):
        super(self.__class__, self).open(Prolific, 
                                         SerialProlific.SCHEME,
                                         SerialProlific.VENDOR_IDS,
                                         SerialProlific.PRODUCT_IDS,
                                         SerialProlific.INTERFACES,
                                         SerialProlific.DEFAULT_VENDOR)

    def _reconfigurePort(self):
        import serial
        BYTESIZES = { 7 : Prolific.BITS_7,
                      8 : Prolific.BITS_8 }
        PARITIES  = { 'N' : Prolific.PARITY_NONE,
                      'O' : Prolific.PARITY_ODD,
                      'E' : Prolific.PARITY_EVEN,
                      'M' : Prolific.PARITY_MARK,
                      'S' : Prolific.PARITY_SPACE }
        STOPBITS  = { 1 : Prolific.STOP_BIT_1,
                      1.5 : Prolific.STOP_BIT_15,
                      2 : Prolific.STOP_BIT_2 }
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
                self.udev.set_flowctrl(Prolific.SIO_RTS_CTS_HS)
            elif self._xonxoff:
                self.udev.set_flowctrl(Prolific.SIO_XON_XOFF_HS)
            else:
                self.udev.set_flowctrl(Prolific.SIO_DISABLE_FLOW_CTRL)
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
        if (status & Prolific.MODEM_DR):
            return 1
        return 0

