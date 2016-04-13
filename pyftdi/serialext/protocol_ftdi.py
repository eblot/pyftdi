# Copyright (c) 2008-2016, Neotion
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

from io import RawIOBase

from pyftdi.ftdi import Ftdi, FtdiError
from pyftdi.serialext.serialusb import UsbSerial

__all__ = ['Serial']


class FtdiSerial(UsbSerial):
    """Serial port implementation for FTDI compatible with pyserial API"""

    BACKEND = 'pyftdi'
    SCHEME = 'ftdi'
    FTDI_VENDOR = 0x403
    VENDOR_IDS = {'ftdi': FTDI_VENDOR}
    PRODUCT_IDS = {
        FTDI_VENDOR:
            {'232': 0x6001,
             '232r': 0x6001,
             '232h': 0x6014,
             '2232': 0x6010,
             '4232': 0x6011,
             '230x': 0x6015,
             'ft232': 0x6001,
             'ft232r': 0x6001,
             'ft232h': 0x6014,
             'ft2232': 0x6010,
             'ft4232': 0x6011,
             'ft230x': 0x6015
             }
        }
    DEFAULT_VENDOR = FTDI_VENDOR

    def open(self):
        """Open the initialized serial port"""
        from serial.serialutil import SerialException
        try:
            UsbSerial.open(self, Ftdi,
                           FtdiSerial.SCHEME,
                           FtdiSerial.VENDOR_IDS,
                           FtdiSerial.PRODUCT_IDS,
                           FtdiSerial.DEFAULT_VENDOR)
        except FtdiError as e:
            raise SerialException(str(e))

    @classmethod
    def add_custom_vendor(cls, vid, vidname=''):
        """Add a custom USB vendor identifier.
           It can be useful to use a pretty URL for opening FTDI device
        """
        if vid in cls.VENDOR_IDS:
            raise ValueError('Vendor ID 0x%04x already registered' % vid)
        if not vidname:
            vidname = '0x%04x' % vid
        cls.VENDOR_IDS[vidname] = vid

    @classmethod
    def add_custom_product(cls, vid, pid, pidname=''):
        """Add a custom USB product identifier.
           It is required for opening FTDI device with non-standard VID/PID
           USB identifiers.
        """
        if vid not in cls.PRODUCT_IDS:
            cls.PRODUCT_IDS[vid] = {}
        elif pid in cls.PRODUCT_IDS[vid]:
            raise ValueError('Product ID 0x%04x already registered' % vid)
        if not pidname:
            pidname = '0x%04x' % pid
        cls.PRODUCT_IDS[vid][pidname] = pid


# assemble Serial class with the platform specific implementation and the base
# for file-like behavior.
class Serial(FtdiSerial, RawIOBase):
    pass
