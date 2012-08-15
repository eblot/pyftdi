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
from pyftdi.pyftdi.ftdi import Ftdi, FtdiError
from pyftdi.pyftdi.misc import to_int
from usbext import SerialUsb

BACKEND = 'pyftdi'

__all__ = ['SerialFtdi']


class SerialFtdi(SerialUsb):
    """Serial port implementation for FTDI compatible with pyserial API"""

    SCHEME = 'ftdi://'
    # the following dictionaries should be augmented to support the various
    # VID/PID that actually map to a USB-serial FTDI device
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
    DEFAULT_VENDOR = 0x403

    def open(self):
        super(SerialFtdi, self).open(Ftdi,
                                     SerialFtdi.SCHEME,
                                     SerialFtdi.VENDOR_IDS,
                                     SerialFtdi.PRODUCT_IDS,
                                     SerialFtdi.DEFAULT_VENDOR)
