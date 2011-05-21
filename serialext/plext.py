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
from pyftdi.usbtools import UsbError

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
        # for HWD devices, should be checked for older devices
        # moreover, HWD devices can be reconfigured as (128, 384)
        return (256, 256)

    def open(self):
        super(self.__class__, self).open(Prolific, 
                                         SerialProlific.SCHEME,
                                         SerialProlific.VENDOR_IDS,
                                         SerialProlific.PRODUCT_IDS,
                                         SerialProlific.INTERFACES,
                                         SerialProlific.DEFAULT_VENDOR)

    def inWaiting(self):
        """Return the number of characters currently in the input buffer.
        """
        # not implemented
        return 0
