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
from usbext import SerialUsb

BACKEND = 'pyprolific'

__all__ = ['SerialProlific']


class SerialProlific(SerialUsb):
    """Serial port implementation for Prolific compatible with pyserial API"""

    SCHEME = 'prolific://'
    # the following dictionaries should be augmented to support the various
    # VID/PID that actually map to a USB-serial Prolific device
    VENDOR_IDS = { 'prolific': 0x067b }
    PRODUCT_IDS = { 0x067b : \
                      { '2303': 0x2303,
                        'pl2303': 0x2303
                      }
                  }
    DEFAULT_VENDOR = 0x067b

    def open(self):
        super(self.__class__, self).open(Prolific, 
                                         SerialProlific.SCHEME,
                                         SerialProlific.VENDOR_IDS,
                                         SerialProlific.PRODUCT_IDS,
                                         SerialProlific.DEFAULT_VENDOR)
