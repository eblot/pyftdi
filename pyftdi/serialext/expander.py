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

import os
import sys

__all__ = ['SerialExpander', 'SerialExpanderError']


class SerialExpanderError(Exception):
    """Raised when the expander is in trouble"""
    pass


class SerialExpander(object):
    """Provides extension classes compatible with PySerial module"
       Create a class on request, to prevent useless dependency on
       non-standard PySerial module"""

    @staticmethod
    def serialclass(device, use_logger=False):
        """Provide a pyserial class that supports high-speed serial port on
        Apple computer (usually through a USB-RS232 adaptor key)"""
        try:
            import serial
        except ImportError:
            raise SerialExpanderError("Python serial module not installed")
        # default serial class
        type_ = serial.Serial
        type_.backend = 'serial'
        if device.startswith('ftdi://'):
            # for now, assume the USB device is a FTDI device
            # a USB dispatcher should be implemented here
            from ftdiext import SerialFtdi, BACKEND
            type_ = SerialFtdi
            type_.backend = BACKEND
        elif device.startswith('prolific://'):
            # for now, assume the USB device is a Prolific device
            # a USB dispatcher should be implemented here
            from plext import SerialProlific, BACKEND
            type_ = SerialProlific
            type_.backend = BACKEND
        elif os.path.exists(device):
            import stat
            from socketext import SerialSocket
            if stat.S_ISSOCK(os.stat(device)[0]):
                type_ = SerialSocket
                type_.backend = 'socket'
        if type_.backend == 'serial' and sys.platform.lower() in ('darwin'):
            # hack for Mac OS X hosts: the underlying termios system library
            # cannot cope with baudrates > 230kbps and pyserial << 9.7
            version = os.uname()[2].split('.')
            if 'version' not in serial.__dict__ or \
              serial.version[0] < 9 or serial.version[1] < 7:
                # Tiger or above can support arbitrary serial speeds
                if int(version[0]) >= 8:
                    # remove all speeds not supported with TERMIOS
                    # so that pyserial never attempts to use them directly
                    for b in serial.baudrate_constants.keys():
                        if b > 230400:
                            del serial.baudrate_constants[b]
                    from darwinext import SerialDarwin
                    class SerialDarwinAdapter(SerialDarwin, type_):
                        backend = type_.backend
                    type_ = SerialDarwinAdapter
        if use_logger:
            from loggerext import SerialLoggerPort
            class SerialLoggerAdapter(SerialLoggerPort, type_):
                backend = type_.backend
            type_ = SerialLoggerAdapter
        return type_
