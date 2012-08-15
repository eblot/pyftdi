# Copyright (c) 2008-2012, Emmanuel Blot <emmanuel.blot@free.fr>
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

import errno
import os
import select
import socket
import sys
from io import RawIOBase
from pyftdi.pyftdi.misc import hexdump
from serial import SerialBase

__all__ = ['Serial']


class SocketSerial(SerialBase):
    """Fake serial port redirected to a Unix socket.

       This is basically a copy of the serialposix serial port implementation
       with redefined IO for a Unix socket"""

    def _reconfigurePort(self):
        pass

    def makeDeviceName(self, port):
        return port

    def open(self):
        """Open the initialized serial port"""
        if self._port is None:
            import serial
            raise serial.SerialException("Port must be configured before use.")
        self._dump = False
        self.sock = None
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            filename = self.portstr[self.portstr.index('://')+3:]
            self.sock.connect(filename)
        except Exception, msg:
            self.sock = None
            import serial
            raise serial.SerialException("Could not open port: %s" % msg)
        self._isOpen = True

    def close(self):
        if self.sock:
            self.sock.close()

    def inWaiting(self):
        """Return the number of characters currently in the input buffer."""
        return 0

    def read(self, size=1):
        """Read size bytes from the serial port. If a timeout is set it may
           return less characters as requested. With no timeout it will block
           until the requested number of bytes is read."""
        if self.sock is None:
            import serial
            raise serial.portNotOpenError
        read = ''
        inp = None
        if size > 0:
            while len(read) < size:
                ready,_,_ = select.select([self.sock],[],[], self._timeout)
                if not ready:
                    break   #timeout
                buf = self.sock.recv(size-len(read))
                read = read + buf
                if self._timeout >= 0 and not buf:
                    break  #early abort on timeout
        return read

    def write(self, data):
        """Output the given string over the serial port."""
        if self.sock is None:
            raise serial.portNotOpenError
        t = len(data)
        d = data
        while t > 0:
            try:
                if self._writeTimeout is not None and self._writeTimeout > 0:
                    _,ready,_ = select.select([],[self.sock],[],
                                                  self._writeTimeout)
                    if not ready:
                        raise serial.writeTimeoutError
                n = self.sock.send(d)
                if self._dump:
                    print hexdump(d[:n])
                if self._writeTimeout is not None and self._writeTimeout > 0:
                    _,ready,_ = select.select([],[self.sock],[],
                                              self._writeTimeout)
                    if not ready:
                        raise serial.writeTimeoutError
                d = d[n:]
                t = t - n
            except OSError,v:
                if v.errno != errno.EAGAIN:
                    raise

    def flush(self):
        """Flush of file like objects. In this case, wait until all data
           is written."""
        pass

    def flushInput(self):
        """Clear input buffer, discarding all that is in the buffer."""
        pass

    def flushOutput(self):
        """Clear output buffer, aborting the current output and
        discarding all that is in the buffer."""
        pass

    def sendBreak(self):
        """Send break condition."""
        pass

    def setRTS(self,on=1):
        """Set terminal status line: Request To Send"""
        pass

    def setDTR(self,on=1):
        """Set terminal status line: Data Terminal Ready"""
        pass

    def getCTS(self):
        """Read terminal status line: Clear To Send"""
        return True

    def getDSR(self):
        """Read terminal status line: Data Set Ready"""
        return True

    def getRI(self):
        """Read terminal status line: Ring Indicator"""
        return False

    def getCD(self):
        """Read terminal status line: Carrier Detect"""
        return False

    # - - platform specific - - - -

    def nonblocking(self):
        """internal - not portable!"""
        if self.sock is None:
            import serial
            raise serial.portNotOpenError
        self.sock.setblocking(0)

    def dump(self, enable):
        self._dump = enable


# assemble Serial class with the platform specifc implementation and the base
# for file-like behavior.
class Serial(SocketSerial, RawIOBase):
    pass
