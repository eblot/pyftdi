import errno
import os
import select
import socket
import sys
from util.misc import hexdump

__all__ = ['SerialSocket']

class SerialSocket:
    """Fake serial port redirected to a Unix socket.
       This is basically a copy of the serialposix serial port implementation
       with redefined IO for a Unix socket"""

    def _reconfigurePort(self):
        pass

    def makeDeviceName(self, port):
        return port

    def open(self):
        if self._port is None:
            import serial
            raise serial.SerialException("Port must be configured before use.")
        self._dump = False
        self.sock = None
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(self.portstr)
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
