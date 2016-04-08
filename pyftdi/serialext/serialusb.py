# Copyright (c) 2008-2012, Neotion
# Copyright (c) 2011-2016, Emmanuel Blot <emmanuel.blot@free.fr>
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

import time
from pyftdi.usbtools import UsbTools, UsbToolsError
from serial import SerialBase, VERSION as pyserialver
from six.moves import range

__all__ = ['UsbSerial']


class UsbSerial(SerialBase):
    """Base class for Serial port implementation compatible with pyserial API
       using a USB device.
    """

    BAUDRATES = sorted([9600 * (x+1) for x in range(6)] +
                       list(range(115200, 1000000, 115200)) +
                       list(range(1000000, 13000000, 100000)))

    PYSERIAL_VERSION = tuple([int(x) for x in pyserialver.split('.')])

    def makeDeviceName(self, port):
        return port

    def open(self, devclass, scheme, vdict, pdict, default_vendor):
        """Open the initialized serial port"""
        from serial import SerialException
        if self._port is None:
            raise SerialException("Port must be configured before use.")
        try:
            vendor, product, interface, sernum, ix = UsbTools.parse_url(
                self.portstr, devclass, scheme, vdict, pdict, default_vendor)
        except UsbToolsError as e:
            raise SerialException(str(e))
        try:
            self.udev = devclass()
            self.udev.open(vendor, product, interface, ix, sernum)
            self.flushOutput()
            self.flushInput()
        except IOError:
            raise SerialException('Unable to open USB port %s' % self.portstr)
        self._set_open_state(True)
        self._reconfigurePort()
        self._product = product

    def close(self):
        """Close the open port"""
        self._set_open_state(False)
        self.udev.close()
        self.udev = None

    def read(self, size=1):
        """Read size bytes from the serial port. If a timeout is set it may
           return less characters as requested. With no timeout it will block
           until the requested number of bytes is read."""
        data = bytearray()
        start = time.time()
        while size > 0:
            buf = self.udev.read_data(size)
            data += buf
            size -= len(buf)
            if self._timeout > 0:
                if buf:
                    break
                ms = time.time()-start
                if ms > self._timeout:
                    break
            time.sleep(0.01)
        return data

    def write(self, data):
        """Output the given string over the serial port."""
        self.udev.write_data(data)

    def flush(self):
        """Flush of file like objects. In this case, wait until all data
           is written."""
        # do nothing

    def flushInput(self):
        """Clear input buffer, discarding all that is in the buffer."""
        self.udev.purge_rx_buffer()

    def flushOutput(self):
        """Clear output buffer, aborting the current output and
        discarding all that is in the buffer."""
        self.udev.purge_tx_buffer()

    def sendBreak(self):
        """Send break condition."""
        # Not supported
        pass

    def setRTS(self, level=True):
        """Set terminal status line: Request To Send"""
        self.udev.set_rts(level)

    def setDTR(self, level=True):
        """Set terminal status line: Data Terminal Ready"""
        self.udev.set_dtr(level)

    def getCTS(self):
        """Read terminal status line: Clear To Send"""
        return self.udev.get_cts()

    def getDSR(self):
        """Read terminal status line: Data Set Ready"""
        return self.udev.get_dsr()

    def getRI(self):
        """Read terminal status line: Ring Indicator"""
        return self.udev.get_ri()

    def getCD(self):
        """Read terminal status line: Carrier Detect"""
        return self.udev.get_cd()

    def inWaiting(self):
        """Return the number of characters currently in the input buffer."""
        # not implemented
        return 0

    @property
    def fifoSizes(self):
        """Return the (TX, RX) tupple of hardware FIFO sizes"""
        return self.udev.fifo_sizes

    def _reconfigurePort(self):
        try:
            self.udev.set_baudrate(self._baudrate)
            self.udev.set_line_property(self._bytesize,
                                        self._stopbits,
                                        self._parity)
            if self._rtscts:
                self.udev.set_flowctrl('hw')
            elif self._xonxoff:
                self.udev.set_flowctrl('sw')
            else:
                self.udev.set_flowctrl('')
            try:
                self.udev.set_dynamic_latency(2, 200, 400)
            except AttributeError:
                # backend does not support this feature
                pass
        except IOError as e:
            from serial import SerialException
            err = self.udev.get_error_string()
            raise SerialException("%s (%s)" % (str(e), err))

    _reconfigure_port = _reconfigurePort

    def _set_open_state(self, open_):
        if self.PYSERIAL_VERSION < (3, 0):
            self._isOpen = bool(open_)
        else:
            self.is_open = bool(open_)
