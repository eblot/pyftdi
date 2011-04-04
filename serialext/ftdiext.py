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
from pyftdi.ftdi import Ftdi, FtdiError
from pyftdi.misc import to_int

BACKEND = 'pyftdi'

__all__ = ['SerialFtdi']


class SerialFtdi:
    """Serial port implementation for FTDI compatible with pyserial API"""

    BAUDRATES = sorted([9600 * (x+1) for x in range(6)] +
                       range(115200, 1000000, 115200) + \
                       range(1000000, 13000000, 100000))
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
            self.ftdi.set_baudrate(self._baudrate)
            self.ftdi.set_line_property(BYTESIZES[self._bytesize],
                                        STOPBITS[self._stopbits],
                                        PARITIES[self._parity])
            if self._rtscts:
                self.ftdi.set_flowctrl(Ftdi.SIO_RTS_CTS_HS)
            elif self._xonxoff:
                self.ftdi.set_flowctrl(Ftdi.SIO_XON_XOFF_HS)
            else:
                self.ftdi.set_flowctrl(Ftdi.SIO_DISABLE_FLOW_CTRL)
            try:
                self.ftdi.set_dynamic_latency(2, 200, 400)
            except AttributeError:
                # FTDI backend does not support this feature
                pass
        except FtdiError, e:
            err = self.ftdi.get_error_string()
            raise serial.SerialException("%s (%s)" % str(e), err)

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

    def makeDeviceName(self, port):
        return port

    def open(self):
        import serial
        if self._port is None:
            raise serial.SerialException("Port must be configured before use.")
        portstr = self.portstr
        if not portstr.startswith(self.SCHEME):
            raise serial.SerialException("Invalid FTDI URL")
        ftdiloc = portstr[len(self.SCHEME):].split('/')
        ftdicomps = ftdiloc[0].split(':') + [''] * 2
        try:
            ftdicomps[0] = self.VENDOR_IDS.get(ftdicomps[0], ftdicomps[0])
            if ftdicomps[0]:
                vendor = to_int(ftdicomps[0])
            else:
                vendor = None
            product_ids = self.PRODUCT_IDS.get(vendor, None)
            if not product_ids:
                product_ids = self.PRODUCT_IDS[self.DEFAULT_VENDOR]
            ftdicomps[1] = product_ids.get(ftdicomps[1], ftdicomps[1])
            if ftdicomps[1]:
                product = to_int(ftdicomps[1])
            else:
                product = None
            if not ftdiloc[1]:
                raise serial.SerialException('Invalid FTDI device port')
            if ftdiloc[1] == '?':
                show_devices = True
            else:
                interface = to_int(ftdiloc[1])
                show_devices = False
        except (IndexError, ValueError):
            raise serial.SerialException('Invalid FTDI device URL')
        sernum = None
        idx = 0
        if ftdicomps[2]:
            try:
                idx = to_int(ftdicomps[2])
                if idx > 255:
                    idx = 0
                    raise ValueError
                if idx:
                    idx -= 1
            except ValueError:
                sernum = ftdicomps[2]
        try:
            self.ftdi = Ftdi()
            if not vendor or not product or sernum or idx:
                # Need to enumerate USB devices to find a matching device
                vendors = vendor and [vendor] or \
                    set(self.VENDOR_IDS.values())
                vps = set()
                for v in vendors:
                    products = self.PRODUCT_IDS.get(v, [])
                    for p in products:
                        vps.add((v, products[p]))
                devices = self.ftdi.find_all(vps)
                candidates = []
                if sernum:
                    if sernum not in [dev[2] for dev in devices]:
                        raise serial.SerialException("No FTDI device with " \
                                                     "S/N %s" % sernum)
                    for v, p, s in devices:
                        if s != sernum:
                            continue
                        if vendor and vendor != v:
                            continue
                        if product and product != p:
                            continue
                        candidates.append((v, p, s))
                else:
                    for v, p, s in devices:
                        if vendor and vendor != v:
                            continue
                        if product and product != p:
                            continue
                        candidates.append((v, p, s))
                    if not show_devices:
                        try:
                            vendor, product, _ = candidates[idx]
                        except IndexError:
                            raise serial.SerialException("No FTDI device #%d" \
                                                         % idx)
            if show_devices:
                self._show_devices(candidates)
                raise SystemExit('Please specify the FTDI device')
            self.ftdi.open(vendor, product, interface, idx, sernum)
        except FtdiError:
            raise IOError('Unable to open FTDI port %s' % self.portstr)
        self._isOpen = True
        self._reconfigurePort()
        self._product = product

    def close(self):
        self._isOpen = False
        self.ftdi.close()
        self.ftdi = None

    def inWaiting(self):
        """Return the number of characters currently in the input buffer."""
        try:
            status = self.ftdi.poll_modem_status()
        except FtdiError, e:
            raise IOError('FTDI communication error: %s' % str(e))
        if (status & Ftdi.MODEM_DR):
            return 1
        return 0

    def read(self, size=1):
        """Read size bytes from the serial port. If a timeout is set it may
           return less characters as requested. With no timeout it will block
           until the requested number of bytes is read."""
        data = ''
        start = time.time()
        while size > 0:
            buf = self.ftdi.read_data(size)
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
        self.ftdi.write_data(data)

    def flush(self):
        """Flush of file like objects. In this case, wait until all data
           is written."""
         # do nothing

    def flushInput(self):
        """Clear input buffer, discarding all that is in the buffer."""
        self.ftdi.usb_purge_rx_buffer()

    def flushOutput(self):
        """Clear output buffer, aborting the current output and
        discarding all that is in the buffer."""
        self.ftdi.usb_purge_tx_buffer()

    def sendBreak(self):
        """Send break condition."""
        # Not supported
        pass

    def setRTS(self,on=1):
        """Set terminal status line: Request To Send"""
        self.ftdi.set_rts(on)

    def setDTR(self,on=1):
        """Set terminal status line: Data Terminal Ready"""
        self.ftdi.set_dtr(on)

    def getCTS(self):
        """Read terminal status line: Clear To Send"""
        status = self.ftdi.poll_modem_status()
        return (status & Ftdi.MODEM_CTS) and True or False

    def getDSR(self):
        """Read terminal status line: Data Set Ready"""
        status = self.ftdi.poll_modem_status()
        return (status & Ftdi.MODEM_DSR) and True or False

    def getRI(self):
        """Read terminal status line: Ring Indicator"""
        status = self.ftdi.poll_modem_status()
        return (status & Ftdi.MODEM_RI) and True or False

    def getCD(self):
        """Read terminal status line: Carrier Detect"""
        status = self.ftdi.poll_modem_status()
        return (status & Ftdi.MODEM_RLSD) and True or False

    def _show_devices(self, candidates, out=None):
        from string import printable as printablechars
        if not out:
            import sys
            out = sys.stdout
        print >> out, "Available interfaces:"
        indices = {}
        for (v, p, s) in candidates:
            try:
                ifcount = self.INTERFACES[v][p]
            except KeyError, e:
                continue
            ikey = (v, p)
            indices[ikey] = indices.get(ikey, 0) + 1
            # try to find a matching string for the current vendor
            vendors = []
            # fallback if no matching string for the current vendor is found
            vendor = '%04x' % v
            for vc in self.VENDOR_IDS:
                if self.VENDOR_IDS[vc] == v:
                    vendors.append(vc)
            if vendors:
                vendors.sort(key=len)
                vendor = vendors[0]
            # try to find a matching string for the current vendor
            # fallback if no matching string for the current product is found
            product = '%04x' % p
            try:
                products = []
                productids = self.PRODUCT_IDS[v]
                for pc in productids:
                    if productids[pc] == p:
                        products.append(pc)
                if products:
                    products.sort(key=len)
                    product = products[0]
            except KeyError:
                pass
            # if the serial number is an ASCII char, use it, or use the index
            # value
            if [c for c in s if c not in printablechars or c == '?']:
                serial = '%d' % indices[ikey]
            else:
                serial = s
            # Now print out the prettiest URL syntax
            for i in range(1, ifcount+1):
                # On most configurations, low interfaces are used for MPSSE,
                # high interfaces are dedicated to UARTs
                print >> out, '  %s%s:%s:%s/%d' % \
                    (self.SCHEME, vendor, product, serial, i)
        print >> out, ''
