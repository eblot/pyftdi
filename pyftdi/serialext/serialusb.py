# Copyright (c) 2008-2012, Neotion
# Copyright (c) 2011-2012, Emmanuel Blot <emmanuel.blot@free.fr>
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
from pyftdi.pyftdi.misc import to_int
from serial import SerialBase

__all__ = ['UsbSerial']


class UsbSerial(SerialBase):
    """Base class for Serial port implementation compatible with pyserial API
       using a USB device.
    """

    BAUDRATES = sorted([9600 * (x+1) for x in range(6)] +
                       range(115200, 1000000, 115200) + \
                       range(1000000, 13000000, 100000))

    def makeDeviceName(self, port):
        return port

    def open(self, devclass, scheme, vdict, pdict, default_vendor):
        """Open the initialized serial port"""
        from serial import SerialException
        if self._port is None:
            raise SerialException("Port must be configured before use.")
        portstr = self.portstr
        if not portstr.startswith(scheme):
            raise SerialException("Invalid URL")
        plloc = portstr[len(scheme):].split('/')
        plcomps = plloc[0].split(':') + [''] * 2
        try:
            plcomps[0] = vdict.get(plcomps[0], plcomps[0])
            if plcomps[0]:
                vendor = to_int(plcomps[0])
            else:
                vendor = None
            product_ids = pdict.get(vendor, None)
            if not product_ids:
                product_ids = pdict[default_vendor]
            plcomps[1] = product_ids.get(plcomps[1], plcomps[1])
            if plcomps[1]:
                product = to_int(plcomps[1])
            else:
                product = None
            if not plloc[1]:
                raise SerialException('Invalid device port')
            if plloc[1] == '?':
                show_devices = True
            else:
                interface = to_int(plloc[1])
                show_devices = False
        except (IndexError, ValueError):
            raise SerialException('Invalid device URL')
        sernum = None
        idx = 0
        if plcomps[2]:
            try:
                idx = to_int(plcomps[2])
                if idx > 255:
                    idx = 0
                    raise ValueError
                if idx:
                    idx -= 1
            except ValueError:
                sernum = plcomps[2]
        try:
            self.udev = devclass()
            if not vendor or not product or sernum or idx:
                # Need to enumerate USB devices to find a matching device
                vendors = vendor and [vendor] or \
                    set(vdict.values())
                vps = set()
                for v in vendors:
                    products = pdict.get(v, [])
                    for p in products:
                        vps.add((v, products[p]))
                devices = devclass.find_all(vps)
                candidates = []
                if sernum:
                    if sernum not in [dev[2] for dev in devices]:
                        raise SerialException("No USB device with S/N %s" % \
                                              sernum)
                    for v, p, s, i, d in devices:
                        if s != sernum:
                            continue
                        if vendor and vendor != v:
                            continue
                        if product and product != p:
                            continue
                        candidates.append((v, p, s, i, d))
                else:
                    for v, p, s, i, d in devices:
                        if vendor and vendor != v:
                            continue
                        if product and product != p:
                            continue
                        candidates.append((v, p, s, i, d))
                    if not show_devices:
                        try:
                            vendor, product, ifport, ifcount, description = \
                                candidates[idx]
                        except IndexError:
                            raise SerialException("No USB device #%d" % idx)
            if show_devices:
                self.show_devices(scheme, vdict, pdict, candidates)
                raise SystemExit(candidates and \
                                    'Please specify the USB device' or \
                                    'No USB-Serial device has been detected')
            if vendor not in pdict:
                raise SerialException('Vendor ID 0x%04x not supported' % \
                                      vendor)
            if product not in pdict[vendor].values():
                raise SerialException('Product ID 0x%04x not supported' % \
                                      product)
            self.udev.open(vendor, product, interface, idx, sernum)
        except IOError:
            raise SerialException('Unable to open USB port %s' % self.portstr)
        self._isOpen = True
        self._reconfigurePort()
        self._product = product

    def close(self):
        """Close the open port"""
        self._isOpen = False
        self.udev.close()
        self.udev = None

    def read(self, size=1):
        """Read size bytes from the serial port. If a timeout is set it may
           return less characters as requested. With no timeout it will block
           until the requested number of bytes is read."""
        data = ''
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
        except IOError, e:
            from serial import SerialException
            err = self.udev.get_error_string()
            raise SerialException("%s (%s)" % (str(e), err))

    @staticmethod
    def show_devices(scheme, vdict, pdict, candidates, out=None):
        """Show supported USB-to-serial devices"""
        from string import printable as printablechars
        if not out:
            import sys
            out = sys.stdout
        indices = {}
        interfaces = []
        for (v, p, s, i, d) in candidates:
            ikey = (v, p)
            indices[ikey] = indices.get(ikey, 0) + 1
            # try to find a matching string for the current vendor
            vendors = []
            # fallback if no matching string for the current vendor is found
            vendor = '%04x' % v
            for vc in vdict:
                if vdict[vc] == v:
                    vendors.append(vc)
            if vendors:
                vendors.sort(key=len)
                vendor = vendors[0]
            # try to find a matching string for the current vendor
            # fallback if no matching string for the current product is found
            product = '%04x' % p
            try:
                products = []
                productids = pdict[v]
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
            for j in range(1, i+1):
                # On most configurations, low interfaces are used for MPSSE,
                # high interfaces are dedicated to UARTs
                interfaces.append((scheme, vendor, product, serial, j, d))
        if interfaces:
            print >> out, "Available interfaces:"
            for scheme, vendor, product, serial, j, d in interfaces:
                if d:
                    desc = '  (%s)' % d
                print >> out, '  %s%s:%s:%s/%d%s' % \
                    (scheme, vendor, product, serial, j, desc)
            print >> out, ''
