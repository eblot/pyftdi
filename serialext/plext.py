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

from pyprolific.prolific import Prolific, ProlificError
from pyftdi.misc import to_int
from pyftdi.usbtools import UsbTools

BACKEND = 'pyprolific'

__all__ = ['SerialProlific']


class SerialProlific:
    """Serial port implementation for Prolific compatible with pyserial API"""

    BAUDRATES = sorted([9600 * (x+1) for x in range(6)] +
                       range(115200, 1000000, 115200) + \
                       range(1000000, 13000000, 100000))
    SCHEME = 'prolific://'
    VENDOR_IDS = { 'prolific': 0x067b }
    PRODUCT_IDS = { 0x067b : \
                      { '2303': 0x2303,
                        'pl2303': 0x2303
                      }
                  }
    INTERFACES = { 0x067b : { 0x2303 : 1 } }
    DEFAULT_VENDOR = 0x067b

    def _reconfigurePort(self):
        import serial
        BYTESIZES = { 7 : Prolific.BITS_7,
                      8 : Prolific.BITS_8 }
        PARITIES  = { 'N' : Prolific.PARITY_NONE,
                      'O' : Prolific.PARITY_ODD,
                      'E' : Prolific.PARITY_EVEN,
                      'M' : Prolific.PARITY_MARK,
                      'S' : Prolific.PARITY_SPACE }
        STOPBITS  = { 1 : Prolific.STOP_BIT_1,
                      1.5 : Prolific.STOP_BIT_15,
                      2 : Prolific.STOP_BIT_2 }
        if self._parity not in PARITIES:
            raise serial.SerialException("Unsupported parity")
        if self._bytesize not in BYTESIZES:
            raise serial.SerialException("Unsupported byte size")
        if self._stopbits not in STOPBITS:
            raise serial.SerialException("Unsupported stop bits")
        try:
            self.pl.set_baudrate(self._baudrate)
            self.pl.set_line_property(BYTESIZES[self._bytesize],
                                      STOPBITS[self._stopbits],
                                      PARITIES[self._parity])
            if self._rtscts:
                self.pl.set_flowctrl(Prolific.SIO_RTS_CTS_HS)
            elif self._xonxoff:
                self.pl.set_flowctrl(Prolific.SIO_XON_XOFF_HS)
            else:
                self.pl.set_flowctrl(Prolific.SIO_DISABLE_FLOW_CTRL)
            try:
                self.pl.set_dynamic_latency(2, 200, 400)
            except AttributeError:
                # Prolific backend does not support this feature
                pass
        except ProlificError, e:
            err = self.pl.get_error_string()
            raise serial.SerialException("%s (%s)" % (str(e), err))

    @property
    def fifoSizes(self):
        """Return the (TX, RX) tupple of hardware FIFO sizes"""
        try:
            # Note that the Prolific datasheets contradict themselves, so
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
            raise serial.SerialException("Invalid Prolific URL")
        plloc = portstr[len(self.SCHEME):].split('/')
        plcomps = plloc[0].split(':') + [''] * 2
        try:
            plcomps[0] = self.VENDOR_IDS.get(plcomps[0], plcomps[0])
            if plcomps[0]:
                vendor = to_int(plcomps[0])
            else:
                vendor = None
            product_ids = self.PRODUCT_IDS.get(vendor, None)
            if not product_ids:
                product_ids = self.PRODUCT_IDS[self.DEFAULT_VENDOR]
            plcomps[1] = product_ids.get(plcomps[1], plcomps[1])
            if plcomps[1]:
                product = to_int(plcomps[1])
            else:
                product = None
            if not plloc[1]:
                raise serial.SerialException('Invalid Prolific device port')
            if plloc[1] == '?':
                show_devices = True
            else:
                interface = to_int(plloc[1])
                show_devices = False
        except (IndexError, ValueError):
            raise serial.SerialException('Invalid Prolific device URL')
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
            self.pl = Prolific()
            if not vendor or not product or sernum or idx:
                # Need to enumerate USB devices to find a matching device
                vendors = vendor and [vendor] or \
                    set(self.VENDOR_IDS.values())
                vps = set()
                for v in vendors:
                    products = self.PRODUCT_IDS.get(v, [])
                    for p in products:
                        vps.add((v, products[p]))
                devices = UsbTools.find_all(vps)
                candidates = []
                if sernum:
                    if sernum not in [dev[2] for dev in devices]:
                        raise serial.SerialException("No Prolific device " \
                                                     "with S/N %s" % sernum)
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
                            raise serial.SerialException("No Prolific device" \
                                                         " #%d" % idx)
            if show_devices:
                UsbTools.show_devices(self.SCHEME, self.VENDOR_IDS,
                                      self.PRODUCT_IDS, self.INTERFACES, 
                                      candidates)
                raise SystemExit('Please specify the Prolific device')
            self.pl.open(vendor, product, interface, idx, sernum)
        except ProlificError:
            raise IOError('Unable to open Prolific port %s' % self.portstr)
        self._isOpen = True
        self._reconfigurePort()
        self._product = product

    def close(self):
        self._isOpen = False
        self.pl.close()
        self.pl = None

    def inWaiting(self):
        """Return the number of characters currently in the input buffer.
           It seems that Prolific does not offer a way to check if some data
           are already available in the input FIFO, so this method is pretty
           useless. This method only reports if data are ready and the RX FIFO
           buffer is already full. A workaround is to actually read data and
           count the received bytes. In any case, USB polling is required.
        """
        try:
            status = self.pl.poll_modem_status()
        except ProlificError, e:
            raise IOError('Prolific communication error: %s' % str(e))
        if (status & Prolific.MODEM_DR):
            return 1
        return 0

    def read(self, size=1):
        """Read size bytes from the serial port. If a timeout is set it may
           return less characters as requested. With no timeout it will block
           until the requested number of bytes is read."""
        data = ''
        start = time.time()
        while size > 0:
            buf = self.pl.read_data(size)
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
        self.pl.write_data(data)

    def flush(self):
        """Flush of file like objects. In this case, wait until all data
           is written."""
         # do nothing

    def flushInput(self):
        """Clear input buffer, discarding all that is in the buffer."""
        self.pl.purge_rx_buffer()

    def flushOutput(self):
        """Clear output buffer, aborting the current output and
        discarding all that is in the buffer."""
        self.pl.purge_tx_buffer()

    def sendBreak(self):
        """Send break condition."""
        # Not supported
        pass

    def setRTS(self,on=1):
        """Set terminal status line: Request To Send"""
        self.pl.set_rts(on)

    def setDTR(self,on=1):
        """Set terminal status line: Data Terminal Ready"""
        self.pl.set_dtr(on)

    def getCTS(self):
        """Read terminal status line: Clear To Send"""
        status = self.pl.poll_modem_status()
        return (status & Prolific.MODEM_CTS) and True or False

    def getDSR(self):
        """Read terminal status line: Data Set Ready"""
        status = self.pl.poll_modem_status()
        return (status & Prolific.MODEM_DSR) and True or False

    def getRI(self):
        """Read terminal status line: Ring Indicator"""
        status = self.pl.poll_modem_status()
        return (status & Prolific.MODEM_RI) and True or False

    def getCD(self):
        """Read terminal status line: Carrier Detect"""
        status = self.pl.poll_modem_status()
        return (status & Prolific.MODEM_RLSD) and True or False
