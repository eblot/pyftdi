import re
import time
from pyftdi import Ftdi, FtdiError
from pyftdi.misc import to_int


class SerialFtdi:
    """Serial port implementation for FTDI compatible with pyserial API"""

    BAUDRATES = sorted(range(115200, 1000000, 115200) + \
                range(1000000, 13000000, 100000))
    VENDOR_IDS = { 'ftdi': 0x0403 }
    PRODUCT_IDS = { '232':  0x6001,
                    '2232': 0x6010,
                    '4232': 0x6011,
                    'ft232': 0x6001,
                    'ft2232': 0x6010,
                    'ft4232': 0x6011 }

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
                # unsupported this feature
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
        vre = r'|'.join(self.VENDOR_IDS.keys() + [r'(?:0x)?[a-f0-9]+'])
        pre = r'|'.join(self.PRODUCT_IDS.keys() + [r'(?:0x)?[a-f0-9]+'])
        usb_re = r'(?i)^ftdi://(?P<vendor>'+vre+r'):(?P<product>'+pre+r')/'
        def replace_usb(mo):
            vendor = mo.group('vendor')
            product = mo.group('product')
            return 'ftdi://%s:%s/' % (self.VENDOR_IDS.get(vendor, vendor),
                                      self.PRODUCT_IDS.get(product, product))
        ftdi_re = r'(?i)^ftdi://((?:0x)?[a-f0-9]+):((?:0x)?[a-f0-9]+)/(\d)\s*$'
        mo = re.match(ftdi_re, re.sub(usb_re, replace_usb, self.portstr))
        if not mo:
            raise serial.SerialException("Invalid FTDI device name")
        vendor = to_int(mo.group(1))
        product = to_int(mo.group(2))
        interface = to_int(mo.group(3))
        try:
            self.ftdi = Ftdi()
            self.ftdi.open(vendor, product, interface)
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
