""" pyftdi - A pure Python FTDI driver on top of pyusb
    Author:  Emmanuel Blot <emmanuel.blot@free.fr>
    License: LGPL, originally based on libftdi C library
    Caveats: Only tested with FT2232 and FT4232 FTDI devices
    Require: pyusb"""


import array
import os
import struct
import threading
import usb.core
import usb.util

__all__ = ['Ftdi', 'FtdiError']


class FtdiError(IOError):
    """Communication error with the FTDI device"""
    pass


class Ftdi(object):
    """FTDI device driver"""
    
    # Shifting commands IN MPSSE Mode
    MPSSE_WRITE_NEG = 0x01 # Write TDI/DO on negative TCK/SK edge
    MPSSE_BITMODE = 0x02   # Write bits, not bytes
    MPSSE_READ_NEG = 0x04  # Sample TDO/DI on negative TCK/SK edge
    MPSSE_LSB = 0x08       # LSB first
    MPSSE_DO_WRITE = 0x10  # Write TDI/DO
    MPSSE_DO_READ = 0x20   # Read TDO/DI
    MPSSE_WRITE_TMS = 0x40 # Write TMS/CS

    # Commands
    WRITE_BYTES_PVE_MSB = 0x10
    WRITE_BYTES_NVE_MSB = 0x11
    WRITE_BITS_PVE_MSB = 0x12
    WRITE_BITS_NVE_MSB = 0x13
    WRITE_BYTES_PVE_LSB = 0x18
    WRITE_BYTES_NVE_LSB = 0x19
    WRITE_BITS_PVE_LSB = 0x1a
    WRITE_BITS_NVE_LSB = 0x1b
    READ_BYTES_PVE_MSB = 0x20
    READ_BYTES_NVE_MSB = 0x24
    READ_BITS_PVE_MSB = 0x22
    READ_BITS_NVE_MSB = 0x26
    READ_BYTES_PVE_LSB = 0x28
    READ_BYTES_NVE_LSB = 0x2c
    READ_BITS_PVE_LSB = 0x2a
    READ_BITS_NVE_LSB = 0x2e
    RW_BYTES_NVE_LSB = 0x3d
    WRITE_BITS_TMS_PVE = 0x4a 
    WRITE_BITS_TMS_NVE = 0x4b
    RW_BITS_TMS_PVE_PVE = 0x6a
    RW_BITS_TMS_NVE_PVE = 0x6b
    RW_BITS_TMS_PVE_NVE = 0x6e
    RW_BITS_TMS_NVE_NVE = 0x6f
    SET_BITS_LOW = 0x80
    SET_BITS_HIGH = 0x82
    GET_BITS_LOW = 0x81
    GET_BITS_HIGH = 0x83
    LOOPBACK_START = 0x84
    LOOPBACK_END = 0x85
    TCK_DIVISOR = 0x86
    SEND_IMMEDIATE = 0x87
    WAIT_ON_HIGH = 0x88
    WAIT_ON_LOW = 0x89
    DISABLE_CLK_DIV5 = 0x8a
    ENABLE_CLK_DIV5 = 0x8b
    
    # Modem status
    MODEM_CTS = (1 << 4)    # Clear to send
    MODEM_DSR = (1 << 5)    # Data set ready
    MODEM_RI = (1 << 6)     # Ring indicator
    MODEM_RLSD = (1 << 7)   # Carrier detect
    MODEM_DR = (1 << 8)     # Data ready
    MODEM_OE = (1 << 9)     # Overrun error
    MODEM_PE = (1 << 10)    # Parity error
    MODEM_FE = (1 << 11)    # Framing error
    MODEM_BI = (1 << 12)    # Break interrupt
    MODEM_THRE = (1 << 13)  # Transmitter holding register
    MODEM_TEMT = (1 << 14)  # Transmitter empty
    MODEM_RCVE = (1 << 15)  # Error in RCVR FIFO
    
    # FTDI MPSSE commands 
    SET_BITS_LOW = 0x80     # Change LSB GPIO output
    SET_BITS_HIGH = 0x82    # Change MSB GPIO output
    GET_BITS_LOW = 0x81     # Get LSB GPIO output
    GET_BITS_HIGH = 0x83    # Get MSB GPIO output
    LOOPBACK_START = 0x84   # Enable loopback
    LOOPBACK_END = 0x85     # Disable loopback
    TCK_DIVISOR = 0x86

    BITMODE_RESET = 0x00    # switch off bitbang mode
    BITMODE_BITBANG = 0x01  # classical asynchronous bitbang mode
    BITMODE_MPSSE = 0x02    # MPSSE mode, available on 2232x chips
    BITMODE_SYNCBB = 0x04   # synchronous bitbang mode
    BITMODE_MCU = 0x08      # MCU Host Bus Emulation mode,
    BITMODE_OPTO = 0x10     # Fast Opto-Isolated Serial Interface Mode
    BITMODE_CBUS = 0x20     # Bitbang on CBUS pins of R-type chips
    BITMODE_SYNCFF = 0x40   # Single Channel Synchronous FIFO mode

    # Commands in MPSSE and Host Emulation Mode 
    SEND_IMMEDIATE = 0x87
    WAIT_ON_HIGH = 0x88
    WAIT_ON_LOW = 0x89

    # Commands in Host Emulation Mode 
    READ_SHORT = 0x90
    READ_EXTENDED = 0x91
    WRITE_SHORT = 0x92
    WRITE_EXTENDED = 0x93

    # USB control requests
    REQ_OUT = usb.util.build_request_type(
                  usb.util.CTRL_OUT,
                  usb.util.CTRL_TYPE_VENDOR,
                  usb.util.CTRL_RECIPIENT_DEVICE)
    REQ_IN = usb.util.build_request_type(
                  usb.util.CTRL_IN,
                  usb.util.CTRL_TYPE_VENDOR,
                  usb.util.CTRL_RECIPIENT_DEVICE)

    # Requests 
    SIO_RESET = 0              # Reset the port
    SIO_SET_MODEM_CTRL = 1     # Set the modem control register
    SIO_SET_FLOW_CTRL = 2      # Set flow control register
    SIO_SET_BAUDRATE = 3       # Set baud rate
    SIO_SET_DATA = 4           # Set the data characteristics of the port
    SIO_POLL_MODEM_STATUS = 5  # Get line status
    SIO_SET_EVENT_CHAR = 6     # Change event character
    SIO_SET_ERROR_CHAR = 7     # Change error character
    SIO_SET_LATENCY_TIMER = 9  # Change latency timer
    SIO_GET_LATENCY_TIMER = 10 # Get latency timer
    SIO_SET_BITMODE = 11       # Change bit mode
    SIO_READ_PINS = 12         # Read GPIO pin value

    # Eeprom requests
    SIO_EEPROM = 0x90
    SIO_READ_EEPROM = SIO_EEPROM + 0   # Read EEPROM content
    SIO_WRITE_EEPROM = SIO_EEPROM + 1  # Write EEPROM content
    SIO_ERASE_EEPROM = SIO_EEPROM + 2  # Erase EEPROM content

    # Reset commands
    SIO_RESET_SIO = 0          # Reset device
    SIO_RESET_PURGE_RX = 1     # Drain RX buffer
    SIO_RESET_PURGE_TX = 2     # Drain TX buffer

    # Flow control
    SIO_DISABLE_FLOW_CTRL = 0x0
    SIO_RTS_CTS_HS = (0x1 << 8)
    SIO_DTR_DSR_HS = (0x2 << 8)
    SIO_XON_XOFF_HS = (0x4 << 8)
    SIO_SET_DTR_MASK = 0x1
    SIO_SET_DTR_HIGH = (SIO_SET_DTR_MASK | (SIO_SET_DTR_MASK << 8))
    SIO_SET_DTR_LOW = (0x0 | (SIO_SET_DTR_MASK << 8))
    SIO_SET_RTS_MASK = 0x2
    SIO_SET_RTS_HIGH = (SIO_SET_RTS_MASK | (SIO_SET_RTS_MASK << 8))
    SIO_SET_RTS_LOW = (0x0 | (SIO_SET_RTS_MASK << 8))

    # Parity bits
    PARITY_NONE, PARITY_ODD, PARITY_EVEN, PARITY_MARK, PARITY_SPACE = range(5)
    # Number of stop bits
    STOP_BIT_1, STOP_BIT_15, STOP_BIT_2 = range(3)
    # Number of bits
    BITS_7, BITS_8 = [7+i for i in range(2)]
    # Break type
    BREAK_OFF, BREAK_ON = range(2)

    BAUDRATE_REF_CLOCK = 3000000 # 3 MHz
    BAUDRATE_TOLERANCE = 3.0 # acceptable clock drift, in %
    BITBANG_CLOCK_MULTIPLIER = 4
    LATENCY_MIN = 1
    LATENCY_MAX = 255
    LATENCY_THRESHOLD = 1000
    
    # Need to maintain a list of reference USB devices, to circumvent a 
    # limitation in pyusb that prevents from opening several times the same 
    # USB device. The following dictionary used vendor/product keys
    # to track (device, refcount) pairs
    DEVICES = {}
    LOCK = threading.Lock()
    
    def __init__(self):
        self.usb_dev = None
        self.usb_read_timeout = 5000
        self.usb_write_timeout = 5000
        self.baudrate = -1
        self.readbuffer = array.array('B')
        self.readoffset = 0
        self.readbuffer_chunksize = 4 << 10 # 4KB
        self.writebuffer_chunksize = 4 << 10 # 4KB
        self.max_packet_size = 0
        self.index = None
        self.in_ep = None
        self.out_ep = None
        self.bitbang_mode = Ftdi.BITMODE_RESET
        self.latency = 0
        self.latency_count = 0
        self.latency_min = self.LATENCY_MIN
        self.latency_max = self.LATENCY_MAX
        self.latency_threshold = None # disable dynamic latency
    
    # --- Public API -------------------------------------------------------
    
    def open(self, vendor=0x403, product=0x6011, interface=1):
        """Open a new interface to the specified FTDI device"""
        self.usb_dev = self._get_device(vendor, product)
        self._set_interface(interface)
        self.max_packet_size = self._get_max_packet_size()
        self._reset_device()

    def close(self):
        """Closing the FTDI interface"""
        self._release_device(self)

    @property
    def type(self):
        """Return the current type of the FTDI device as a string"""
        types = { 0x200: 'ft232am',
                  0x400: 'ft232bm', # bug with S/N == 0 not handled
                  0x500: 'ft2232c',
                  0x600: 'ft232c',
                  0x700: 'ft2232h',
                  0x800: 'ft4232h' }
        return types[self.usb_dev.bcdDevice]

    @property
    def bitbang_enabled(self):
        """Tell whether some bitbang mode is activated"""
        return not (self.bitbang_mode == Ftdi.BITMODE_RESET)

    def set_baudrate(self, baudrate):
        """Change the current interface baudrate"""
        if self.bitbang_enabled:
            baudrate *= Ftdi.BITBANG_CLOCK_MULTIPLIER
        actual, value, index = self._convert_baudrate(baudrate)
        delta = 100*abs(float(actual-baudrate))/baudrate
        if delta > Ftdi.BAUDRATE_TOLERANCE:
            raise AssertionError('Cannot represent baudrate')
        try:
            if self.usb_dev.ctrl_transfer(Ftdi.REQ_OUT,
                                          Ftdi.SIO_SET_BAUDRATE, value,
                                          index, '', self.usb_write_timeout):
                raise FtdiError('Unable to set baudrate')
            self.baudrate = baudrate
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def purge_rx_buffer(self):
        """Clears the read buffer on the chip and the internal read buffer."""
        if self._ctrl_transfer_out(Ftdi.SIO_RESET, SIO_RESET_PURGE_RX):
            raise FtdiError('Unable to set baudrate')
        # Invalidate data in the readbuffer
        ftdi.readbuffer_offset = 0
        ftdi.readbuffer = array.array('B')

    def purge_tx_buffer(self):
        """Clears the write buffer on the chip."""
        if self._ctrl_transfer_out(Ftdi.SIO_RESET, SIO_RESET_PURGE_TX):
            raise FtdiError('Unable to set baudrate')

    def purge_buffers(self):
        """Clears the buffers on the chip and the internal read buffer."""
        self.purge_rx_buffer()
        self.purge_tx_buffer()

    # --- todo: Replace with properties -----------
    def write_data_set_chunksize(self, chunksize):
        """Configure write buffer chunk size."""
        self.writebuffer_chunksize = chunksize

    def write_data_get_chunksize(self):
        """Get write buffer chunk size."""
        return self.writebuffer_chunksize

    def read_data_set_chunksize(self, chunksize):
        """Configure read buffer chunk size."""
        # Invalidate all remaining data
        self.readoffset = 0
        self.readbuffer = array.array('B')
        import sys
        if sys.platform == 'linux':
            if chunksize > 16384:
                chunksize = 16384
        self.readbuffer = []
        self.readbuffer_chunksize = chunksize

    def read_data_get_chunksize(self):
        """Get read buffer chunk size."""
        return self.readbuffer_chunksize
    # --- end of todo section ---------------------

    def set_bitmode(self, bitmask, mode):
        """Enable/disable bitbang modes."""
        value = bitmask | (mode << 8)
        if self._ctrl_transfer_out(Ftdi.SIO_SET_BITMODE, value):
            raise FtdiError('Unable to set bitmode')
        self.bitbang_mode = mode
    
    def read_pins(self):
        """Directly read pin state, circumventing the read buffer. 
           Useful for bitbang mode."""
        pins = self._ctrl_transfer_in(Ftdi.SIO_READ_PINS, 1)
        if not pins:
            raise FtdiError('Unable to read pins')
        return pins[0]

    def set_latency_timer(self, latency):
        """Set latency timer
           The FTDI chip keeps data in the internal buffer for a specific
           amount of time if the buffer is not full yet to decrease
           load on the usb bus."""
        if not (Ftdi.LATENCY_MIN <= latency <= Ftdi.LATENCY_MAX):
            raise AssertionError("Latency out of range")
        if self._ctrl_transfer_out(Ftdi.SIO_SET_LATENCY_TIMER, latency):
            raise FtdiError('Unable to latency timer')

    def get_latency_timer(self):
        """Get latency timer"""
        latency = self._ctrl_transfer_in(Ftdi.SIO_GET_LATENCY_TIMER, 1)
        if not latency:
            raise FtdiError('Unable to get latency')
        return latency[0]

    def poll_modem_status(self):
        """Poll modem status information
           This function allows the retrieve the two status bytes of the device.
           The device sends these bytes also as a header for each read access
           where they are discarded by ftdi_read_data(). The chip generates
           the two stripped status bytes in the absence of data every 40 ms.
           Layout of the first byte:
           - B0..B3 - must be 0
           - B4       Clear to send (CTS)  0 = inactive / 1 = active
           - B5       Data set ready (DTS) 0 = inactive / 1 = active
           - B6       Ring indicator (RI)  0 = inactive / 1 = active
           - B7       Receive line signal detect (RLSD) 
                                           0 = inactive / 1 = active
           Layout of the second byte:
           - B0       Data ready (DR)
           - B1       Overrun error (OE)
           - B2       Parity error (PE)
           - B3       Framing error (FE)
           - B4       Break interrupt (BI)
           - B5       Transmitter holding register (THRE)
           - B6       Transmitter empty (TEMT)
           - B7       Error in RCVR FIFO"""
        value = self._ctrl_transfer_in(Ftdi.SIO_POLL_MODEM_STATUS, 2)
        if not value or len(value) != 2:
            raise FtdiError('Unable to get modem status')
        status, = struct.unpack('<H', value)
        return status

    def set_flowctrl(self, flowctrl):
        """Set flowcontrol for ftdi chip"""
        value = flowctrl | self.index
        try:
            if self.usb_dev.ctrl_transfer(Ftdi.REQ_OUT,
                                          Ftdi.SIO_SET_FLOW_CTRL, 0,
                                          value, '', self.usb_write_timeout):
                raise FtdiError('Unable to set flow control')
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def set_dtr(self, state):
        """Set dtr line"""
        value = state and Ftdi.SIO_SET_DTR_HIGH or Ftdi.SIO_SET_DTR_LOW
        if self._ctrl_transfer_out(Ftdi.SIO_SET_MODEM_CTRL, value):
            raise FtdiError('Unable to set DTR line')

    def set_rts(self, state):
        """Set rts line"""
        value = state and Ftdi.SIO_SET_RTS_HIGH or Ftdi.SIO_SET_RTS_LOW
        if self._ctrl_transfer_out(Ftdi.SIO_SET_MODEM_CTRL, value):
            raise FtdiError('Unable to set RTS line')

    def set_dtr_rts(self, dtr, rts):
        """Set dtr and rts lines"""
        value = 0
        value |= dtr and Ftdi.SIO_SET_DTR_HIGH or Ftdi.SIO_SET_DTR_LOW
        value |= state and Ftdi.SIO_SET_RTS_HIGH or Ftdi.SIO_SET_RTS_LOW
        if self._ctrl_transfer_out(Ftdi.SIO_SET_FLOW_CTRL, value):
            raise FtdiError('Unable to set DTR/RTS lines')

    def set_event_char(self, eventch, enable):
        """Set the special event character"""
        value = eventch
        if enable:
            value |= 1 << 8
        if self._ctrl_transfer_out(Ftdi.SIO_SET_EVENT_CHAR, value):
            raise FtdiError('Unable to set DTR/RTS lines')

    def set_error_char(self, errorch, enable):
        """Set error character"""
        value = errorch
        if enable:
            value |= 1 << 8
        if self._ctrl_transfer_out(Ftdi.SIO_SET_ERROR_CHAR, value):
            raise FtdiError('Unable to set DTR/RTS lines')

    def set_line_property(self, bits, stopbits, parity, break_=0):
        """Set (RS232) line characteristics"""
        value = bits & 0x0F
        try:
            value |= { Ftdi.PARITY_NONE : 0x00 << 8,
                       Ftdi.PARITY_ODD : 0x01 << 8,
                       Ftdi.PARITY_EVEN : 0x02 << 8,
                       Ftdi.PARITY_MARK : 0x03 << 8,
                       Ftdi.PARITY_SPACE : 0x04 << 8 }[parity]
            value |= { Ftdi.STOP_BIT_1 : 0x00 << 11,
                       Ftdi.STOP_BIT_15 : 0x01 << 11,
                       Ftdi.STOP_BIT_2 : 0x02 << 11 }[stopbits]
            if break_ == Ftdi.BREAK_ON:
                value |= 0x01 << 14
        except KeyError:
            raise AssertionError('Invalid line property')
        if self._ctrl_transfer_out(Ftdi.SIO_SET_DATA, value):
            raise FtdiError('Unable to set line property')

    def write_data(self, data):
        """Writes data in chunks (see write_data_set_chunksize) to the chip"""
        offset = 0
        size = len(data)
        try:
            while offset < size:
                write_size = self.writebuffer_chunksize
                if offset + write_size > size:
                    write_size = size - offset
                length = self.usb_dev.write(self.in_ep,
                                            data[offset:offset+write_size],
                                            self.index-1,
                                            self.usb_write_timeout)
                if length <= 0:
                    raise FtdiError("Usb bulk write error")
                offset += length
            return offset
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def read_data(self, size):
        """Reads data in chunks (see read_data_set_chunksize) from the chip.
           Automatically strips the two modem status bytes transfered during 
           every read."""
        # Packet size sanity check
        if not self.max_packet_size:
            raise FtdiError("max_packet_size is bogus")
        packet_size = self.max_packet_size
        length = 1 # initial condition to enter the usb_read loop
        data = array.array('B')
        # everything we want is still in the cache?
        if size <= len(self.readbuffer)-self.readoffset:
            data = self.readbuffer[self.readoffset:self.readoffset+size]
            self.readoffset += size
            return data.tostring()
        # something still in the cache, but not enough to satisfy 'size'?
        if len(self.readbuffer)-self.readoffset != 0:
            data = self.readbuffer[self.readoffset:]
            # end of readbuffer reached
            self.readoffset = len(self.readbuffer)
        # read from USB, filling in the local cache as it is empty
        try:
            while (len(data) < size) and (length > 0):
                tempbuf = self.usb_dev.read(self.out_ep,
                                            self.readbuffer_chunksize,
                                            self.index-1,
                                            self.usb_read_timeout)
                length = len(tempbuf)
                # the received buffer contains at least one useful databyte
                # (first 2 bytes in each packet represent the current modem status)
                if length > 2:
                    if self.latency_threshold:
                        self.latency_count = 0
                        if self.latency != self.latency_min:
                            self.set_latency_timer(self.latency_min)
                            self.latency = self.latency_min
                    # skip the status bytes
                    chunks = (length+packet_size-1) // packet_size
                    count = packet_size - 2
                    self.readbuffer = array.array('B')
                    self.readoffset = 0
                    srcoff = 2
                    for i in xrange(chunks):
                        self.readbuffer += tempbuf[srcoff:srcoff+count]
                        srcoff += packet_size
                    length = len(self.readbuffer)
                else:
                    # received buffer only contains the modem status bytes
                    # clear them out
                    self.readbuffer = array.array('B')
                    self.readoffset = 0
                    if self.latency_threshold:
                        self.latency_count += 1
                        if self.latency != self.latency_max:
                            if self.latency_count > self.latency_threshold:
                                self.set_latency_timer(self.latency_max)
                                self.latency = self.latency_max
                    # no more data to read?
                    return data.tostring()
                if length > 0:
                    # data still fits in buf?
                    if (len(data) + length) <= size:
                        data += self.readbuffer[self.readoffset: \
                                                self.readoffset+length]
                        self.readoffset += length
                        # did we read exactly the right amount of bytes?
                        if len(data) == size:
                            return data.tostring()
                    else:
                        # partial copy, not enough bytes in the local cache to
                        # fulfill the request
                        part_size = min(size-len(data),
                                        len(self.readbuffer)-self.readoffset)
                        if part_size < 0:
                            raise AssertionError("Internal Error")
                        data += self.readbuffer[self.readoffset:\
                                                self.readoffset+part_size]
                        self.readoffset += part_size
                        return data.tostring()
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))
        # never reached
        raise FtdiError("Internal error")

    def set_dynamic_latency(self, lmin, lmax, threshold):
        """Set up or disable latency values"""
        if not threshold:
            self.latency_count = 0
            self.latency_threshold = None
        else:
            for lat in (lmin, lmax):
                if not (0 < lat < 256):
                    raise AssertionError("Latency out of range: %d")
            self.latency_min = lmin
            self.latency_max = lmax
            self.latency_threshold = threshold
            self.latency = lmax
            self.set_latency_timer(self.latency)

    def get_error_string(self):
        """Wrapper for libftdi compatibility"""
        return "Unknown error"

    # --- Private implementation -------------------------------------------
    
    @classmethod
    def _get_device(cls, vendor, product):
        """Find a previously open device with the same vendor/product
           or initialize a new one, and return it"""
        device = (vendor<<16)+product
        cls.LOCK.acquire()
        try:
            if device not in cls.DEVICES:
                dev = usb.core.find(idVendor=vendor, idProduct=product)
                if not dev:
                    raise ValueError('Device not found')
                for configuration in dev:
                    # we need to detach any kernel driver from the device
                    # be greedy: reclaim all device interfaces from the kernel
                    for interface in configuration: 
                        ifnum = interface.bInterfaceNumber
                        if not dev.is_kernel_driver_active(ifnum):
                            continue
                        try:
                            dev.detach_kernel_driver(ifnum)
                        except usb.core.USBError, e:
                            pass
                dev.set_configuration()
                cls.DEVICES[device] = [dev, 1]
            else:
                cls.DEVICES[device][1] += 1
            return cls.DEVICES[device][0]
        finally:
            cls.LOCK.release()
    
    @classmethod
    def _release_device(cls, usb_dev):
        """Release a previously open device, if it not used anymore"""
        # Lookup for ourselves in the class dictionary
        cls.LOCK.acquire()
        try:
            for device in cls.DEVICES:
                dev, refcount = cls.DEVICES[device]
                if dev == usb_dev:
                    # found
                    if refcount > 1:
                        # another interface is open, decrement
                        cls.DEVICES[device][1] -= 1
                    else:
                        # last interface in use, release
                        del cls.DEVICES[device]
                    break
        finally:
            cls.LOCK.release()
            
    def _set_interface(self, interface):
        """Select the interface to use on the FTDI device"""
        if interface == 0:
            interface = 1
        if interface not in xrange(1, 5):
            # should use the actual interface count, depending on the device
            raise ValueError("Interface does not exist")
        self.index = interface
        self.in_ep = 2*interface
        self.out_ep = 0x80 + self.in_ep - 1
        
    def _reset_device(self):
        """Reset the ftdi device"""
        if self._ctrl_transfer_out(Ftdi.SIO_RESET, Ftdi.SIO_RESET_SIO):
            raise FtdiError('Unable to reset FTDI device')
        # Invalidate data in the readbuffer
        self.readoffset = 0
        self.readbuffer = array.array('B')

    def _ctrl_transfer_out(self, reqtype, value, data=''):
        try:
            return self.usb_dev.ctrl_transfer(Ftdi.REQ_OUT, reqtype, value,
                                              self.index, data,
                                              self.usb_write_timeout)
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def _ctrl_transfer_in(self, reqtype, length):
        try:
            return self.usb_dev.ctrl_transfer(Ftdi.REQ_IN, reqtype, 0,
                                              self.index, length,
                                              self.usb_read_timeout)
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def _get_max_packet_size(self):
        """Retrieve the maximum length of a data packet"""
        if not self.usb_dev:
            raise AssertionError("Device is not yet known")
        if self.type in ('ft2232h', 'ft4232h'):
            packet_size = 512
        else:
            packet_size = 64
        cfg_iter = self.usb_dev.__iter__()
        cfg = next(cfg_iter)
        if self.index < cfg.bNumInterfaces:
            interface = cfg[(self.index, 0)]
            endpoint = interface[0]
            packet_size = endpoint.wMaxPacketSize
        return packet_size

    def _convert_baudrate(self, baudrate):
        """Convert a requested baudrate into the closest possible baudrate
           that can be assigned to the FTDI device"""
        am_adjust_up = [0, 0, 0, 1, 0, 3, 2, 1]
        am_adjust_dn = [0, 0, 0, 1, 0, 1, 2, 3]
        frac_code = [0, 3, 2, 4, 1, 5, 6, 7]
        #int divisor, best_divisor, best_baud, best_baud_diff
        #unsigned long encoded_divisor
        #int i
        ref_clock = Ftdi.BAUDRATE_REF_CLOCK
        hispeed = False
        if baudrate <= 0:
            raise AssertionError('Invalid baudrate: %d' % baudrate)
        if self.type in ('ft2232h', 'ft2432h'):
            # these chips can support a 12MHz clock in addition to the original
            # 3MHz clock. This allows higher baudrate (up to 12Mbps) and more
            # precise baudrates for baudrate > 3Mbps/2
            if baudrate > (Ftdi.BAUDRATE_REF_CLOCK>>1):
                ref_clock *= 4 # 12 MHz
                hispeed = True
        divisor = (ref_clock<<3) // baudrate
        if self.type == 'ft232am':
            # Round down to supported fraction (AM only)
            divisor -= am_adjust_dn[divisor & 7]
        # Try this divisor and the one above it (because division rounds down)
        best_divisor = 0
        best_baud = 0
        best_baud_diff = 0
        for i in xrange(0, 2):
            try_divisor = divisor + i
            if not hispeed:
                # Round up to supported divisor value
                if try_divisor <= 8:
                    # Round up to minimum supported divisor
                    try_divisor = 8
                elif self.type not in 'ft232am' and try_divisor < 12:
                    # BM doesn't support divisors 9 through 11 inclusive
                    try_divisor = 12
                elif divisor < 16:
                    # AM doesn't support divisors 9 through 15 inclusive
                    try_divisor = 16
                else:
                    if self.type in 'ft232am':
                        # Round up to supported fraction (AM only)
                        try_divisor += am_adjust_up[try_divisor & 7]
                        if try_divisor > 0x1FFF8:
                            # Round down to maximum supported div value (AM)
                            try_divisor = 0x1FFF8
                    else:
                        if try_divisor > 0x1FFFF:
                            # Round down to maximum supported div value (BM)
                            try_divisor = 0x1FFFF
            # Get estimated baud rate (to nearest integer)
            baud_estimate = ((ref_clock<<3) + (try_divisor//2))//try_divisor
            # Get absolute difference from requested baud rate
            if baud_estimate < baudrate:
                baud_diff = baudrate - baud_estimate
            else:
                baud_diff = baud_estimate - baudrate
            if (i == 0) or (baud_diff < best_baud_diff):
                # Closest to requested baud rate so far
                best_divisor = try_divisor
                best_baud = baud_estimate
                best_baud_diff = baud_diff
                if baud_diff == 0:
                    break
        # Encode the best divisor value
        encoded_divisor = (best_divisor >> 3) | \
                          (frac_code[best_divisor & 7] << 14)
        # Deal with special cases for encoded value
        if encoded_divisor == 1:
            encoded_divisor = 0 # 3000000 baud
        elif encoded_divisor == 0x4001:
            encoded_divisor = 1 # 2000000 baud (BM only)
        # Split into "value" and "index" values
        value = encoded_divisor & 0xFFFF
        if self.type in ('ft2232c', 'ft2232h', 'ft4232h'):
            index = (encoded_divisor >> 8) & 0xFFFF
            index &= 0xFF00
            index |= self.index
        else:
            index = (encoded_divisor >> 16) & 0xFFFF
        if hispeed:
            index |= 1<<9 # use hispeed mode
        return (best_baud, value, index)


# --- to be removed ---------------
VENDOR_IDS = { 'ftdi': 0x0403 }
PRODUCT_IDS = { '232':  0x6001,
                '2232': 0x6010,
                '4232': 0x6011,
                'ft232': 0x6001,
                'ft2232': 0x6010,
                'ft4232': 0x6011 }
#----------------------------------
