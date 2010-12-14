""" pyftdi - A pure Python FTDI driver on top of pyusb
    Author:  Emmanuel Blot <emmanuel.blot@free.fr>
    License: LGPL, originally based on libftdi C library
    Caveats: Only tested with FT2232 and FT4232 FTDI devices
    Require: pyusb """


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
    MPSSE_BITMODE = 0x02 # Write bits, not bytes 
    MPSSE_READ_NEG = 0x04 # Sample TDO/DI on negative TCK/SK edge 
    MPSSE_LSB = 0x08 # LSB first 
    MPSSE_DO_WRITE = 0x10 # Write TDI/DO 
    MPSSE_DO_READ = 0x20 # Read TDO/DI 
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
    SET_BITS_LOW = 0x80
    SET_BITS_HIGH = 0x82
    GET_BITS_LOW = 0x81
    GET_BITS_HIGH = 0x83
    LOOPBACK_START = 0x84
    LOOPBACK_END = 0x85
    TCK_DIVISOR = 0x86

    BITMODE_RESET = 0x00   # switch off bitbang mode
    BITMODE_BITBANG = 0x01 # classical asynchronous bitbang mode
    BITMODE_MPSSE = 0x02   # MPSSE mode, available on 2232x chips
    BITMODE_SYNCBB = 0x04  # synchronous bitbang mode
    BITMODE_MCU = 0x08     # MCU Host Bus Emulation mode,
    BITMODE_OPTO = 0x10    # Fast Opto-Isolated Serial Interface Mode
    BITMODE_CBUS = 0x20    # Bitbang on CBUS pins of R-type chips
    BITMODE_SYNCFF = 0x40  # Single Channel Synchronous FIFO mode

    # Commands in MPSSE and Host Emulation Mode 
    SEND_IMMEDIATE = 0x87
    WAIT_ON_HIGH = 0x88
    WAIT_ON_LOW = 0x89

    # Commands in Host Emulation Mode 
    READ_SHORT = 0x90
    READ_EXTENDED = 0x91
    WRITE_SHORT = 0x92
    WRITE_EXTENDED = 0x93

    # Definitions for flow control 
    SIO_RESET = 0 # Reset the port 
    SIO_MODEM_CTRL = 1 # Set the modem control register 
    SIO_SET_FLOW_CTRL = 2 # Set flow control register 
    SIO_SET_BAUD_RATE = 3 # Set baud rate 
    SIO_SET_DATA = 4 # Set the data characteristics of the port 

    # USB control requests
    DEV_OUT_REQTYPE = (usb.util.CTRL_TYPE_VENDOR << 5) | \
                       usb.util.CTRL_RECIPIENT_DEVICE | \
                       usb.util.CTRL_OUT
    DEV_IN_REQTYPE = (usb.util.CTRL_TYPE_VENDOR << 5) | \
                      usb.util.CTRL_RECIPIENT_DEVICE | \
                      usb.util.CTRL_IN

    # Requests 
    SIO_RESET_REQUEST = SIO_RESET
    SIO_SET_BAUDRATE_REQUEST = SIO_SET_BAUD_RATE
    SIO_SET_DATA_REQUEST = SIO_SET_DATA
    SIO_SET_FLOW_CTRL_REQUEST = SIO_SET_FLOW_CTRL
    SIO_SET_MODEM_CTRL_REQUEST = SIO_MODEM_CTRL
    SIO_POLL_MODEM_STATUS_REQUEST = 0x05
    SIO_SET_EVENT_CHAR_REQUEST = 0x06
    SIO_SET_ERROR_CHAR_REQUEST = 0x07
    SIO_SET_LATENCY_TIMER_REQUEST = 0x09
    SIO_GET_LATENCY_TIMER_REQUEST = 0x0A
    SIO_SET_BITMODE_REQUEST = 0x0B
    SIO_READ_PINS_REQUEST = 0x0C
    SIO_READ_EEPROM_REQUEST = 0x90
    SIO_WRITE_EEPROM_REQUEST = 0x91
    SIO_ERASE_EEPROM_REQUEST = 0x92

    SIO_RESET_SIO = 0
    SIO_RESET_PURGE_RX = 1
    SIO_RESET_PURGE_TX = 2

    # Flow control
    SIO_DISABLE_FLOW_CTRL = 0x0
    SIO_RTS_CTS_HS = (0x1 << 8)
    SIO_DTR_DSR_HS = (0x2 << 8)
    SIO_XON_XOFF_HS = (0x4 << 8)
    SIO_SET_DTR_MASK = 0x1
    SIO_SET_DTR_HIGH = ( 1 | ( SIO_SET_DTR_MASK  << 8))
    SIO_SET_DTR_LOW = ( 0 | ( SIO_SET_DTR_MASK  << 8))
    SIO_SET_RTS_MASK = 0x2
    SIO_SET_RTS_HIGH = ( 2 | ( SIO_SET_RTS_MASK << 8 ))
    SIO_SET_RTS_LOW = ( 0 | ( SIO_SET_RTS_MASK << 8 ))

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
        self.readbuffer_offset = 0
        self.readbuffer_remaining = 0
        self.readbuffer_chunksize = 4096
        self.writebuffer_chunksize = 4096
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
    
    def open(self, vendor=0x403, product=0x6011, interface=0):
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
            baudrate *= 4
        actual, value, index = self._convert_baudrate(baudrate)
        delta = 100*abs(float(actual-baudrate))/baudrate
        if delta > Ftdi.BAUDRATE_TOLERANCE:
            raise AssertionError('Cannot represent baudrate')
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_BAUDRATE_REQUEST, value,
                                      index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set baudrate')
        self.baudrate = baudrate

    def purge_rx_buffer(self):
        """Clears the read buffer on the chip and the internal read buffer."""
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_RESET_REQUEST, 
                                      Ftdi.SIO_RESET_PURGE_RX,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set baudrate')
        # Invalidate data in the readbuffer
        ftdi.readbuffer_offset = 0
        ftdi.readbuffer_remaining = 0

    def purge_tx_buffer(self):
        """Clears the write buffer on the chip."""
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_RESET_REQUEST, 
                                      Ftdi.SIO_RESET_PURGE_TX,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set baudrate')

    def purge_buffers(self):
        """Clears the buffers on the chip and the internal read buffer."""
        self.purge_rx_buffer()
        self.purge_tx_buffer()

    # ---Replace with properties-------
    def write_data_set_chunksize(self, chunksize):
        """Configure write buffer chunk size."""
        self.writebuffer_chunksize = chunksize

    def write_data_get_chunksize(self):
        """Get write buffer chunk size."""
        return self.writebuffer_chunksize

    def read_data_set_chunksize(self, chunksize):
        """Configure read buffer chunk size."""
        # Invalidate all remaining data
        self.readbuffer_offset = 0
        self.readbuffer_remaining = 0
        import sys
        if sys.platform == 'linux':
            if chunksize > 16384:
                chunksize = 16384
        self.readbuffer = []
        self.readbuffer_chunksize = chunksize

    def read_data_get_chunksize(self):
        """Get read buffer chunk size."""
        return self.readbuffer_chunksize
    # ---------------------------------

    def set_bitmode(self, bitmask, mode):
        """Enable/disable bitbang modes."""
        usb_val = bitmask # low byte: bitmask
        usb_val |= mode << 8
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_BITMODE_REQUEST, usb_val,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set bitmode')
        self.bitbang_mode = mode
    
    def read_pins(self):
        """Directly read pin state, circumventing the read buffer. 
           Useful for bitbang mode."""
        pins = self.usb_dev.ctrl_transfer(Ftdi.DEV_IN_REQTYPE,
                                          Ftdi.SIO_READ_PINS_REQUEST, 0,
                                          self.index, 1, 
                                          self.usb_read_timeout)
        if not pins:
            raise FtdiError('Unable to read pins')
        return pins[0]

    def set_latency_timer(self, latency):
        """Set latency timer
           The FTDI chip keeps data in the internal buffer for a specific
           amount of time if the buffer is not full yet to decrease
           load on the usb bus."""
        if not (0 < latency < 256):
            raise AssertionError("Latency out of range")
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_LATENCY_TIMER_REQUEST, 
                                      latency,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to latency timer')

    def get_latency_timer(self):
        """Get latency timer"""
        latency = self.usb_dev.ctrl_transfer(Ftdi.DEV_IN_REQTYPE,
                                             Ftdi.SIO_READ_PINS_REQUEST, 0,
                                             self.index, 1, 
                                             self.usb_read_timeout)
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
        usb_val = self.usb_dev.ctrl_transfer(Ftdi.DEV_IN_REQTYPE, 
                                             Ftdi.SIO_POLL_MODEM_STATUS_REQUEST, 
                                             0, self.index, 2, 
                                             self.usb_read_timeout)
        if not usb_val or len(usb_val) != 2:
            raise FtdiError('Unable to get modem status')
        status, = struct.unpack('<H', usb_val)
        return status

    def set_flowctrl(self, flowctrl):
        """Set flowcontrol for ftdi chip"""
        usb_val = flowctrl | self.index
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_FLOW_CTRL_REQUEST, 0,
                                      usb_val, '', self.usb_write_timeout):
            raise FtdiError('Unable to set flow control')

    def set_dtr(self, state):
        """Set dtr line"""
        usb_val = state and Ftdi.SIO_SET_DTR_HIGH or Ftdi.SIO_SET_DTR_LOW
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_MODEM_CTRL_REQUEST, usb_val,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set DTR line')

    def set_rts(self, state):
        """Set rts line"""
        usb_val = state and Ftdi.SIO_SET_RTS_HIGH or Ftdi.SIO_SET_RTS_LOW
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_MODEM_CTRL_REQUEST, usb_val,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set RTS line')

    def set_dtr_rts(self, dtr, rts):
        """Set dtr and rts line in one pass"""
        usb_val = 0
        usb_val |= dtr and Ftdi.SIO_SET_DTR_HIGH or Ftdi.SIO_SET_DTR_LOW
        usb_val |= state and Ftdi.SIO_SET_RTS_HIGH or Ftdi.SIO_SET_RTS_LOW
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_MODEM_CTRL_REQUEST, usb_val,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set DTR/RTS lines')

    def set_event_char(self, eventch, enable):
        """Set the special event character"""
        usb_val = eventch
        if enable:
            usb_val |= 1 << 8
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_EVENT_CHAR_REQUEST, usb_val,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set DTR/RTS lines')

    def set_error_char(self, errorch, enable):
        """Set error character"""
        usb_val = errorch
        if enable:
            usb_val |= 1 << 8
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_ERROR_CHAR_REQUEST, usb_val,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set DTR/RTS lines')

    def set_line_property(self, bits, stopbits, parity, break_=0):
        """Set (RS232) line characteristics"""
        usb_val = bits & 0x0F
        try:
            usb_val |= { Ftdi.PARITY_NONE : 0x00 << 8,
                         Ftdi.PARITY_ODD : 0x01 << 8,
                         Ftdi.PARITY_EVEN : 0x02 << 8, 
                         Ftdi.PARITY_MARK : 0x03 << 8,
                         Ftdi.PARITY_SPACE : 0x04 << 8 }[parity]
            usb_val |= { Ftdi.STOP_BIT_1 : 0x00 << 11,
                         Ftdi.STOP_BIT_15 : 0x01 << 11,
                         Ftdi.STOP_BIT_2 : 0x02 << 11 }[stopbits]
            if break_ == Ftdi.BREAK_ON:
                usb_val |= 0x01 << 14
        except KeyError:
            raise AssertionError('Invalid line property')
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_SET_DATA_REQUEST, usb_val,
                                      self.index, '', self.usb_write_timeout):
            raise FtdiError('Unable to set line property')

    def write_data(self, data):
        """Writes data in chunks (see write_data_set_chunksize) to the chip"""
        offset = 0
        size = len(data)
        while offset < size:
            write_size = self.writebuffer_chunksize
            if offset + write_size > size:
                write_size = size - offset
            actual_length = self.usb_dev.write(self.in_ep, 
                                               data[offset:offset+write_size], 
                                               self.index-1, 
                                               self.usb_write_timeout)
            if actual_length <= 0:
                raise FtdiError("Usb bulk write error")
            offset += actual_length
        return offset

    def read_data(self, size):
        """Reads data in chunks (see read_data_set_chunksize) from the chip.
           Automatically strips the two modem status bytes transfered during 
           every read."""
        # Packet size sanity check (avoid division by zero)
        if not self.max_packet_size:
            raise AssertionError("max_packet_size is bogus")
        offset = 0
        packet_size = self.max_packet_size
        actual_length = 1
        data = array.array('B')
        # everything we want is still in the readbuffer?
        if size <= self.readbuffer_remaining:
            data = self.readbuffer[self.readbuffer_offset:\
                                   self.readbuffer_offset+size]
            # Fix offsets
            self.readbuffer_remaining -= size
            self.readbuffer_offset += size
            return data.tostring()
        # something still in the readbuffer, but not enough to satisfy 'size'?
        if self.readbuffer_remaining != 0:
            data[:] = self.readbuffer[self.readbuffer_offset:\
                                      self.readbuffer_offset + \
                                      self.readbuffer_remaining]
            # Fix offset
            offset += self.readbuffer_remaining
        # do the actual USB read
        while (offset < size) and (actual_length > 0):
            self.readbuffer_remaining = 0
            self.readbuffer_offset = 0
            self.readbuffer = self.usb_dev.read(self.out_ep,
                                                self.readbuffer_chunksize,
                                                self.index-1,
                                                self.usb_read_timeout)
            actual_length = len(self.readbuffer)
            if actual_length > 2:
                if self.latency_threshold:
                    self.latency_count = 0
                    if self.latency != self.latency_min:
                        self.set_latency_timer(self.latency_min)
                        self.latency = self.latency_min
                # skip FTDI status bytes.
                # Maybe stored in the future to enable modem use
                num_of_chunks = actual_length // packet_size
                chunk_remains = actual_length % packet_size
                self.readbuffer_offset += 2
                actual_length -= 2
                count = packet_size - 2
                if actual_length > packet_size - 2:
                    for i in xrange(1, num_of_chunks+1):
                        dstoff = self.readbuffer_offset + count * i
                        srcoff = self.readbuffer_offset + packet_size * i
                        self.readbuffer[dstoff:dstoff+count] = \
                            self.readbuffer[srcoff:srcoff+count]
                    if chunk_remains > 2:
                        i = num_of_chunks+1
                        count = chunk_remains-2
                        dstoff = self.readbuffer_offset + count * i
                        srcoff = self.readbuffer_offset + packet_size * i
                        self.readbuffer[dstoff:dstoff+count] = \
                            self.readbuffer[srcoff:srcoff+count]
                        actual_length -= 2*num_of_chunks
                    else:
                        actual_length -= 2*(num_of_chunks-1)+chunk_remains
            elif actual_length <= 2:
                if self.latency_threshold:
                    self.latency_count += 1
                    if self.latency != self.latency_max:
                        if self.latency_count > self.latency_threshold:
                            self.set_latency_timer(self.latency_max)
                            self.latency = self.latency_max
                # no more data to read?
                return data.tostring()
            if actual_length > 0:
                # data still fits in buf?
                if offset+actual_length <= size:
                    data[offset:offset+actual_length] = \
                        self.readbuffer[self.readbuffer_offset:\
                                        self.readbuffer_offset+actual_length]
                    offset += actual_length
                    # Did we read exactly the right amount of bytes?
                    if offset == size:
                        return data.tostring()
                else:
                    # only copy part of the data or size<=readbuffer_chunksize
                    part_size = size-offset
                    data[offset:offset+part_size] = \
                        self.readbuffer[self.readbuffer_offset:\
                                        self.readbuffer_offset+part_size]
                    self.readbuffer_offset += part_size
                    self.readbuffer_remaining = actual_length-part_size
                    offset += part_size
                    return data.tostring()
        # never reached
        raise AssertionError("Never reached")

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
        if interface == 0:
            interface = 1
        if interface not in xrange(1, 5):
            # should use the actual interface count, depending on the device
            raise ValueError("Interface does not exist")
        self.index = interface
        self.in_ep = 2*interface
        self.out_ep = 0x80 + self.in_ep - 1
        
    def _reset_device(self):
        """Resets the ftdi device"""
        if self.usb_dev.ctrl_transfer(Ftdi.DEV_OUT_REQTYPE,
                                      Ftdi.SIO_RESET_REQUEST, 
                                      Ftdi.SIO_RESET_SIO,
                                      self.index, '', 
                                      self.usb_write_timeout):
            raise FtdiError('Unable to reset FTDI device')
        # Invalidate data in the readbuffer
        self.readbuffer_offset = 0
        self.readbuffer_remaining = 0

    def _get_max_packet_size(self):
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
