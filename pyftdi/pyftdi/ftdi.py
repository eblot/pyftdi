# pyftdi - A pure Python FTDI driver
# Copyright (C) 2010-2011 Emmanuel Blot <emmanuel.blot@free.fr>
#   Originally based on the C libftdi project
#   http://www.intra2net.com/en/developer/libftdi/
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

"""pyftdi - A pure Python FTDI driver on top of pyusb

Author:  Emmanuel Blot <emmanuel.blot@free.fr>
License: LGPL, originally based on libftdi C library
Caveats: Only tested with FT2232 and FT4232 FTDI devices
Require: pyusb
"""

import os
import struct
import usb.core
import usb.util
from array import array as Array
from usbtools import UsbTools

__all__ = ['Ftdi', 'FtdiError']


class FtdiError(IOError):
    """Communication error with the FTDI device"""


class Ftdi(object):
    """FTDI device driver"""

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
    RW_BYTES_PVE_PVE_LSB = 0x38
    RW_BYTES_PVE_NVE_LSB = 0x39
    RW_BYTES_NVE_PVE_LSB = 0x3c
    RW_BYTES_NVE_NVE_LSB = 0x3d
    RW_BITS_PVE_PVE_LSB = 0x3a
    RW_BITS_PVE_NVE_LSB = 0x3b
    RW_BITS_NVE_PVE_LSB = 0x3e
    RW_BITS_NVE_NVE_LSB = 0x3f
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
    READ_SHORT = 0x90
    READ_EXTENDED = 0x91
    WRITE_SHORT = 0x92
    WRITE_EXTENDED = 0x93

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
    BITMODE_MASK = 0x4F     # Mask for all bitmodes

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

    # cts:  Clear to send
    # dsr:  Data set ready
    # ri:   Ring indicator
    # dcd:  Data carrier detect
    # dr:   Data ready
    # oe:   Overrun error
    # pe:   Parity error
    # fe:   Framing error
    # bi:   Break interrupt
    # thre: Transmitter holding register
    # temt: Transmitter empty
    # err:  Error in RCVR FIFO
    MODEM_STATUS = [('_0 _1 _2 _3 cts dsr ri dcd'.split()),
                    ('dr oe pe fe bi thre temt error'.split())]

    # Clocks and baudrates
    BUS_CLOCK_BASE = 6.0E6 # 6 MHz
    BUS_CLOCK_HIGH = 30.0E6 # 30 MHz
    BAUDRATE_REF_BASE = int(3.0E6)  # 3 MHz
    BAUDRATE_REF_HIGH = int(12.0E6) # 12 MHz
    BAUDRATE_REF_SPECIAL = int(2.0E6)  # 3 MHz
    BAUDRATE_TOLERANCE = 3.0 # acceptable clock drift, in %
    BITBANG_CLOCK_MULTIPLIER = 4

    # Latency
    LATENCY_MIN = 1
    LATENCY_MAX = 255
    LATENCY_THRESHOLD = 1000

    # Special devices
    LEGACY_DEVICES = ('ft232am', )
    EXSPEED_DEVICES = ('ft2232d', )
    HISPEED_DEVICES = ('ft2232h', 'ft4232h')

    def __init__(self):
        self.usb_dev = None
        self.usb_read_timeout = 5000
        self.usb_write_timeout = 5000
        self.baudrate = -1
        self.readbuffer = Array('B')
        self.readoffset = 0
        self.readbuffer_chunksize = 4 << 10 # 4KiB
        self.writebuffer_chunksize = 4 << 10 # 4KiB
        self.max_packet_size = 0
        self.interface = None
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

    @staticmethod
    def find_all(vps):
        """Find all devices that match the vendor/product pairs of the vps
           list."""
        return UsbTools.find_all(vps)

    def open(self, vendor, product, interface, index=0, serial=None,
             description=None):
        """Open a new interface to the specified FTDI device"""
        self.usb_dev = UsbTools.get_device(vendor, product, index, serial,
                                           description)
        # detect invalid interface as early as possible
        config = self.usb_dev.get_active_configuration()
        if interface > config.bNumInterfaces:
            raise FtdiError('No such FTDI port: %d' % interface)
        self._set_interface(config, interface)
        self.max_packet_size = self._get_max_packet_size()
        self._reset_device()
        self.set_latency_timer(self.LATENCY_MIN)

    def close(self):
        """Close the FTDI interface"""
        self.set_latency_timer(self.LATENCY_MAX)
        UsbTools.release_device(self.usb_dev)

    def open_mpsse(self, vendor, product, interface=1,
                   index=0, serial=None, description=None,
                   direction=0x0, initial=0x0, frequency=6.0E6, latency=16):
        """Configure the interface for MPSSE mode"""
        # Open an FTDI interface
        self.open(vendor, product, interface, index, serial, description)
        # Set latency timer
        self.set_latency_timer(latency)
        # Set chunk size
        self.write_data_set_chunksize(512)
        self.read_data_set_chunksize(512)
        # Drain input buffer
        self.purge_buffers()
        # Enable MPSSE mode
        self.set_bitmode(direction, Ftdi.BITMODE_MPSSE)
        # Configure clock
        frequency = self._set_frequency(frequency)
        # Configure I/O
        self.write_data(Array('B', [Ftdi.SET_BITS_LOW, initial, direction]))
        # Disable loopback
        self.write_data(Array('B', [Ftdi.LOOPBACK_END]))
        self.validate_mpsse()
        # Drain input buffer
        self.purge_buffers()
        # Return the actual frequency
        return frequency

    def open_bitbang(self, vendor, product, interface=1,
                     index=0, serial=None, description=None,
                     direction=0x0, baudrate=115200, latency=16):
        """Configure the interface for BITBANG mode"""
        # Open an FTDI interface
        self.open(vendor, product, interface, index, serial, description)
        # Set latency timer
        self.set_latency_timer(latency)
        # Set chunk size
        self.write_data_set_chunksize(512)
        self.read_data_set_chunksize(512)
        # Disable loopback
        self.write_data(Array('B', [Ftdi.LOOPBACK_END]))
        # Enable BITBANG mode
        self.set_bitmode(direction, Ftdi.BITMODE_BITBANG)
        # Configure baudrate
        self.set_baudrate(baudrate)
        self.validate_mpsse()
        # Drain input buffer
        self.purge_buffers()

    @property
    def type(self):
        """Return the current type of the FTDI device as a string"""
        types = { 0x200: 'ft232am',
                  0x400: 'ft232bm', # bug with S/N == 0 not handled
                  0x500: 'ft2232d',
                  0x600: 'ft232c',
                  0x700: 'ft2232h',
                  0x800: 'ft4232h' }
        return types[self.usb_dev.bcdDevice]

    @property
    def bitbang_enabled(self):
        """Tell whether some bitbang mode is activated"""
        return not self.bitbang_mode in [
                Ftdi.BITMODE_RESET,
                Ftdi.BITMODE_CBUS # CBUS mode does not change base frequency
        ]

    @property
    def frequency_max(self):
        """Tells the maximum frequency for MPSSE clock"""
        if self.type in self.HISPEED_DEVICES:
            return Ftdi.BUS_CLOCK_HIGH
        return Ftdi.BUS_CLOCK_BASE

    @property
    def fifo_sizes(self):
        """Return the (TX, RX) tupple of hardware FIFO sizes"""
        # Note that the FTDI datasheets contradict themselves, so
        # the following values may not be the right ones...
        # Note that 'TX' and 'RX' are inverted with the datasheet terminology:
        # Values here are seen from the host perspective, whereas datasheet
        # values are defined from the device perspective
        sizes = { 'ft232c': (128, 256),     # TX: 128, RX: 256
                  'ft2232d': (384, 128),    # TX: 384, RX: 128
                  'ft2232h': (4096, 4096),  # TX: 4KiB, RX: 4KiB
                  'ft4232h': (2048, 2048) } # TX: 2KiB, RX: 2KiB
        return sizes.get(self.type, (128, 128)) # default sizes

    def set_baudrate(self, baudrate):
        """Change the current interface baudrate"""
        if self.bitbang_enabled:
            baudrate *= Ftdi.BITBANG_CLOCK_MULTIPLIER
        actual, value, index = self._convert_baudrate(baudrate)
        delta = 100*abs(float(actual-baudrate))/baudrate
        if delta > Ftdi.BAUDRATE_TOLERANCE:
            raise AssertionError('Baudrate tolerance exceeded: %.02f%% '
                '(wanted %d, achievable %d)' % (delta, baudrate, actual) )
        try:
            if self.usb_dev.ctrl_transfer(Ftdi.REQ_OUT,
                                          Ftdi.SIO_SET_BAUDRATE, value,
                                          index, '', self.usb_write_timeout):
                raise FtdiError('Unable to set baudrate')
            self.baudrate = baudrate
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def set_frequency(self, frequency):
        return self._set_frequency(frequency)

    def purge_rx_buffer(self):
        """Clear the read buffer on the chip and the internal read buffer."""
        if self._ctrl_transfer_out(Ftdi.SIO_RESET, Ftdi.SIO_RESET_PURGE_RX):
            raise FtdiError('Unable to flush RX buffer')
        # Invalidate data in the readbuffer
        self.readoffset = 0
        self.readbuffer = Array('B')

    def purge_tx_buffer(self):
        """Clear the write buffer on the chip."""
        if self._ctrl_transfer_out(Ftdi.SIO_RESET, Ftdi.SIO_RESET_PURGE_TX):
            raise FtdiError('Unable to flush TX buffer')

    def purge_buffers(self):
        """Clear the buffers on the chip and the internal read buffer."""
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
        self.readbuffer = Array('B')
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
        value = (bitmask & 0xff) | ((mode & self.BITMODE_MASK) << 8)
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
        """
        value = self._ctrl_transfer_in(Ftdi.SIO_POLL_MODEM_STATUS, 2)
        if not value or len(value) != 2:
            raise FtdiError('Unable to get modem status')
        status, = struct.unpack('<H', value)
        return status

    def modem_status(self):
        """Provide the current modem status as a tuple of set signals"""
        value = self._ctrl_transfer_in(Ftdi.SIO_POLL_MODEM_STATUS, 2)
        if not value or len(value) != 2:
            raise FtdiError('Unable to get modem status')
        status = []
        for pos, byte_ in enumerate(value):
            for b,v in enumerate(Ftdi.MODEM_STATUS[pos]):
                if byte_ & (1<<b):
                    status.append(Ftdi.MODEM_STATUS[pos][b])
        return tuple(status)

    def set_flowctrl(self, flowctrl):
        """Set flowcontrol for ftdi chip"""
        ctrl = { 'hw' : Ftdi.SIO_RTS_CTS_HS,
                 'sw' : Ftdi.SIO_XON_XOFF_HS,
                 '' : Ftdi.SIO_DISABLE_FLOW_CTRL }
        value = ctrl[flowctrl] | self.index
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
        value |= rts and Ftdi.SIO_SET_RTS_HIGH or Ftdi.SIO_SET_RTS_LOW
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

    def set_line_property(self, bits, stopbit, parity, break_=0):
        """Set (RS232) line characteristics"""
        bytelength = { 7 : Ftdi.BITS_7,
                       8 : Ftdi.BITS_8 }
        parities  = { 'N' : Ftdi.PARITY_NONE,
                      'O' : Ftdi.PARITY_ODD,
                      'E' : Ftdi.PARITY_EVEN,
                      'M' : Ftdi.PARITY_MARK,
                      'S' : Ftdi.PARITY_SPACE }
        stopbits  = { 1 : Ftdi.STOP_BIT_1,
                      1.5 : Ftdi.STOP_BIT_15,
                      2 : Ftdi.STOP_BIT_2 }
        if parity not in parities:
            raise FtdiError("Unsupported parity")
        if bits not in bytelength:
            raise FtdiError("Unsupported byte length")
        if stopbit not in stopbits:
            raise FtdiError("Unsupported stop bits")
        value = bits & 0x0F
        try:
            value |= { Ftdi.PARITY_NONE : 0x00 << 8,
                       Ftdi.PARITY_ODD : 0x01 << 8,
                       Ftdi.PARITY_EVEN : 0x02 << 8,
                       Ftdi.PARITY_MARK : 0x03 << 8,
                       Ftdi.PARITY_SPACE : 0x04 << 8 }[parities[parity]]
            value |= { Ftdi.STOP_BIT_1 : 0x00 << 11,
                       Ftdi.STOP_BIT_15 : 0x01 << 11,
                       Ftdi.STOP_BIT_2 : 0x02 << 11 }[stopbits[stopbit]]
            if break_ == Ftdi.BREAK_ON:
                value |= 0x01 << 14
        except KeyError:
            raise AssertionError('Invalid line property')
        if self._ctrl_transfer_out(Ftdi.SIO_SET_DATA, value):
            raise FtdiError('Unable to set line property')

    def write_data(self, data):
        """Write data in chunks to the chip"""
        offset = 0
        size = len(data)
        try:
            while offset < size:
                write_size = self.writebuffer_chunksize
                if offset + write_size > size:
                    write_size = size - offset
                length = self.usb_dev.write(self.in_ep,
                                            data[offset:offset+write_size],
                                            self.interface,
                                            self.usb_write_timeout)
                if length <= 0:
                    raise FtdiError("Usb bulk write error")
                offset += length
            return offset
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def read_data_bytes(self, size, attempt=1):
        """Read data in chunks from the chip.
           Automatically strips the two modem status bytes transfered during
           every read."""
        # Packet size sanity check
        if not self.max_packet_size:
            raise FtdiError("max_packet_size is bogus")
        packet_size = self.max_packet_size
        length = 1 # initial condition to enter the usb_read loop
        data = Array('B')
        # everything we want is still in the cache?
        if size <= len(self.readbuffer)-self.readoffset:
            data = self.readbuffer[self.readoffset:self.readoffset+size]
            self.readoffset += size
            return data
        # something still in the cache, but not enough to satisfy 'size'?
        if len(self.readbuffer)-self.readoffset != 0:
            data = self.readbuffer[self.readoffset:]
            # end of readbuffer reached
            self.readoffset = len(self.readbuffer)
        # read from USB, filling in the local cache as it is empty
        try:
            while (len(data) < size) and (length > 0):
                while True:
                    tempbuf = self.usb_dev.read(self.out_ep,
                                                self.readbuffer_chunksize,
                                                self.interface,
                                                self.usb_read_timeout)
                    attempt -= 1
                    length = len(tempbuf)
                    # the received buffer contains at least one useful databyte
                    # (first 2 bytes in each packet represent the current modem
                    # status)
                    if length > 2:
                        if self.latency_threshold:
                            self.latency_count = 0
                            if self.latency != self.latency_min:
                                self.set_latency_timer(self.latency_min)
                                self.latency = self.latency_min
                        # skip the status bytes
                        chunks = (length+packet_size-1) // packet_size
                        count = packet_size - 2
                        self.readbuffer = Array('B')
                        self.readoffset = 0
                        srcoff = 2
                        for i in xrange(chunks):
                            self.readbuffer += tempbuf[srcoff:srcoff+count]
                            srcoff += packet_size
                        length = len(self.readbuffer)
                        break
                    else:
                        # received buffer only contains the modem status bytes
                        # no data received, may be late, try again
                        if attempt > 0:
                            continue
                        # no actual data
                        self.readbuffer = Array('B')
                        self.readoffset = 0
                        if self.latency_threshold:
                            self.latency_count += 1
                            if self.latency != self.latency_max:
                                if self.latency_count > self.latency_threshold:
                                    self.set_latency_timer(self.latency_max)
                                    self.latency = self.latency_max
                        # no more data to read?
                        return data
                if length > 0:
                    # data still fits in buf?
                    if (len(data) + length) <= size:
                        data += self.readbuffer[self.readoffset: \
                                                self.readoffset+length]
                        self.readoffset += length
                        # did we read exactly the right amount of bytes?
                        if len(data) == size:
                            return data
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
                        return data
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))
        # never reached
        raise FtdiError("Internal error")

    def read_data(self, size):
        """Read data in chunks from the chip.
           Automatically strips the two modem status bytes transfered during
           every read."""
        return self.read_data_bytes(size).tostring()

    def get_cts(self):
        """Read terminal status line: Clear To Send"""
        status = self.poll_modem_status()
        return (status & self.MODEM_CTS) and True or False

    def get_dsr(self):
        """Read terminal status line: Data Set Ready"""
        status = self.poll_modem_status()
        return (status & self.MODEM_DSR) and True or False

    def get_ri(self):
        """Read terminal status line: Ring Indicator"""
        status = self.poll_modem_status()
        return (status & self.MODEM_RI) and True or False

    def get_cd(self):
        """Read terminal status line: Carrier Detect"""
        status = self.poll_modem_status()
        return (status & self.MODEM_RLSD) and True or False

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

    def validate_mpsse(self):
        # only useful in MPSSE mode
        bytes_ = self.read_data(2)
        if (len(bytes_) >=2 ) and (bytes_[0] == '\xfa'):
            raise FtdiError("Invalid command @ %d" % ord(bytes_[1]))

    def get_error_string(self):
        """Wrapper for libftdi compatibility"""
        return "Unknown error"

    # --- Private implementation -------------------------------------------

    def _set_interface(self, config, ifnum):
        """Select the interface to use on the FTDI device"""
        if ifnum == 0:
            ifnum = 1
        if ifnum-1 not in xrange(config.bNumInterfaces):
            raise ValueError("No such interface for this device")
        self.index = ifnum
        self.interface = config[(ifnum-1, 0)]
        endpoints = sorted([ep.bEndpointAddress for ep in self.interface])
        self.in_ep, self.out_ep = endpoints[:2]

    def _reset_device(self):
        """Reset the ftdi device"""
        if self._ctrl_transfer_out(Ftdi.SIO_RESET, Ftdi.SIO_RESET_SIO):
            raise FtdiError('Unable to reset FTDI device')
        # Invalidate data in the readbuffer
        self.readoffset = 0
        self.readbuffer = Array('B')

    def _ctrl_transfer_out(self, reqtype, value, data=''):
        """Send a control message to the device"""
        try:
            return self.usb_dev.ctrl_transfer(Ftdi.REQ_OUT, reqtype, value,
                                              self.index, data,
                                              self.usb_write_timeout)
        except usb.core.USBError, e:
            raise FtdiError('UsbError: %s' % str(e))

    def _ctrl_transfer_in(self, reqtype, length):
        """Request for a control message from the device"""
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
        if not self.interface:
            raise AssertionError("Interface is not yet known")
        if self.type in self.HISPEED_DEVICES:
            packet_size = 512
        else:
            packet_size = 64
        endpoint = self.interface[0]
        packet_size = endpoint.wMaxPacketSize
        return packet_size

    def _convert_baudrate(self, baudrate):
        """Convert a requested baudrate into the closest possible baudrate
           that can be assigned to the FTDI device"""
        if baudrate < ((2*self.BAUDRATE_REF_BASE)//(2*16384+1)):
            raise AssertionError('Invalid baudrate (too low)')
        if baudrate > self.BAUDRATE_REF_BASE:
            if self.type not in self.HISPEED_DEVICES or \
                baudrate > self.BAUDRATE_REF_HIGH:
                    raise AssertionError('Invalid baudrate (too high)')
            refclock = self.BAUDRATE_REF_HIGH
            hispeed = True
        else:
            refclock = self.BAUDRATE_REF_BASE
            hispeed = False
        # AM legacy device only supports 3 sub-integer dividers, where the
        # other devices supports 8 sub-integer dividers
        am_adjust_up = [0, 0, 0, 1, 0, 3, 2, 1]
        am_adjust_dn = [0, 0, 0, 1, 0, 1, 2, 3]
        # Sub-divider code are not ordered in the natural order
        frac_code = [0, 3, 2, 4, 1, 5, 6, 7]
        divisor = (refclock*8) // baudrate
        if self.type in self.LEGACY_DEVICES:
            # Round down to supported fraction (AM only)
            divisor -= am_adjust_dn[divisor & 7]
        # Try this divisor and the one above it (because division rounds down)
        best_divisor = 0
        best_baud = 0
        best_baud_diff = 0
        for i in xrange(2):
            try_divisor = divisor + i
            if not hispeed:
                # Round up to supported divisor value
                if try_divisor <= 8:
                    # Round up to minimum supported divisor
                    try_divisor = 8
                elif self.type not in self.LEGACY_DEVICES and try_divisor < 12:
                    # BM doesn't support divisors 9 through 11 inclusive
                    try_divisor = 12
                elif divisor < 16:
                    # AM doesn't support divisors 9 through 15 inclusive
                    try_divisor = 16
                else:
                    if self.type in self.LEGACY_DEVICES:
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
            baud_estimate = ((refclock*8) + (try_divisor//2))//try_divisor
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
        if self.type in self.EXSPEED_DEVICES + self.HISPEED_DEVICES:
            index = (encoded_divisor >> 8) & 0xFFFF
            index &= 0xFF00
            index |= self.index
        else:
            index = (encoded_divisor >> 16) & 0xFFFF
        if hispeed:
            index |= 1<<9 # use hispeed mode
        return (best_baud, value, index)

    def _set_frequency(self, frequency):
        """Convert a frequency value into a TCK divisor setting"""
        if frequency > self.frequency_max:
            raise FtdiError("Unsupported frequency: %f" % frequency)
        if frequency <= Ftdi.BUS_CLOCK_BASE:
            divcode = Ftdi.ENABLE_CLK_DIV5
            divisor = int(Ftdi.BUS_CLOCK_BASE/frequency)-1
            actual_freq = Ftdi.BUS_CLOCK_BASE/(divisor+1)
        elif frequency <= Ftdi.BUS_CLOCK_HIGH:
            # not supported on non-H device, however it seems that 2232D
            # devices simply ignore the settings. Could be improved though
            divcode = Ftdi.DISABLE_CLK_DIV5
            divisor = int(Ftdi.BUS_CLOCK_HIGH/frequency)-1
            actual_freq = Ftdi.BUS_CLOCK_HIGH/(divisor+1)
        else:
            raise FtdiError("Unsupported frequency: %f" % frequency)
        # FTDI expects little endian
        if self.type in self.HISPEED_DEVICES:
            cmd = Array('B', [divcode])
        else:
            cmd = Array('B')
        cmd.extend([Ftdi.TCK_DIVISOR, divisor&0xff, (divisor>>8)&0xff])
        self.write_data(cmd)
        self.validate_mpsse()
        # Drain input buffer
        self.purge_rx_buffer()
        return actual_freq

    def __get_timeouts(self):
        return self.usb_read_timeout, self.usb_write_timeout

    def __set_timeouts(self, (read_timeout, write_timeout)):
        self.usb_read_timeout = read_timeout
        self.usb_write_timeout = write_timeout

    timeouts = property(__get_timeouts, __set_timeouts)
