# pyprolific - A pure Python PL2303 driver
# Copyright (C) 2011 Emmanuel Blot <emmanuel.blot@free.fr>
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

"""pl2303 - A pure Python PL2303 driver on top of pyusb

Author:  Emmanuel Blot <emmanuel.blot@free.fr>
License: MIT
Require: pyusb
"""

import os
import struct
import threading
import usb.core
import usb.util
from array import array as Array
from pyftdi.usbtools import UsbTools, UsbError

__all__ = ['Prolific', 'ProlificError']

class ProlificError(UsbError):
    """Communication error with the Prolific device"""
    pass


class Prolific(object):
    """PL2303 device driver"""

    SET_LINE_REQUEST = 0x20
    SET_CONTROL_REQUEST = 0x22
    CONTROL_DTR = 0x01
    CONTROL_RTS = 0x02
    BREAK_REQUEST_TYPE = 0x21
    BREAK_REQUEST = 0x23
    BREAK_ON = 0xffff
    BREAK_OFF = 0x0000
    GET_LINE_REQUEST = 0x21
    VENDOR_WRITE_REQUEST = 0x01
    VENDOR_READ_REQUEST = 0x01
    FLUSH_RX_REQUEST = 0x08
    FLUSH_TX_REQUEST = 0x09
    SET_FLOWCONTROL_REQUEST = 0x00
    FLOWCONTROL_HW = 0x41
    FLOWCONTROL_HW_HX = 0x61
    FLOWCONTROL_NONE = 0x00
    UART_STATE = 0x08
    UART_STATE_TRANSIENT_MASK = 0x74
    UART_DCD = 0x01
    UART_DSR = 0x02
    UART_BREAK_ERROR = 0x04
    UART_RING = 0x08
    UART_FRAME_ERROR = 0x10
    UART_PARITY_ERROR = 0x20
    UART_OVERRUN_ERROR = 0x40
    UART_CTS = 0x80

    # USB control requests
    VENDOR_OUT = usb.util.build_request_type(        # 40
                  usb.util.CTRL_OUT,                 # 00
                  usb.util.CTRL_TYPE_VENDOR,         # 40
                  usb.util.CTRL_RECIPIENT_DEVICE)    # 00
    VENDOR_IN = usb.util.build_request_type(         # c0
                  usb.util.CTRL_IN,                  # 80
                  usb.util.CTRL_TYPE_VENDOR,         # 40
                  usb.util.CTRL_RECIPIENT_DEVICE)    # 00
    CTRL_OUT = usb.util.build_request_type(          # 21
                  usb.util.CTRL_OUT,                 # 00
                  usb.util.CTRL_TYPE_CLASS,          # 20
                  usb.util.CTRL_RECIPIENT_INTERFACE) # 01
    CTRL_IN = usb.util.build_request_type(           # a1
                  usb.util.CTRL_IN,                  # 80
                  usb.util.CTRL_TYPE_CLASS,          # 20
                  usb.util.CTRL_RECIPIENT_INTERFACE) # 00

    BAUDRATES = [ 75, 150, 300, 600, 1200, 1800, 2400, 3600,
                  4800, 7200, 9600, 14400, 19200, 28800, 38400,
                  57600, 115200, 230400, 460800, 614400,
                  921600, 1228800 ]
    HX_BAUDRATES = [ 2457600, 3000000, 6000000 ]

    def __init__(self):
        self.usb_dev = None
        self.usb_read_timeout = 1000
        self.usb_write_timeout = 5000
        self.baudrate = -1
        self.readbuffer = Array('B')
        self.readoffset = 0
        self.readbuffer_chunksize = 4 << 10 # 4KB
        self.writebuffer_chunksize = 4 << 10 # 4KB
        self.max_packet_size = 0
        self.interface = None
        self.index = None
        self.in_ep = None
        self.out_ep = None
        self.int_ep = None
        self._type = None
        self._lines = 0x00

    # --- Public API -------------------------------------------------------

    def open(self, vendor, product, interface, index=0, serial=None):
        """Open a new interface to the specified FTDI device"""
        self.usb_dev = UsbTools.get_device(vendor, product, index, serial)
        # detect invalid interface as early as possible
        config = self.usb_dev.get_active_configuration()
        if interface > config.bNumInterfaces:
            raise ProlificError('No such Prolific port: %d' % interface)
        self._set_interface(config, interface)
        self.max_packet_size = self._get_max_packet_size()
        self._set_control_lines(self._lines)
        self._reset_device()
        self._do_black_magic()

    def close(self):
        """Close the FTDI interface"""
        self._reset_device()
        UsbTools.release_device(self.usb_dev)

    @property
    def type(self):
        """Return the current type of the FTDI device as a string"""
        if not self._type:
            device = self.usb_dev
            if device.bDeviceClass == 0x02:
                self._type = 'type0'
            elif device.bMaxPacketSize0 == 0x40:
                self._type = 'hx'
            elif device.bDeviceClass in (0x00, 0xff):
                self._type = 'type1'
            else:
                self._type = 'unknown'
        return self._type

    def set_baudrate(self, baudrate):
        """Change the current interface baudrate"""
        if self.type == 'hx':
            baudrates = self.BAUDRATES + self.HX_BAUDRATES
        else:
            baudrates = self.BAUDRATES
        if baudrate not in baudrates:
            raise ProlificError('Unsupported baudrate: %d' % baudrate)
        data = self._ctrl_in(Prolific.GET_LINE_REQUEST, 0, 0, 7)
        if len(data) != 7:
            raise ProlificError('Invalid line request')
        (prevbr, x, y, z) = struct.unpack('<IBBB', data.tostring())
        if prevbr != baudrate:
            data = Array('B', struct.pack('<IBBB', baudrate, x, y, z))
            self._ctrl_out(Prolific.SET_LINE_REQUEST, 0, 0, data)
        self.baudrate = baudrate

    def purge_rx_buffer(self):
        """Clear the read buffer on the chip and the internal read buffer."""
        # Invalidate data in the readbuffer
        self._vendor_out(Prolific.FLUSH_RX_REQUEST, 0)
        self.readoffset = 0
        self.readbuffer = Array('B')

    def purge_tx_buffer(self):
        """Clear the write buffer on the chip."""
        self._vendor_out(Prolific.FLUSH_TX_REQUEST, 0)

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

    def set_latency_timer(self, latency):
        """Set latency timer"""
        pass

    def get_latency_timer(self):
        """Get latency timer"""
        return 0

    def poll_modem_status(self):
        """Poll modem status information
           This function allows the retrieve the two status bytes of the device.
        """
        # deadlock on the second call, to be investigated, reading from an
        # interrupt endpoint is not yet known
        #data = self.usb_dev.read(self.int_ep, 10, self.interface, 10)
        return 0

    def set_flowctrl(self, flowctrl):
        """Set flowcontrol for ftdi chip"""
        if flowctrl == 'hw':
            self._vendor_out(Prolific.SET_FLOWCONTROL_REQUEST,
                self.type == 'hx' and Prolific.FLOWCONTROL_HW_HX \
                    or Prolific.FLOWCONTROL_HW)
        else:
            self._vendor_out(Prolific.SET_FLOWCONTROL_REQUEST,
                             Prolific.FLOWCONTROL_NONE)

    def set_dtr(self, state):
        """Set dtr line"""
        lines = self._lines
        if state:
            lines |= Prolific.CONTROL_DTR
        else:
            lines &= ~Prolific.CONTROL_DTR
        if self._lines != lines:
            self._lines = lines
            self._set_control_lines(lines)

    def set_rts(self, state):
        """Set rts line"""
        lines = self._lines
        if state:
            lines |= Prolific.CONTROL_RTS
        else:
            lines &= ~Prolific.CONTROL_RTS
        if self._lines != lines:
            self._lines = lines
            self._set_control_lines(lines)

    def set_dtr_rts(self, dtr, rts):
        """Set dtr and rts lines"""
        lines = self._lines
        if dtr:
            lines |= Prolific.CONTROL_DTR
        else:
            lines &= ~Prolific.CONTROL_DTR
        if rts:
            lines |= Prolific.CONTROL_RTS
        else:
            lines &= ~Prolific.CONTROL_RTS
        if self._lines != lines:
            self._lines = lines
            self._set_control_lines(lines)

    def set_line_property(self, bits, stopbits, parity, break_=0):
        """Set (RS232) line characteristics"""
        if bits not in range(5, 9):
            raise ProlificError('Unsupported byte length: %d' % bits)
        try:
            np = { 'N' : 0,
                   'O' : 1,
                   'E' : 2,
                   'M' : 3,
                   'S' : 4}[parity]
        except KeyError:
            raise ProlificError('Unsupported parity: %d' % parity)
        try:
            ns = { 1 : 0,
                   2 : 2,
                   3 : 1 }[stopbits]
        except KeyError:
            raise ProlificError('Unsupported stop bits: %d' % stopbits)
        data = self._ctrl_in(Prolific.GET_LINE_REQUEST, 0, 0, 7)
        if len(data) != 7:
            raise ProlificError('Invalid line request')
        (br, sbits, pbit, blen) = struct.unpack('<IBBB', data.tostring())
        new_data = Array('B', struct.pack('<IBBB', br, ns, np, bits))
        self._ctrl_out(Prolific.SET_LINE_REQUEST, 0, 0, new_data)
        self._reset_device()

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
                    raise ProlificError("Usb bulk write error")
                offset += length
            return offset
        except usb.core.USBError, e:
            raise ProlificError('UsbError: %s' % str(e))

    def read_data_bytes(self, size, attempt=1):
        """Read data in chunks from the chip."""
        # Attempt count is useless with PL2303
        # Packet size sanity check
        if not self.max_packet_size:
            raise ProlificError("max_packet_size is bogus")
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
                try:
                    tempbuf = self.usb_dev.read(self.out_ep,
                                                self.readbuffer_chunksize,
                                                self.interface,
                                                self.usb_read_timeout)
                    length = len(tempbuf)
                except usb.core.USBError:
                    # todo how to differentiate regular timeout from
                    # unexpected issues?
                    length = 0
                # the received buffer contains at least one useful databyte
                if not length:
                    # no actual data
                    self.readbuffer = Array('B')
                    self.readoffset = 0
                    return data
                self.readbuffer = tempbuf
                self.readoffset = 0
                # data still fits in buf?
                if (len(data) + length) <= size:
                    data += self.readbuffer[:length]
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
            raise ProlificError('UsbError: %s' % str(e))
        # never reached
        raise ProlificError("Internal error")

    def read_data(self, size):
        """Read data in chunks from the chip.
           Automatically strips the two modem status bytes transfered during
           every read."""
        return self.read_data_bytes(size).tostring()

    def get_cts(self):
        """Read terminal status line: Clear To Send"""
        status = self.poll_modem_status()
        return 0

    def get_dsr(self):
        """Read terminal status line: Data Set Ready"""
        status = self.poll_modem_status()
        return 0

    def get_ri(self):
        """Read terminal status line: Ring Indicator"""
        status = self.poll_modem_status()
        return 0

    def get_cd(self):
        """Read terminal status line: Carrier Detect"""
        status = self.poll_modem_status()
        return 0

    def get_error_string(self):
        """Wrapper for libftdi compatibility"""
        return "Unknown error"

    # --- Private implementation -------------------------------------------

    def _vendor_out(self, value, index, data=''):
        """Send a vendor message to the device"""
        try:
            return self.usb_dev.ctrl_transfer(Prolific.VENDOR_OUT,
                Prolific.VENDOR_WRITE_REQUEST, value, index, data,
                self.usb_write_timeout)
        except usb.core.USBError, e:
            raise ProlificError('UsbError: %s' % str(e))

    def _vendor_in(self, value, index, length):
        """Request for a vendor message from the device"""
        try:
            return self.usb_dev.ctrl_transfer(Prolific.VENDOR_IN,
                Prolific.VENDOR_READ_REQUEST, value, index, length,
                self.usb_read_timeout)
        except usb.core.USBError, e:
            raise ProlificError('UsbError: %s' % str(e))

    def _ctrl_out(self, req, value, index, data=''):
        """Send a vendor message to the device"""
        try:
            return self.usb_dev.ctrl_transfer(Prolific.CTRL_OUT,
                req, value, index, data, self.usb_write_timeout)
        except usb.core.USBError, e:
            raise ProlificError('UsbError: %s' % str(e))

    def _ctrl_in(self, req, value, index, length):
        """Request for a vendor message from the device"""
        try:
            return self.usb_dev.ctrl_transfer(Prolific.CTRL_IN,
                req, value, index, length, self.usb_read_timeout)
        except usb.core.USBError, e:
            raise ProlificError('UsbError: %s' % str(e))

    def _do_black_magic(self):
        self._vendor_in(0x8484, 0, 1)
        self._vendor_out(0x0404, 0)
        self._vendor_in(0x8484, 0, 1)
        self._vendor_in(0x8383, 0, 1)
        self._vendor_in(0x8484, 0, 1)
        self._vendor_out(0x0404, 1)
        self._vendor_in(0x8484, 0, 1)
        self._vendor_in(0x8383, 0, 1)
        self._vendor_out(0, 1)
        self._vendor_out(1, 0)
        self._vendor_out(2, self.type == 'hx' and 0x44 or 0x22)

    def _set_control_lines(self, value):
        """Control IO lines"""
        try:
            return self._ctrl_out(Prolific.SET_CONTROL_REQUEST, value, 0)
        except usb.core.USBError, e:
            raise ProlificError('UsbError: %s' % str(e))

    def _set_interface(self, config, ifnum):
        """Select the interface to use on the FTDI device"""
        if ifnum not in (0, 1):
            raise ValueError("No such interface for this device")
        self.index = 1
        self.in_ep = 0x02
        self.out_ep = 0x83
        self.int_ep = 0x81
        self.interface = config[(0, 0)]

    def _reset_device(self):
        """Reset the ftdi device"""
        # Invalidate data in the readbuffer
        self.purge_buffers()
        self.readoffset = 0
        self.readbuffer = Array('B')

    def _get_max_packet_size(self):
        """Retrieve the maximum length of a data packet"""
        if not self.usb_dev:
            raise AssertionError("Device is not yet known")
        if not self.interface:
            raise AssertionError("Interface is not yet known")
        for endpoint in self.interface:
            # look for the 'read' endpoint
            if endpoint.bEndpointAddress == self.out_ep:
                packet_size = endpoint.wMaxPacketSize
                return packet_size
        return 0
