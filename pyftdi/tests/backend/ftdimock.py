"""PyUSB virtual FTDI device."""

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

from array import array
from binascii import hexlify
from collections import deque
from logging import getLogger
from pyftdi.tracer import FtdiMpsseTracer
from .consts import FTDICONST, USBCONST

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=unused-argument
#pylint: disable-msg=invalid-name
#pylint: disable-msg=too-many-arguments
#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-few-public-methods
#pylint: disable-msg=no-self-use


class MockMpsseTracer(FtdiMpsseTracer):
    """Reuse MPSSE tracer as a MPSSE command decoder engine.
    """

    def __init__(self):
        super().__init__(self)
        self.log = getLogger('pyftdi.mock.mpsse')


class MockFtdi:
    """Fake FTDI device.
    """

    def __init__(self):
        self.log = getLogger('pyftdi.mock.ftdi')
        self._bitmode = FTDICONST.get_value('bitmode', 'reset')
        self._mpsse = None
        self._direction = 0
        self._gpio = 0
        self._queues = deque(), deque()
        self._status = 0

    def control(self, dev_handle: 'MockDeviceHandle', bmRequestType: int,
                bRequest: int, wValue: int, wIndex: int, data: array,
                timeout: int) -> int:
        req_ctrl = USBCONST.dec_req_ctrl(bmRequestType)
        req_type = USBCONST.dec_req_type(bmRequestType)
        req_rcpt = USBCONST.dec_req_rcpt(bmRequestType)
        req_desc = ':'.join([req_ctrl, req_type, req_rcpt])
        req_name = FTDICONST.dec_req_name(bRequest)
        dstr = (hexlify(data).decode() if USBCONST.is_req_out(bmRequestType)
                else f'({len(data)})')
        self.log.debug('> control ftdi hdl %d, %s, %s, '
                       'val 0x%04x, idx 0x%04x, data %s, to %d',
                       dev_handle.handle, req_desc, req_name,
                       wValue, wIndex, dstr, timeout)
        size = 0
        try:
            handler = getattr(self, f'_control_{req_name}')
        except AttributeError:
            self.log.warning('Unknown request: %s', req_name)
            return size
        buf = handler(wValue, wIndex, data) or b''
        size = len(buf)
        data[:size] = array('B', buf)
        self.log.debug('< (%d) %s', size, hexlify(data[:size]).decode())
        return size

    def write(self, dev_handle: 'MockDeviceHandle', ep: int, intf: int,
              data: array, timeout: int) -> int:
        if self._bitmode == FTDICONST.get_value('bitmode', 'mpsse'):
            self._mpsse.send(data)
            return len(data)
        if self._bitmode == FTDICONST.get_value('bitmode', 'reset'):
            self._queues[0].extend(data)
            return len(data)
        if self._bitmode == FTDICONST.get_value('bitmode', 'bitbang'):
            self._gpio &= ~0xFF
            self._gpio |= data[0] & self._direction
            self.log.info('. %02x: %s', self._gpio, f'{self._gpio:08b}')
            return 1
        mode = FTDICONST.get_name('bitmode', self._bitmode)
        self.log.warning('Write buffer discarded, mode %s', mode)
        self.log.warning('. (%d) %s', len(data), hexlify(data).decode())
        return 0

    def read(self, dev_handle: 'MockDeviceHandle', ep: int, intf: int,
             buff: array, timeout: int) -> int:
        if self._bitmode == FTDICONST.get_value('bitmode', 'reset'):
            count = len(buff)
            if count < 2:
                return 0
            cts = 0x08 if self._gpio & 0x08 else 0
            dsr = 0x04 if self._gpio & 0x20 else 0
            ri = 0x02 if self._gpio & 0x80 else 0
            dcd = 0x01 if self._gpio & 0x40 else 0
            buff[0] = cts | dsr | ri | dcd
            buff[1] = self._status
            pos = 2
            while self._queues[1] and pos < count:
                buff[pos] = self._queues[1].popleft()
                pos += 1
            return pos
        mode = FTDICONST.get_name('bitmode', self._bitmode)
        self.log.debug('Read buffer discarded, mode %s', mode)
        self.log.debug('. (%d)', len(buff))
        return 0

    @property
    def gpio(self) -> int:
        return self._gpio

    def uart_write(self, buffer: bytes) -> None:
        self._queues[1].extend(buffer)

    def uart_read(self, count: int) -> bytes:
        # it might be worth to use pipes here, but for now we do not care
        # about the performance
        buf = bytearray()
        while self._queues[0] and count:
            buf.append(self._queues[0].popleft())
            count -= 1
        return bytes(buf)

    @gpio.setter
    def gpio(self, gpio: int) -> None:
        self._gpio |= gpio & ~self._direction & 0xFFFF

    @property
    def direction(self) -> int:
        return self._direction

    @direction.setter
    def direction(self, direction: int) -> None:
        self._direction = direction & 0xFFFF

    def _control_reset(self, wValue: int, wIndex: int,
                       data: array) -> None:
        reset = FTDICONST.get_name('sio_reset', wValue)
        self.log.info('> ftdi reset %s', reset)

    def _control_set_bitmode(self, wValue: int, wIndex: int,
                             data: array) -> None:
        direction = wValue & 0xff
        bitmode = (wValue >> 8) & 0x7F
        mode = FTDICONST.get_name('bitmode', bitmode)
        self.log.info('> ftdi bitmode %s: %s', mode, f'{direction:08b}')
        self._bitmode = bitmode
        self._direction = direction
        self._mpsse = FtdiMpsseTracer() if mode == 'mpsse' else None

    def _control_set_latency_timer(self, wValue: int, wIndex: int,
                                   data: array) -> None:
        self.log.info('> ftdi latency timer: %d', wValue)

    def _control_set_event_char(self, wValue: int, wIndex: int,
                                data: array) -> None:
        char = wValue & 0xFF
        enable = bool(wValue >> 8)
        self.log.info('> ftdi %sable event char: 0x%02x',
                      'en' if enable else 'dis', char)

    def _control_set_error_char(self, wValue: int, wIndex: int,
                                data: array) -> None:
        char = wValue & 0xFF
        enable = bool(wValue >> 8)
        self.log.info('> ftdi %sable error char: 0x%02x',
                      'en' if enable else 'dis', char)

    def _control_read_pins(self, wValue: int, wIndex: int,
                           data: array) -> bytes:
        self.log.info('> ftdi read_pins')
        low_gpio = self._gpio & 0xFF
        self.log.info('< %02x: %s', low_gpio, f'{low_gpio:08b}')
        return bytes([low_gpio])

    def _control_set_baudrate(self, wValue: int, wIndex: int,
                              data: array) -> None:
        pass

    def _control_set_data(self, wValue: int, wIndex: int,
                          data: array) -> None:
        pass

    def _control_set_flow_ctrl(self, wValue: int, wIndex: int,
                               data: array) -> None:
        pass
