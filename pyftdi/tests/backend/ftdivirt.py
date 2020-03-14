"""PyUSB virtual FTDI device."""

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=unused-argument
#pylint: disable-msg=invalid-name
#pylint: disable-msg=too-many-arguments
#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-many-instance-attributes
#pylint: disable-msg=too-few-public-methods
#pylint: disable-msg=no-self-use

from array import array
from binascii import hexlify
from collections import deque
from logging import getLogger
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from sys import version_info
from typing import Mapping, Optional, Sequence, Tuple
from pyftdi.eeprom import FtdiEeprom   # only for consts, do not use code
from pyftdi.tracer import FtdiMpsseTracer
from .consts import FTDICONST, USBCONST

# need support for f-string syntax
if version_info[:2] < (3, 6):
    raise AssertionError('Python 3.6 is required for this module')


class VirtMpsseTracer(FtdiMpsseTracer):
    """Reuse MPSSE tracer as a MPSSE command decoder engine.
    """

    def __init__(self):
        super().__init__(self)
        self.log = getLogger('pyftdi.virt.mpsse')


class VirtFtdi:
    """Fake FTDI device.

       :param version: FTDI version (device kind)
       :param eeprom_size: size of external EEPROM size, if any
    """

    EXT_EEPROMS: Mapping[str, int] = {
        '93c46': 128,  # 1024 bits
        '93c56': 256,  # 2048 bits
        '93c66': 256,  # 2048 bits  (93C66 seen as 93C56)
    }
    """External EEPROMs."""

    INT_EEPROMS: Mapping[int, int] = {
        0x0600: 0x80,  # FT232R: 128 bytes, 1024 bits
        0x1000: 0x400  # FT23*X: 1KiB
    }
    """Internal EEPROMs."""

    BUS_WIDTHS: Mapping[int, Tuple[int, int]] = {
        0x0200: (8, 0),   # FT232AM
        0x0400: (8, 0),   # FT232BM
        0x0500: (8, 0),   # FT2232D
        0x0600: (8, 5),   # FT232R
        0x0700: (16, 0),  # FT2232H
        0x0800: (8, 0),   # FT4232H
        0x0900: (8, 10),  # FT232H
        0x1000: (8, 4),   # FT231X
    }
    """Width of port/bus (regular, cbus)."""

    def __init__(self, version: int, eeprom: Optional[dict] = None):
        self.log = getLogger('pyftdi.virt.ftdi')
        self._bitmode = FTDICONST.get_value('bitmode', 'reset')
        self._mpsse: Optional[VirtMpsseTracer] = None
        self._direction: int = 0
        self._gpio: int = 0
        self._cbus_dir: int = 0
        self._cbus: int = 0
        self._queues: Tuple[deque, deque] = (deque(), deque())
        self._status: int = 0
        self._version = version
        self._eeprom: bytearray = self._build_eeprom(version, eeprom)
        self._cbus_gpio: int = 0
        self._cbus_force: int = 0
        self._cbus_map: Optional[dict] = None

    def apply_eeprom_config(self, devdesc: dict,
                            cfgdescs: Sequence[dict]) -> None:
        self._load_eeprom(devdesc, cfgdescs)

    def control(self, dev_handle: 'VirtDeviceHandle', bmRequestType: int,
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

    def write(self, dev_handle: 'VirtDeviceHandle', ep: int, intf: int,
              data: array, timeout: int) -> int:
        if self._bitmode == FTDICONST.get_value('bitmode', 'mpsse'):
            self._mpsse.send(data)
            return len(data)
        if self._bitmode == FTDICONST.get_value('bitmode', 'reset'):
            self._queues[0].extend(data)
            return len(data)
        if self._bitmode == FTDICONST.get_value('bitmode', 'bitbang'):
            # only 8 LSBs are addressable through this command
            self._gpio &= ~0xFF
            self._gpio |= data[0] & self._direction
            self.log.info('. %02x: %s', self._gpio, f'{self._gpio:08b}')
            return 1
        mode = FTDICONST.get_name('bitmode', self._bitmode)
        self.log.warning('Write buffer discarded, mode %s', mode)
        self.log.warning('. (%d) %s', len(data), hexlify(data).decode())
        return 0

    def read(self, dev_handle: 'VirtDeviceHandle', ep: int, intf: int,
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

    @property
    def gpio(self) -> int:
        """Emulate GPIO output (from FTDI to peripheral)."""
        return self._gpio

    @gpio.setter
    def gpio(self, gpio: int) -> None:
        """Emulate GPIO input (from peripheral to FTDI)."""
        mask = (1 << self.BUS_WIDTHS[self._version][0]) - 1
        self._gpio |= gpio & ~self._direction & mask

    @property
    def cbus(self) -> int:
        """Emulate CBUS output (from FTDI to peripheral)."""
        return self._cbus_read()

    @cbus.setter
    def cbus(self, cbus: int) -> None:
        """Emulate CBUS input (from peripheral to FTDI)."""
        self._cbus_write(cbus)

    @property
    def eeprom(self) -> bytes:
        return bytes(self._eeprom)

    @eeprom.setter
    def eeprom(self, value: bytes):
        if len(value) != len(self._eeprom):
            raise ValueError('EEPROM size mismatch')
        self._eeprom = bytearray(value)

    @property
    def direction(self) -> int:
        return self._direction

    @classmethod
    def _build_eeprom(cls, version, eeprom: Optional[dict]) -> bytearray:
        size = 0
        data = b''
        if eeprom:
            model = eeprom.get('model', None)
            if model:
                if version in cls.INT_EEPROMS:
                    raise ValueError('No external EEPROM supported on this '
                                     'device')
                try:
                    size = cls.EXT_EEPROMS[model.lower()]
                except KeyError:
                    raise ValueError('Unsupported EEPROM model: {model}')
            data = eeprom.get('data', b'')
        if version in cls.INT_EEPROMS:
            int_size = cls.INT_EEPROMS[version]
            # FT232R, FT230x, FT231x, FT234x
            if size:
                if size != int_size:
                    raise ValueError('Internal EEPROM size cannot be changed')
            else:
                size = int_size
        else:
            if size and size not in cls.EXT_EEPROMS.values():
                raise ValueError(f'Invalid EEPROM size: {size}')
        if data and len(data) > size:
            raise ValueError('Data cannot fit into EEPROM')
        buf = bytearray(size)
        buf[:len(data)] = data
        return buf

    def _checksum_eeprom(self, data: bytearray) -> int:
        length = len(data)
        if length & 0x1:
            raise ValueError('Length not even')
        # NOTE: checksum is computed using 16-bit values in little endian
        # ordering
        checksum = 0XAAAA
        mtp = self._version == 0x1000  # FT230X
        for idx in range(0, length, 2):
            if mtp and 0x24 <= idx < 0x80:
                # special MTP user section which is not considered for the CRC
                continue
            val = ((data[idx+1] << 8) + data[idx]) & 0xffff
            checksum = val ^ checksum
            checksum = ((checksum << 1) & 0xffff) | ((checksum >> 15) & 0xffff)
        return checksum

    def _load_eeprom(self, devdesc: dict, cfgdescs: Sequence[dict]) -> None:
        whole = self._version != 0x1000  # FT230X
        buf = self._eeprom if whole else self._eeprom[:0x100]
        chksum = self._checksum_eeprom(buf)
        if chksum:
            self.log.warning('Invalid EEPROM checksum, ignoring content')
            return
        # only apply a subset of what the EEPROM can configure for now
        devdesc['idVendor'] = sunpack('<H', self._eeprom[2:4])[0]
        devdesc['idProduct'] = sunpack('<H', self._eeprom[4:6])[0]
        devdesc['iManufacturer'] = self._decode_eeprom_string(0x0e)
        devdesc['iProduct'] = self._decode_eeprom_string(0x10)
        devdesc['iSerialNumber'] = self._decode_eeprom_string(0x12)
        for desc in cfgdescs:
            if desc.bConfigurationValue == 0:
                # only update first configuration
                desc['bMaxPower'] = self._eeprom[0x09]
                desc['bmAttributes'] = 0x80 | (self._eeprom[0x08] & 0x0F)
        cbus_dec = f'_decode_cbus_x{self._version:04x}'
        try:
            cbus_func = getattr(self, cbus_dec)
        except AttributeError:
            self.log.debug('No CBUS support: %s', cbus_dec)
            return
        cbus_func()

    def _decode_eeprom_string(self, offset):
        str_offset, str_size = sunpack('<BB', self._eeprom[offset:offset+2])
        if str_size:
            str_size -= scalc('<H')
            str_offset += scalc('<H')
            manufacturer = self._eeprom[str_offset:str_offset+str_size]
            return manufacturer.decode('utf16', errors='ignore')
        return ''

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
        if mode == 'cbus':
            self._cbus_dir = direction >> 4
            mask = (1 << self.BUS_WIDTHS[self._version][1]) - 1
            self._cbus_dir &= mask
            # clear output pins
            self._cbus &= ~self._cbus_dir & 0xF
            # update output pins
            output = direction & 0xF & self._cbus_dir
            self._cbus |= output
            self.log.info('> ftdi cbus %s %s',
                          f'{self._cbus_dir:04b}', f'{self._cbus:04b}')
        else:
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
        mode = FTDICONST.get_name('bitmode', self._bitmode)
        self.log.info('> ftdi read_pins %s', mode)
        if mode == 'cbus':
            cbus = self._cbus & ~self._cbus_dir & 0xF
            self.log.info('< cbus 0x%01x: %s', cbus, f'{cbus:04b}')
            return bytes([cbus])
        low_gpio = self._gpio & 0xFF
        self.log.info('< gpio 0x%02x: %s', low_gpio, f'{low_gpio:08b}')
        return bytes([low_gpio])

    def _control_set_baudrate(self, wValue: int, wIndex: int,
                              data: array) -> None:
        self.log.info('> ftdi set_baudrate')

    def _control_set_data(self, wValue: int, wIndex: int,
                          data: array) -> None:
        self.log.info('> ftdi set_data')

    def _control_set_flow_ctrl(self, wValue: int, wIndex: int,
                               data: array) -> None:
        self.log.info('> ftdi set_flow_ctrl')

    def _control_read_eeprom(self, wValue: int, wIndex: int,
                             data: array) -> Optional[bytes]:
        self.log.debug('> ftdi read_eeprom @ 0x%04x', wIndex*2)
        if not self._eeprom:
            self.log.warning('Missing EEPROM')
            return None
        if len(data) != 2:
            self.log.warning('Unexpected read size: %d', len(data))
            # resume anyway
        address = abs(wIndex * 2)
        if address + 1 > len(self._eeprom):
            # out of bound
            self.log.warning('Invalid EEPROM address: 0x%04x', wValue)
            return None
        word = bytes(self._eeprom[address: address+2])
        return word

    def _control_write_eeprom(self, wValue: int, wIndex: int,
                              data: array) -> None:
        self.log.info('> ftdi write_eeprom @ 0x%04x', wIndex*2)
        if not self._eeprom:
            self.log.warning('Missing EEPROM')
            return
        address = abs(wIndex * 2)
        if address + 1 > len(self._eeprom):
            # out of bound
            self.log.warning('Invalid EEPROM address: 0x%04x', wValue)
            return
        if self._version == 0x1000:
            if 0x80 <= address < 0xA0:
                # those address are R/O on FT230x
                self.log.warning('Protected EEPROM address: 0x%04x', wValue)
                return
        self._eeprom[address: address+2] = spack('<H', wValue)

    def _decode_cbus_x1000(self) -> None:
        cbus_gpio = 0
        cbus_force = 0
        for bix in range(4):
            value = self._eeprom[0x1A + bix]
            if FtdiEeprom.CBUSX(value).name == 'IOMODE':
                cbus_gpio |= 1 << bix
            if FtdiEeprom.CBUSX(value).name == 'DRIVE1':
                cbus_force |= 1 << bix
        self._cbus_gpio = cbus_gpio
        self._cbus_force = cbus_force

    def _decode_cbus_x0900(self) -> None:
        cbus_gpio = 0
        cbus_force = 0
        for bix in range(5):
            value = self._eeprom[0x18 + bix]
            low, high = value & 0x0F, value >> 4
            if FtdiEeprom.CBUSH(low).name == 'IOMODE':
                cbus_gpio |= 1 << (2*bix)
            if FtdiEeprom.CBUSH(high).name == 'IOMODE':
                cbus_gpio |= 1 << ((2*bix) + 1)
            if FtdiEeprom.CBUSH(low).name == 'DRIVE1':
                cbus_force |= 1 << (2*bix)
            if FtdiEeprom.CBUSH(high).name == 'DRIVE1':
                cbus_force |= 1 << ((2*bix) + 1)
        self._cbus_gpio = cbus_gpio
        self._cbus_force = cbus_force
        self._cbus_map = {0: 5, 1: 6, 2: 8, 3: 9}

    def _decode_cbus_x0600(self) -> None:
        cbus_gpio = 0
        bix = 0
        while True:
            value = self._eeprom[0x14 + bix]
            low, high = value & 0x0F, value >> 4
            if FtdiEeprom.CBUS(low).name == 'IOMODE':
                cbus_gpio |= 1 << (2*bix)
            if bix == 2:
                break
            if FtdiEeprom.CBUS(high).name == 'IOMODE':
                cbus_gpio |= 1 << ((2*bix) + 1)
            bix += 1
        self._cbus_gpio = cbus_gpio
        self._cbus_force = 0

    def _cbus_write(self, cbus: int) -> None:
        gpio = cbus & ~self._cbus_dir
        if self._cbus_map:
            # convert logical gpio into physical gpio
            pgpio = 0
            for log, phy in self._cbus_map:
                if gpio & (1 << log):
                    pgpio |= 1 << phy
            gpio = pgpio
        # mask out CBUS pins which are not configured as GPIOs
        gpio &= self._cbus_gpio
        # apply DRIVE1 to gpio
        # High-Z is not supported, so High-Z and zero are considered the same
        gpio |= self._cbus_force
        mask = (1 << self.BUS_WIDTHS[self._version][1]) - 1
        self._cbus = gpio & mask

    def _cbus_read(self) -> int:
        gpio = self._cbus
        # mask out CBUS pins which are not configured as GPIOs
        gpio &= self._cbus_gpio
        if self._cbus_map:
            # convert physical gpio into logical gpio
            lgpio = 0
            for log, phy in self._cbus_map:
                if gpio & (1 << phy):
                    lgpio |= 1 << log
            gpio = lgpio
        return gpio
