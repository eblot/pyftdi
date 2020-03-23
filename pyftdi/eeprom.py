# Copyright (c) 2019-2020, Emmanuel Blot <emmanuel.blot@free.fr>
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

"""EEPROM management for PyFdti"""

import sys
from binascii import hexlify
from collections import OrderedDict, namedtuple
from configparser import ConfigParser
from enum import IntEnum
if sys.version_info[:2] > (3, 5):
    from enum import IntFlag
from logging import getLogger
from random import randint
from re import match
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from typing import BinaryIO, List, Optional, Set, TextIO, Union
if sys.version_info[:2] == (3, 5):
    from aenum import IntFlag
from usb.core import Device as UsbDevice
from .ftdi import Ftdi, FtdiError
from .misc import to_bool, to_int


class FtdiEepromError(FtdiError):
    """FTDI EEPROM error."""


class Hex2Int(int):
    """Hexa representation of a byte."""
    def __str__(self):
        return '0x%02x' % int(self)


class Hex4Int(int):
    """Hexa representation of a half-word."""
    def __str__(self):
        return '0x%04x' % int(self)


class FtdiEeprom:
    """FTDI EEPROM management
    """

    _PROPS = namedtuple('PROPS', 'size user dynoff')
    """Properties for each FTDI device release."""

    _PROPERTIES = {
        0x0200: _PROPS(0, None, 0x94),     # FT232AM
        0x0400: _PROPS(256, 0x14, 0x94),   # FT232BM
        0x0500: _PROPS(256, 0x16, 0x96),   # FT2232D
        0x0600: _PROPS(128, None, 0x98),   # FT232R
        0x0700: _PROPS(256, 0x1A, 0x9A),   # FT2232H
        0x0800: _PROPS(256, 0x1A, 0x9A),   # FT4232H
        0x0900: _PROPS(256, 0x1A, 0xA0),   # FT232H
        0x1000: _PROPS(1024, 0x1A, 0xA0),  # FT230X/FT231X/FT234X
    }
    """EEPROM properties."""


    CBUS = IntEnum('CBUS',
                   'TXDEN PWREN RXLED TXLED TXRXLED SLEEP CLK48 CLK24 CLK12 '
                   'CLK6 IOMODE BB_WR BB_R', start=0)
    """Alternate features for legacy FT232R devices."""

    CBUSH = IntEnum('CBUSH',
                    'TRISTATE TXLED RXLED TXRXLED PWREN SLEEP DRIVE0 DRIVE1 '
                    'IOMODE TXDEN CLK30 CLK15 CLK7_5', start=0)
    """Alternate features for FT232H/FT2232H/FT4232H devices."""

    CBUSX = IntEnum('CBUSX',
                    'TRISTATE TXLED RXLED TXRXLED PWREN SLEEP DRIVE0 DRIVE1 '
                    'IOMODE TXDEN CLK24 CLK12 CLK6 BAT_DETECT BAT_DETECT_NEG '
                    'I2C_TXE I2C_RXF VBUS_SENSE BB_WR BB_RD TIME_STAMP AWAKE',
                    start=0)
    """Alternate features for FT230X devices."""

    INVERT = IntFlag('INVERT', 'TXD RXD RTS CTS DTR DSR DCD RI')
    """Inversion flags for FT232R devices."""

    CHANNEL = IntFlag('CHANNEL', 'FIFO OPTO CPU FT128 RS485')
    """Alternate port mode."""

    DRIVE = IntEnum('DRIVE',
                    'LOW HIGH SLOW_SLEW SCHMITT _10 _20 _40 PWRSAVE_DIS')
    """Driver options for I/O pins."""

    CFG1 = IntFlag('CFG1', 'CLK_IDLE_STATE DATA_LSB FLOW_CONTROL _08 '
                           'HIGH_CURRENTDRIVE _20 _40 SUSPEND_DBUS7')
    """Configuration bits stored @ 0x01."""

    VAR_STRINGS = ('manufacturer', 'product', 'serial')
    """EEPROM strings with variable length."""

    def __init__(self):
        self.log = getLogger('pyftdi.eeprom')
        self._ftdi = Ftdi()
        self._eeprom = bytearray()
        self._dev_ver = 0
        self._valid = False
        self._config = OrderedDict()
        self._dirty = set()

    def __getattr__(self, name):
        if name in self._config:
            return self._config[name]
        raise AttributeError('No such attribute: %s' % name)

    def open(self, device: Union[str, UsbDevice],
             ignore: bool = False) -> None:
        """Open a new connection to the FTDI USB device.

           :param device: the device URL or a USB device instance.
           :param ignore: whether to ignore existing content
        """
        if self._ftdi.is_connected:
            raise FtdiError('Already open')
        if isinstance(device, str):
            self._ftdi.open_from_url(device)
        else:
            self._ftdi.open_from_device(device)
        if not ignore:
            self._read_eeprom()
            if self._valid:
                self._decode_eeprom()

    def close(self) -> None:
        """Close the current connection to the FTDI USB device,
        """
        if self._ftdi.is_connected:
            self._ftdi.close()
            self._eeprom = bytearray()
            self._dev_ver = 0
            self._config.clear()

    def connect(self, ftdi: Ftdi, ignore: bool = False) -> None:
        """Connect a FTDI EEPROM to an existing Ftdi instance.

           :param ftdi: the Ftdi instance to use
           :param ignore: whether to ignore existing content
        """
        self._ftdi = ftdi
        self._eeprom = bytearray()
        self._dev_ver = 0
        self._valid = False
        self._config = OrderedDict()
        self._dirty = set()
        if not ignore:
            self._read_eeprom()
            if self._valid:
                self._decode_eeprom()

    @property
    def device_version(self) -> int:
        """Report the version of the FTDI device.

           :return: the release
        """
        if not self._dev_ver:
            if not self._ftdi.is_connected:
                raise FtdiError('Not connected')
            self._dev_ver = self._ftdi.device_version
        return self._dev_ver

    @property
    def size(self) -> int:
        """Report the EEPROM size.

           The physical EEPROM size may be greater.

           :return: the size in bytes
        """
        try:
            eeprom_size = self._PROPERTIES[self.device_version].size
        except (AttributeError, KeyError):
            raise FtdiError('No EEPROM')
        return eeprom_size

    @property
    def data(self) -> bytes:
        """Returns the content of the EEPROM.

           :return: the content as bytes.
        """
        self._sync_eeprom()
        return bytes(self._eeprom)

    @property
    def properties(self) ->  Set[str]:
        """Returns the supported properties for the current device.

           :return: the supported properies.
        """
        props = set(self._config.keys())
        props -= set(self.VAR_STRINGS)
        return props

    @property
    def is_empty(self) -> bool:
        """Reports whether the EEPROM has been erased, or no EEPROM is
           connected to the FTDI EEPROM port.

           :return: True if no content is detected
        """
        if len(self._eeprom) != self._PROPERTIES[self.device_version].size:
            return False
        for byte in self._eeprom:
            if byte != 0xFF:
                return False
        return True

    @property
    def cbus_pins(self) -> List[int]:
        """Return the list of CBUS pins configured as GPIO, if any

           :return: list of CBUS pins
        """
        pins = [pin for pin in range(0, 10)
                if self._config.get('cbus_func_%d' % pin, '') == 'IOMODE']
        return pins

    @property
    def cbus_mask(self) -> int:
        """Return the bitmask of CBUS pins configured as GPIO.

           The bitmap contains four bits, ordered in natural order.

           :return: CBUS mask
        """
        if self.device_version == 0x900:  # FT232H
            cbus = [5, 6, 8, 9]
        else:
            cbus = list(range(4))
        mask = 0
        for bix, pin in enumerate(cbus):
            if self._config.get('cbus_func_%d' % pin, '') == 'IOMODE':
                mask |= 1 << bix
        return mask

    def save_config(self, file: TextIO) -> None:
        """Save the EEPROM content as an INI stream.

           :param file: output stream
        """
        self._sync_eeprom()
        cfg = ConfigParser()
        cfg.add_section('values')
        for name, value in self._config.items():
            cfg.set('values', name, str(value))
        cfg.add_section('raw')
        length = 16
        for i in range(0, len(self._eeprom), length):
            chunk = self._eeprom[i:i+length]
            hexa = hexlify(chunk).decode()
            cfg.set('raw', '@%02x' % i, hexa)
        cfg.write(file)

    def set_serial_number(self, serial: str) -> None:
        """Define a new serial number."""
        self._validate_string(serial)
        self._update_var_string('serial', serial)
        self.set_property('has_serial', True)

    def set_manufacturer_name(self, manufacturer: str) -> None:
        """Define a new manufacturer string."""
        self._validate_string(manufacturer)
        self._update_var_string('manufacturer', manufacturer)

    def set_product_name(self, product: str) -> None:
        """Define a new product name."""
        self._validate_string(product)
        self._update_var_string('product', product)

    def set_property(self, name: str, value: Union[str, int, bool],
                     out: Optional[TextIO] = None) -> None:
        """Change the value of a stored property.

           :see: :py:meth:`properties` for a list of valid property names.
                 Note that for now, only a small subset of properties can be
                 changed.
           :param name: the property to change
           :param value: the new value (supported values depend on property)
           :param out: optional output stream to report hints
        """
        mobj = match(r'cbus_func_(\d)', name)
        if mobj:
            if not isinstance(value, str):
                raise ValueError("'%s' should be specified as a string" % name)
            self._set_cbus_func(int(mobj.group(1)), value, out)
            self._dirty.add(name)
            return
        hwords = {
            'vendor_id': 0x02,
            'product_id': 0x04,
            'type': 0x06,
            'usb_version': 0x0c
        }
        if name in hwords:
            val = to_int(value)
            if not 0 <= val <= 0xFFFF:
                raise ValueError('Invalid value for %s' % name)
            offset = hwords[name]
            self._eeprom[offset:offset+2] = spack('<H', val)
            return
        confs = {
            'remote_wakeup': (0, 5),
            'self_powered': (0, 6),
            'in_isochronous': (2, 0),
            'out_isochronous': (2, 1),
            'suspend_pull_down': (2, 2),
            'has_serial': (2, 3),
            'has_usb_version': (2, 4),
        }
        if name in confs:
            val = to_bool(value, permissive=False, allow_int=True)
            offset, bit = confs[name]
            mask = 1 << bit
            if val:
                self._eeprom[0x08+offset] |= mask
            else:
                self._eeprom[0x0a+offset] &= ~mask
            return
        if name == 'power_max':
            val = to_int(value) >> 1
            self._eeprom[0x09] = val
            return
        if name in self.properties:
            raise NotImplementedError("Change to '%s' is not yet supported" %
                                      name)
        raise ValueError("Unknown property '%s'" % name)

    def erase(self) -> None:
        """Erase the whole EEPROM."""
        self._eeprom = bytearray([0x00] * self.size)
        self._config.clear()

    def initialize(self) -> None:
        """Initialize the EEPROM with some default sensible values.
        """
        dev_ver = self.device_version
        dev_name = Ftdi.DEVICE_NAMES[dev_ver]
        vid = Ftdi.FTDI_VENDOR
        pid = Ftdi.PRODUCT_IDS[vid][dev_name]
        self.set_manufacturer_name('FTDI')
        self.set_product_name(dev_name.upper())
        sernum = ''.join([chr(randint(ord('A'), ord('Z'))) for _ in range(5)])
        self.set_serial_number('FT%d%s' % (randint(0, 9), sernum))
        self.set_property('vendor_id', vid)
        self.set_property('product_id', pid)
        self.set_property('type', dev_ver)
        self.set_property('usb_version', 0x200)
        self.set_property('power_max', 150)
        self._sync_eeprom()

    def sync(self) -> None:
        """Force re-evaluation of configuration after some changes.

           This API is not useful for regular usage, but might help for testing
           when the EEPROM does not go through a full save/load cycle
        """
        self._sync_eeprom()

    def dump_config(self, file: Optional[BinaryIO] = None) -> None:
        """Dump the configuration to a file.

           :param file: the output file, default to stdout
        """
        if self._dirty:
            self._decode_eeprom()
        for name, value in self._config.items():
            print('%s: %s' % (name, value), file=file or sys.stdout)

    def commit(self, dry_run: bool = True) -> bool:
        """Commit any changes to the EEPROM.

           :param dry_run: log what should be written, do not actually
                           change the EEPROM content

           :return: True if some changes have been committed to the EEPROM
        """
        if not self._dirty:
            self.log.info('No change to commit')
            return False
        self.log.info('Changes to commit: %s', ', '.join(sorted(self._dirty)))
        self._sync_eeprom()
        self._ftdi.overwrite_eeprom(self._eeprom, dry_run=dry_run)
        return dry_run

    @classmethod
    def _validate_string(cls, string):
        for invchr in ':/':
            # do not accept characters which are interpreted as URL seperators
            if invchr in string:
                raise ValueError("Invalid character '%s' in string" % invchr)

    def _update_var_string(self, name: str, value: str) -> None:
        if name not in self.VAR_STRINGS:
            raise ValueError('%s is not a variable string' % name)
        try:
            if value == self._config[name]:
                return
        except KeyError:
            # not yet defined
            pass
        self._config[name] = value
        self._dirty.add(name)

    def _generate_var_strings(self, fill=True) -> None:
        stream = bytearray()
        dynpos = self._PROPERTIES[self.device_version].dynoff
        data_pos = dynpos
        tbl_pos = 0x0e
        for name in self.VAR_STRINGS:
            try:
                ustr = self._config[name].encode('utf-16le')
            except KeyError:
                ustr = ''
            length = len(ustr)+2
            stream.append(length)
            stream.append(0x03)  # string descriptor
            stream.extend(ustr)
            self._eeprom[tbl_pos] = data_pos
            tbl_pos += 1
            self._eeprom[tbl_pos] = length
            tbl_pos += 1
            data_pos += length
        self._eeprom[dynpos:dynpos+len(stream)] = stream
        crc_size = scalc('<H')
        if fill:
            mtp = self._ftdi.device_version == 0x1000
            crc_pos = 0x100 if mtp else len(self._eeprom)
            crc_pos -= crc_size
            rem = len(self._eeprom) - (dynpos + len(stream)) - crc_size
            self._eeprom[dynpos+len(stream):crc_pos] = bytes(rem)

    def _sync_eeprom(self):
        if not self._dirty:
            return
        if any([x in self._dirty for x in self.VAR_STRINGS]):
            self._generate_var_strings()
            for varstr in self.VAR_STRINGS:
                self._dirty.discard(varstr)
        self._update_crc()
        self._decode_eeprom()
        self._dirty.clear()

    def _compute_crc(self, check=False):
        mtp = self._ftdi.device_version == 0x1000
        crc_pos = 0x100 if mtp else len(self._eeprom)
        crc_size = scalc('<H')
        if not check:
            # check mode: add CRC itself, so that result should be zero
            crc_pos -= crc_size
        crc = self._ftdi.calc_eeprom_checksum(self._eeprom[:crc_pos])
        return crc, crc_pos, crc_size

    def _update_crc(self):
        crc, crc_pos, crc_size = self._compute_crc()
        self._eeprom[crc_pos:crc_pos+crc_size] = spack('<H', crc)

    def _read_eeprom(self):
        buf = self._ftdi.read_eeprom(0, eeprom_size=self.size)
        self._eeprom = bytearray(buf)
        crc = self._compute_crc(True)[0]
        if crc:
            if self.is_empty:
                self.log.info('No EEPROM or EEPROM erased')
            else:
                self.log.error('Invalid CRC or EEPROM content')
        self._valid = not bool(crc)

    def _decode_eeprom(self):
        cfg = self._config
        cfg.clear()
        cfg['vendor_id'] = Hex4Int(sunpack('<H', self._eeprom[0x02:0x04])[0])
        cfg['product_id'] = Hex4Int(sunpack('<H', self._eeprom[0x04:0x06])[0])
        cfg['type'] = Hex4Int(sunpack('<H', self._eeprom[0x06:0x08])[0])
        power_supply, power_max, conf = sunpack('<3B', self._eeprom[0x08:0x0b])
        cfg['self_powered'] = bool(power_supply & (1 << 6))
        cfg['remote_wakeup'] = bool(power_supply & (1 << 5))
        cfg['power_max'] = power_max << 1
        cfg['has_usb_version'] = bool(conf & (1 << 4))
        cfg['has_serial'] = bool(conf & (1 << 3))
        cfg['suspend_pull_down'] = bool(conf & (1 << 2))
        cfg['out_isochronous'] = bool(conf & (1 << 1))
        cfg['in_isochronous'] = bool(conf & (1 << 0))
        cfg['usb_version'] = Hex4Int(sunpack('<H', self._eeprom[0x0c:0x0e])[0])
        cfg['manufacturer'] = self._decode_string(0x0e)
        cfg['product'] = self._decode_string(0x10)
        cfg['serial'] = self._decode_string(0x12)
        try:
            name = Ftdi.DEVICE_NAMES[cfg['type']]
            func = getattr(self, '_decode_%s' % name[2:])
        except (KeyError, AttributeError):
            pass
        else:
            func()

    def _decode_string(self, offset):
        str_offset, str_size = sunpack('<BB', self._eeprom[offset:offset+2])
        if str_size:
            str_offset &= self.size - 1
            str_size -= scalc('<H')
            str_offset += scalc('<H')
            manufacturer = self._eeprom[str_offset:str_offset+str_size]
            return manufacturer.decode('utf16', errors='ignore')
        return ''

    def _set_cbus_func(self, cpin: int, value: str,
                       out: Optional[TextIO]) -> None:
        cmap = {0x600: (self.CBUS, 5, 0x14, 4),    # FT232R
                0x900: (self.CBUSH, 10, 0x18, 4),  # FT232H
                0x1000: (self.CBUSX, 4, 0x1A, 8)}  # FT230X/FT231X/FT234X
        try:
            cbus, count, offset, width = cmap[self.device_version]
        except KeyError:
            raise ValueError('This property is not supported on this device')
        if value == '?' and out:
            print(', '.join(sorted([item.name for item in cbus])), file=out)
            return
        if not 0 <= cpin < count:
            raise ValueError("Unsupported CBUS pin '%d'" % cpin)
        try:
            code = cbus[value.upper()]
        except KeyError:
            raise ValueError("CBUS pin %d does not have function '%s'" %
                             (cpin, value))
        addr = offset + (cpin*width)//8
        if width == 4:
            bitoff = 4 if cpin & 0x1 else 0
            mask = 0x0F << bitoff
        else:
            bitoff = 0
            mask = 0xFF
        old = self._eeprom[addr]
        self._eeprom[addr] &= ~mask
        self._eeprom[addr] |= code << bitoff
        self.log.debug('Cpin %d, addr 0x%02x, value 0x%02x->0x%02x',
                       cpin, addr, old, self._eeprom[addr])

    def _decode_230x(self):
        cfg = self._config
        misc, = sunpack('<H', self._eeprom[0x00:0x02])
        cfg['channel_a_driver'] = 'VCP' if misc & (1 << 7) else 'D2XX'
        for bit in self.INVERT:
            value = self._eeprom[0x0B]
            cfg['invert_%s' % self.INVERT(bit).name] = bool(value & bit)
        max_drive = self.DRIVE.LOW | self.DRIVE.HIGH
        value = self._eeprom[0x0c]
        for grp in range(2):
            conf = value &0xF
            cfg['group_%d_drive' % grp] = bool((conf & max_drive) == max_drive)
            cfg['group_%d_schmitt' % grp] = conf & self.DRIVE.SCHMITT
            cfg['group_%d_slew' % grp] = conf & self.DRIVE.SLOW_SLEW
            value >>= 4
        for bix in range(4):
            value = self._eeprom[0x1A + bix]
            cfg['cbus_func_%d' % bix] = self.CBUSX(value).name
        cfg['chip'] = Hex2Int(self._eeprom[0x1E])

    def _decode_232h(self):
        cfg = self._config
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_a_type'] = cfg0 & 0x0F
        cfg['channel_a_driver'] = 'VCP' if (cfg0 & (1 << 4)) else 'D2XX'
        cfg['clock_polarity'] = 'high' if (cfg1 & self.CFG1.CLK_IDLE_STATE) \
                                else 'low'
        cfg['lsb_data'] = bool(cfg1 & self.CFG1.DATA_LSB)
        cfg['flow_control'] = 'on' if (cfg1 & self.CFG1.FLOW_CONTROL) \
                              else 'off'
        cfg['powersave'] = bool(cfg1 & self.DRIVE.PWRSAVE_DIS)
        max_drive = self.DRIVE.LOW | self.DRIVE.HIGH
        for grp in range(2):
            conf = self._eeprom[0x0c+grp]
            cfg['group_%d_drive' % grp] = bool((conf & max_drive) == max_drive)
            cfg['group_%d_schmitt' % grp] = conf & self.DRIVE.SCHMITT
            cfg['group_%d_slew' % grp] = conf & self.DRIVE.SLOW_SLEW
        for bix in range(5):
            value = self._eeprom[0x18 + bix]
            low, high = value & 0x0F, value >> 4
            cfg['cbus_func_%d' % ((2*bix)+0)] = self.CBUSH(low).name
            cfg['cbus_func_%d' % ((2*bix)+1)] = self.CBUSH(high).name
        cfg['chip'] = Hex2Int(self._eeprom[0x1E])

    def _decode_232r(self):
        cfg = self._config
        cfg0 = self._eeprom[0x00]
        cfg['channel_a_driver'] = 'VCP' if (~cfg0 & (1 << 3)) else ''
        cfg['high_current'] = bool(~cfg0 & (1 << 2))
        cfg['external_oscillator'] = cfg0 & 0x02
        for bit in self.INVERT:
            value = self._eeprom[0x0B]
            cfg['invert_%s' % self.INVERT(bit).name] = bool(value & bit)
        bix = 0
        while True:
            value = self._eeprom[0x14 + bix]
            low, high = value & 0x0F, value >> 4
            cfg['cbus_func_%d' % ((2*bix)+0)] = self.CBUS(low).name
            if bix == 2:
                break
            cfg['cbus_func_%d' % ((2*bix)+1)] = self.CBUS(high).name
            bix += 1

    def _decode_2232h(self):
        cfg = self._config
        self._decode_x232h(cfg)
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_a_type'] = self.CHANNEL(cfg0 & 0x7).name or 'UART'
        cfg['channel_b_type'] = self.CHANNEL(cfg1 & 0x7).name or 'UART'
        cfg['suspend_dbus7'] = cfg1 & self.CFG1.SUSPEND_DBUS7

    def _decode_4232h(self):
        cfg = self._config
        self._decode_x232h(cfg)
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_c_driver'] = 'VCP' if ((cfg0 >> 4) & (1 << 3)) else 'D2XX'
        cfg['channel_d_driver'] = 'VCP' if ((cfg1 >> 4) & (1 << 3)) else 'D2XX'
        conf = self._eeprom[0x0B]
        rs485 = self.CHANNEL.RS485
        for chix in range(4):
            cfg['channel_%x_rs485' % (0xa+chix)] = bool(conf & (rs485 << chix))

    def _decode_x232h(self, cfg):
        # common code for2232h and 4232h
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_a_driver'] = 'VCP' if (cfg0 & (1 << 3)) else 'D2XX'
        cfg['channel_b_driver'] = 'VCP' if (cfg1 & (1 << 3)) else 'D2XX'
        max_drive = self.DRIVE.LOW | self.DRIVE.HIGH
        for bix in range(4):
            if not bix & 1:
                val = self._eeprom[0x0c + bix//2]
            else:
                val >>= 4
            cfg['group_%d_drive' % bix] = bool(val & max_drive)
            cfg['group_%d_schmitt' % bix] = bool(val & self.DRIVE.SCHMITT)
            cfg['group_%d_slew' % bix] = bool(val & self.DRIVE.SLOW_SLEW)
        cfg['chip'] = Hex2Int(self._eeprom[0x18])
