from binascii import hexlify
from collections import OrderedDict, namedtuple
from configparser import ConfigParser
from enum import IntEnum, IntFlag
from logging import getLogger
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from sys import stdout
from typing import BinaryIO, Optional, TextIO, Union
from usb.core import Device as UsbDevice

from .ftdi import Ftdi, FtdiError

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
        0x0200: _PROPS(0, None, 0x94),    # FT232AM
        0x0400: _PROPS(256, 0x14, 0x94),  # FT232BM
        0x0500: _PROPS(256, 0x16, 0x96),  # FT2232D
        0x0600: _PROPS(128, None, 0x98),  # FT232R
        0x0700: _PROPS(256, 0x1A, 0x9A),  # FT2232H
        0x0800: _PROPS(256, 0x1A, 0x9A),  # FT4232H
        0x0900: _PROPS(256, 0x1A, 0xA0),  # FT232H
        0x1000: _PROPS(256, 0x1A, 0xA0),  # FT230X
    }
    """EEPROM properties."""


    _CBUS = IntEnum('CBUS',
                    'TXDEN PWREN RXLED TXLED TXRXLED SLEEP CLK48 CLK24 CLK12 '
                    'CLK6 IOMODE BB_WR BB_R', start=0)
    """Alternate features for legacy FT232R devices."""

    _CBUSH = IntEnum('CBUSH',
                     'TRISTATE TXLED RXLED TXRXLED PWREN SLEEP DRIVE0 DRIVE1 '
                     'IOMODE TXDEN CLK30 CLK15 CLK7_5', start=0)
    """Alternate features for FT232H/FT2232H/FT4232H devices."""

    _CBUSX = IntEnum('CBUSX',
                     'TRISTATE TXLED RXLED TXRXLED PWREN SLEEP DRIVE0 DRIVE1 '
                     'IOMODE TXDEN CLK24 CLK12 CLK6 BAT_DETECT BAT_DETECT_NEG '
                     'I2C_TXE I2C_RXF VBUS_SENSE BB_WR BB_RD TIME_STAMP AWAKE',
                     start=0)
    """Alternate features for FT230X devices."""

    _INVERT = IntFlag('INVERT', 'TXD RXD RTS CTS DTR DSR DCD RI')
    """Inversion flags for FT232R devices."""

    _CHANNEL = IntFlag('CHANNEL', 'FIFO OPTO CPU FT128 RS485')
    """Alternate port mode."""

    _DRIVE = IntEnum('DRIVE',
                     'LOW HIGH SLOW_SLEW SCHMITT _10 _20 _40 PWRSAVE_DIS')
    """Driver options for I/O pins."""

    _CFG1 = IntFlag('CFG1', 'CLK_IDLE_STATE DATA_LSB FLOW_CONTROL _08 '
                            'HIGH_CURRENT_DRIVE _20 _40 SUSPEND_DBUS7')
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

    def open(self, device: Union[str, UsbDevice]) -> None:
        """Open a new connection to the FTDI USB device.

           :param device: the device URL or a USB device instance.
        """
        if self._ftdi.is_connected:
            raise FtdiError('Already open')
        if isinstance(device, str):
            self._ftdi.open_from_url(device)
        else:
            self._ftdi.open_from_device(device)
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
        return bytes(self._eeprom)

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

    def save_config(self, file: TextIO) -> None:
        """Save the EEPROM content as an INI stream.

           :param file: output stream
        """
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
        self._update_var_string('serial', serial)

    def set_manufacturer_name(self, manufacturer: str) -> None:
        """Define a new manufacturer string."""
        self._update_var_string('manufacturer', manufacturer)

    def set_product_name(self, product: str) -> None:
        """Define a new product name."""
        self._update_var_string('product', product)

    def erase(self) -> None:
        """Erase the whole EEPROM."""
        self._eeprom = bytearray([0xFF] * self.size)
        self._config.clear()

    def dump_config(self, file: Optional[BinaryIO] = None) -> None:
        """Dump the configuration to a file.

           :param file: the output file, default to stdout
        """
        for name, value in self._config.items():
            print(f' {name}: {value}', file=file or stdout)

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
        if any([x in self._dirty for x in self.VAR_STRINGS]):
            self._generate_var_strings()
        self._ftdi.overwrite_eeprom(self._eeprom, dry_run=dry_run)
        return dry_run

    def _update_var_string(self, name: str, value: str) -> None:
        if name not in self.VAR_STRINGS:
            raise ValueError('%s is not a variable string' % name)
        if value == self._config[name]:
            return
        self._config[name] = value
        self._dirty.add(name)

    def _generate_var_strings(self, fill=True) -> None:
        stream = bytearray()
        dynpos = self._PROPERTIES[self.device_version].dynoff
        data_pos = dynpos
        tbl_pos = 0x0e
        for name in self.VAR_STRINGS:
            ustr = self._config[name].encode('utf-16le')
            length = len(ustr)+2
            stream.append(length)
            stream.append(0x03)  # no idea what this constant means
            stream.extend(ustr)
            self._eeprom[tbl_pos] = data_pos
            tbl_pos += 1
            self._eeprom[tbl_pos] = length
            tbl_pos += 1
            data_pos += length
        self._eeprom[dynpos:dynpos+len(stream)] = stream
        crc_size = scalc('<H')
        if fill:
            rem = len(self._eeprom) - (dynpos + len(stream)) - crc_size
            self._eeprom[dynpos+len(stream):-crc_size] = bytes(rem)
        crc = self._ftdi.calc_eeprom_checksum(self._eeprom[:-crc_size])
        self._eeprom[-crc_size:] = spack('<H', crc)

    def _read_eeprom(self):
        self._eeprom = bytearray(self._ftdi.read_eeprom(0,
                                                        eeprom_size=self.size))
        crc = self._ftdi.calc_eeprom_checksum(self._eeprom)
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

    def _decode_230x(self):
        cfg = self._config
        cfg['channel_a_driver'] = 'VCP'
        for bit in self.INVERT:
            value = self._eeprom[0x0B]
            cfg['invert_%s' % self.INVERT(bit).name] = bool(value & bit)
        max_drive = self._DRIVE.LOW | self._DRIVE.HIGH
        value = self._eeprom[0x0c]
        for grp in range(2):
            conf = value &0xF
            cfg['group_%d_drive' % grp] = bool((conf & max_drive) == max_drive)
            cfg['group_%d_schmitt' % grp] = conf & self._DRIVE.SCHMITT
            cfg['group_%d_slew' % grp] = conf & self._DRIVE.SLOW_SLEW
            value >>= 4
        for bix in range(4):
            value = self._eeprom[0x1A + bix]
            cfg['cbus_func_%d' % bix] = self._CBUSX(value).name
        cfg['chip'] = Hex2Int(self._eeprom[0x1E])

    def _decode_232h(self):
        cfg = self._config
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_a_type'] = cfg0 & 0x0F
        cfg['channel_a_driver'] = 'VCP' if (cfg0 & (1 << 4)) else ''
        cfg['clock_polarity'] = 'high' if (cfg1 & self._CFG1.CLK_IDLE_STATE) \
                                else 'low'
        cfg['lsb_data'] = bool(cfg1 & self._CFG1.DATA_LSB)
        cfg['flow_control'] = 'on' if (cfg1 & self._CFG1.FLOW_CONTROL) \
                              else 'off'
        cfg['powersave'] = bool(cfg1 & self._DRIVE.PWRSAVE_DIS)
        max_drive = self._DRIVE.LOW | self._DRIVE.HIGH
        for grp in range(2):
            conf = self._eeprom[0x0c+grp]
            cfg['group_%d_drive' % grp] = bool((conf & max_drive) == max_drive)
            cfg['group_%d_schmitt' % grp] = conf & self._DRIVE.SCHMITT
            cfg['group_%d_slew' % grp] = conf & self._DRIVE.SLOW_SLEW
        for bix in range(5):
            value = self._eeprom[0x18 + bix]
            low, high = value & 0x0F, value >> 4
            cfg['cbus_func_%d' % ((2*bix)+0)] = self._CBUSH(low).name
            cfg['cbus_func_%d' % ((2*bix)+1)] = self._CBUSH(high).name
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
            cfg['cbus_func_%d' % ((2*bix)+0)] = self._CBUS(low).name
            if bix == 2:
                break
            cfg['cbus_func_%d' % ((2*bix)+1)] = self._CBUS(high).name
            bix += 1

    def _decode_2232h(self):
        cfg = self._config
        self._decode_x232h(cfg)
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_a_type'] = self._CHANNEL(cfg0 & 0x7).name or 'UART'
        cfg['channel_b_type'] = self._CHANNEL(cfg1 & 0x7).name or 'UART'
        cfg['suspend_dbus7'] = cfg1 & self._CFG1.SUSPEND_DBUS7

    def _decode_4232h(self):
        cfg = self._config
        self._decode_x232h(cfg)
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_c_driver'] = 'VCP' if ((cfg0 >> 4) & (1 << 3)) else ''
        cfg['channel_d_driver'] = 'VCP' if ((cfg1 >> 4) & (1 << 3)) else ''
        conf = self._eeprom[0x0B]
        rs485 = self._CHANNEL.RS485
        for chix in range(4):
            cfg['channel_%x_rs485' % (0xa+chix)] = bool(conf & (rs485 << chix))

    def _decode_x232h(self, cfg):
        # common code for2232h and 4232h
        cfg0, cfg1 = self._eeprom[0x00], self._eeprom[0x01]
        cfg['channel_a_driver'] = 'VCP' if (cfg0 & (1 << 3)) else ''
        cfg['channel_b_driver'] = 'VCP' if (cfg1 & (1 << 3)) else ''
        max_drive = self._DRIVE.LOW | self._DRIVE.HIGH
        for bix in range(4):
            if not bix & 1:
                val = self._eeprom[0x0c + bix//2]
            else:
                val >>= 4
            cfg['group_%d_drive' % bix] = bool(val & max_drive)
            cfg['group_%d_schmitt' % bix] = bool(val & self._DRIVE.SCHMITT)
            cfg['group_%d_slew' % bix] = bool(val & self._DRIVE.SLOW_SLEW)
        cfg['chip'] = Hex2Int(self._eeprom[0x18])
