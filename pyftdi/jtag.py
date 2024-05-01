# Copyright (c) 2010-2024, Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2016, Emmanuel Bouaziz <ebouaziz@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""JTAG tools.
"""

from logging import getLogger
from time import sleep
from typing import Optional, Union

from .ftdi import Ftdi
from .tools.bits import BitSequence
from .tools.jtag import JtagController


class JtagError(Exception):
    """Generic JTAG error."""


class JtagFtdiController(JtagController):
    """JTAG controller implementation for the FTDI MPSSE engine.
    """

    TCK_BIT = 0x01   # FTDI output
    TDI_BIT = 0x02   # FTDI output
    TDO_BIT = 0x04   # FTDI input
    TMS_BIT = 0x08   # FTDI output
    # TRST_BIT = 0x10  # FTDI output, not available on 2232 JTAG debugger
    JTAG_MASK = 0x0f
    FTDI_PIPE_LEN = 512

    def __init__(self):
        self._log = getLogger('pyftdi.jtag')
        self._ftdi = Ftdi()
        self._trst: Optional[int] = None
        self._direction = (JtagFtdiController.TCK_BIT |
                           JtagFtdiController.TDI_BIT |
                           JtagFtdiController.TMS_BIT)
        self._write_buff = bytearray()
        self._last_tdi: Optional[bool] = None  # tri-state

    # Public API
    def configure(self, url: str, direction: int = 0x0, initial: int = 0x0,
                  frequency: float = 1.0E6, trst_bit: Optional[int] = None) \
            -> None:
        """Configure the FTDI interface as a JTAG controller"""
        self._direction |= direction & ~self.JTAG_MASK
        if trst_bit is not None:
            trst = 1 << trst_bit
            # pylint: disable=superfluous-parens
            if (self.JTAG_MASK & trst) or not (trst_bit & 0xFF):
                raise ValueError('Invalid TRST bit')
            self._trst = trst
            self._direction |= trst
        initial |= initial & ~self.JTAG_MASK
        self._ftdi.open_mpsse_from_url(
            url, direction=self._direction, initial=initial,
            frequency=frequency)
        self._ftdi.purge_buffers()
        self._write_buff = bytearray()

    def close(self, freeze: bool = False) -> None:
        """Close the JTAG interface/port.

           :param freeze: if set, FTDI port is not reset to its default
                          state on close. This means the port is left with
                          its current configuration and output signals.
                          This feature should not be used except for very
                          specific needs.
        """
        if self._ftdi.is_connected:
            self._ftdi.close(freeze)

    @property
    def ftdi(self) -> Ftdi:
        """Return the Ftdi instance.

           :return: the Ftdi instance
        """
        return self._ftdi

    def purge(self) -> None:
        """Pure FTDI HW buffers."""
        self._ftdi.purge_buffers()

    def sync(self) -> None:
        """Push output buffer to FTDI."""
        if not self._ftdi.is_connected:
            raise JtagError('No FTDI connection')
        self._flush()

    def tap_reset(self, use_trst: bool = False) -> None:
        if use_trst and self._trst is None:
            raise ValueError('No TRST pin available')
        if not self._ftdi.is_connected:
            raise JtagError('No FTDI connection')
        self._last_tdi = None
        if use_trst:
            cmd = bytes([Ftdi.GET_BITS_LOW, Ftdi.SEND_IMMEDIATE])
            self._ftdi.write_data(cmd)
            data = self._ftdi.read_data_bytes(1, 4)
            if len(data):
                raise JtagError('Cannot read GPIO')
            value = data[0]
            # pylint: disable=invalid-unary-operand-type
            value &= ~self._trst
            # nTRST
            cmd = bytes((Ftdi.SET_BITS_LOW, value, self._direction))
            self._ftdi.write_data(cmd)
            sleep(0.1)
            # nTRST should be left to the high state
            value |= self._trst
            cmd = bytes((Ftdi.SET_BITS_LOW, value, self._direction))
            self._ftdi.write_data(cmd)
            sleep(0.1)
        # TAP reset (even after HW reset)
        self._log.info('TAP RESET')
        # 5 '1' bits should be enough
        self.write_tms(BitSequence('11111'))
        self._ftdi.read_data_bytes(4, 1)
        self._ftdi.purge_rx_buffer()

    def system_reset(self) -> None:
        if not self._ftdi.is_connected:
            raise JtagError('No FTDI connection')
        self._log.warning('No SRST pin')

    def quit(self) -> None:
        self.close()

    def write_tms(self, modesel: BitSequence, immediate: bool = True) \
            -> BitSequence:
        if not isinstance(modesel, BitSequence):
            raise ValueError('Not a BitSequence')
        length = len(modesel)
        if not 0 < length < 8:
            raise ValueError('Invalid TMS sequence length')
        data = modesel.copy(True).resize(8)
        # left bit, i.e. data[0] is the first bit to send, as LSB mode is
        # selected.
        byte = data.to_byte()
        # apply the last TDO bit
        lbit = self._last_tdi is not None
        if lbit:
            byte |= int(self._last_tdi) << 7
            self._log.debug('WRITE TMS [%d] %s + TDI %u',
                            length, modesel, self._last_tdi)
            self._last_tdi = None
            fcmd = Ftdi.RW_BITS_TMS_PVE_NVE
        else:
            self._log.debug('WRITE TMS [%d] %s', length, modesel)
            fcmd = Ftdi.WRITE_BITS_TMS_NVE
        self._log.debug('TMS %02x', byte)
        cmd = bytes((fcmd, length-1, byte))
        self._ftdi.purge_rx_buffer()
        self._push_bytes(cmd)
        if immediate or lbit:
            self._flush()
        if lbit:
            bs = self._read_bits(length)
            return bs.pop_right()
        return BitSequence()

    def write(self, out: BitSequence, immediate: bool = True) -> None:
        if not isinstance(out, BitSequence):
            raise ValueError('Not a BitSequence')
        self._last_tdi = out.pop_left_bit()
        self._log.debug("WRITE TDI %s [+%u]", out, self._last_tdi)
        byte_count = len(out)//8
        pos = 8 * byte_count
        bit_count = len(out)-pos
        self._log.debug('%dB, %db len %d', byte_count, bit_count, len(out))
        if byte_count:
            self._prepare_write_bytes(byte_count)
            self._write_bytes(out[:pos])
        if bit_count:
            self._prepare_write_bits(bit_count)
            self._write_bits(out[pos:])
        if immediate:
            self._flush()

    def read(self, length: int, tdi: Optional[bool] = None) -> BitSequence:
        # tdi argument is not used with PyFtdi for now
        self._log.debug("READ TDO %d", length)
        byte_count = length // 8
        bit_count = length - 8 * byte_count
        if byte_count:
            self._prepare_read_bytes(byte_count)
        if bit_count:
            self._prepare_read_bits(bit_count)
        self._flush()
        bs = BitSequence()
        if byte_count:
            bs.push_right(self._read_bytes(byte_count))
        if bit_count:
            bs.push_right(self._read_bits(bit_count))
        return bs

    def exchange(self, out: BitSequence) -> BitSequence:
        self._last_tdi = out.pop_left_bit()
        length = len(out)
        self._log.debug("WRITE TDI %s [+%u] + READ TDO %u", out, self._last_tdi,
                        length)
        byte_count = length // 8
        bit_count = length - 8 * byte_count
        pos = 8 * byte_count
        bs = BitSequence()
        if byte_count:
            self._prepare_exchange_bytes(byte_count)
            self._write_bytes(out[:pos])
        if bit_count:
            self._prepare_exchange_bits(bit_count)
            self._write_bits(out[pos:])
        self._flush()
        if byte_count:
            bs = self._read_bytes(byte_count)
        if bit_count:
            bs.push_left(self._read_bits(bit_count))
        return bs

    def _push_bytes(self, cmd: Union[bytes, bytearray]):
        if not isinstance(cmd, (bytes, bytearray)):
            raise TypeError('Expect bytes or bytearray or single byte')
        length = len(cmd)
        # Currrent buffer + new command + send_immediate
        if ((len(self._write_buff) + length + 1) >=
                JtagFtdiController.FTDI_PIPE_LEN):
            self._flush()
        self._write_buff.extend(cmd)

    def _flush(self):
        self._log.debug('flushing %d bytes', len(self._write_buff))
        if self._write_buff:
            self._ftdi.write_data(self._write_buff)
            self._write_buff = bytearray()

    def _prepare_read_bits(self, length: int) -> None:
        if length > 8:
            raise ValueError('Too large')
        cmd = bytes((Ftdi.READ_BITS_NVE_LSB, length-1))
        self._push_bytes(cmd)

    def _prepare_write_bits(self, length: int) -> None:
        if length > 8:
            raise ValueError('Too large')
        cmd = bytes((Ftdi.WRITE_BITS_NVE_LSB, length-1))
        self._push_bytes(cmd)

    def _write_bits(self, out: BitSequence) -> None:
        """Output bits on TDI"""
        byte = out.to_byte()
        self._log.debug('WRITE BITS %s / 0x%02x', out, byte)
        self._push_bytes(bytes((byte,)))

    def _prepare_exchange_bits(self, length: int) -> None:
        if length > 8:
            raise ValueError('Too large')
        cmd = bytes((Ftdi.RW_BITS_PVE_NVE_LSB, length-1))
        self._push_bytes(cmd)

    def _read_bits(self, length: int) -> BitSequence:
        data = self._ftdi.read_data_bytes(1, 4)
        if len(data) != 1:
            raise JtagError('Failed to read from FTDI')
        byte = data[0] >> (8 - length)
        bs = BitSequence(byte, width=length)
        self._log.debug('READ BITS %s', bs)
        return bs

    def _prepare_read_bytes(self, length: int) -> BitSequence:
        """Read out bytes from TDO"""
        if length > JtagFtdiController.FTDI_PIPE_LEN:
            raise JtagError('Cannot fit into FTDI fifo')
        alen = length - 1
        cmd = bytes((Ftdi.READ_BYTES_NVE_LSB, alen & 0xff, (alen >> 8) & 0xff))
        self._push_bytes(cmd)

    def _prepare_exchange_bytes(self, length: int) -> None:
        if length > JtagFtdiController.FTDI_PIPE_LEN:
            raise JtagError('Cannot fit into FTDI fifo')
        olen = length - 1
        cmd = bytearray((Ftdi.RW_BYTES_PVE_NVE_LSB, olen & 0xff,
                        (olen >> 8) & 0xff))
        self._push_bytes(cmd)

    def _prepare_write_bytes(self, length: int) -> None:
        if length > JtagFtdiController.FTDI_PIPE_LEN:
            raise JtagError('Cannot fit into FTDI fifo')
        olen = length - 1
        cmd = bytearray((Ftdi.WRITE_BYTES_NVE_LSB, olen & 0xff,
                        (olen >> 8) & 0xff))
        self._push_bytes(cmd)

    def _read_bytes(self, length: int) -> BitSequence:
        data = self._ftdi.read_data_bytes(length, 4)
        if len(data) != length:
            raise JtagError('Failed to read from FTDI')
        bs = BitSequence.from_bytestream(data, lsbyte=True)
        self._log.debug('READ BYTES %s', bs)
        return bs

    def _write_bytes(self, out: BitSequence) -> None:
        """Output bytes on TDI"""
        self._log.debug('WRITE BYTES %s', out)
        self._push_bytes(out.to_bytestream())
