"""PyUSB virtual FTDI device."""

# Copyright (c) 2020-2021, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=unused-argument
#pylint: disable-msg=invalid-name
#pylint: disable-msg=too-many-arguments
#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-many-branches
#pylint: disable-msg=too-many-statements
#pylint: disable-msg=too-many-instance-attributes
#pylint: disable-msg=too-many-public-methods
#pylint: disable-msg=too-few-public-methods
#pylint: disable-msg=no-self-use

import os
from array import array
from binascii import hexlify
from collections import deque
from enum import IntEnum, unique
from logging import getLogger
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from sys import version_info
from threading import Event, Lock, Thread
from time import sleep, time as now
from typing import List, Mapping, NamedTuple, Optional, Sequence, Tuple
from pyftdi.eeprom import FtdiEeprom   # only for consts, do not use code
from .consts import FTDICONST, USBCONST
from .mpsse import VirtMpsseEngine, VirtMpsseTracer


class Pipe:
    """Wrapper around os.pipe
    """

    PIPE = {'r': 0, 'w': 1}

    def __init__(self):
        self._pipe = None
        self._zombie = False

    def close(self):
        if self._pipe is None:
            return
        os.close(self._pipe[0])
        os.close(self._pipe[1])
        self._pipe = None
        self._zombie = True

    def read(self, count: int) -> bytes:
        return os.read(self.r, count)

    def write(self, buf: bytes) -> None:
        os.write(self.w, buf)

    @property
    def ep_out(self) -> 'Pipe':
        # if pin is output, return RX endpoint (so client can read from)
        return self.r

    @property
    def ep_in(self) -> 'Pipe':
        # if pin in input, return TX endpoint (so client can write to)
        return self.w

    def __getattr__(self, name):
        if self._zombie:
            raise IOError('Closed pipe')
        try:
            pos = self.PIPE[name[0]]
        except KeyError as exc:
            raise AttributeError(f'No such pipe attribute: {name}') from exc
        if not self._pipe:
            # lazy instanciation
            self._pipe = os.pipe()
        return self._pipe[pos]


class Fifo:
    """Communication queue."""

    def __init__(self, size: int = 0):
        self.q = deque()
        self.lock = Lock()
        self.event = Event()
        self.size: int = size


class VirtualFtdiPin:
    """Virtual FTDI pin to enable interconnexion/communication
    """

    class Function(IntEnum):

        TRISTATE = 0
        GPIO = 1
        CLOCK = 2
        STREAM = 3

    def __init__(self, port, position):
        self.log = getLogger(f'pyftdi.vftdi[{port.iface}].{position}')
        self._port = port
        self._position = position
        self._next_pin = None
        self._function = self.Function.TRISTATE
        self._pipe = None

    def connect_to(self, pin: 'VirtualFtdiPin') -> None:
        if self == pin and self._position == pin.position:
            raise ValueError(f'Cannot connect pin {self._port.iface}:'
                             f'{self._position} to itself')
        self._next_pin = pin

    def disconnect(self, pin: 'VirtualFtdiPin') -> None:
        self._next_pin = None

    def set_function(self,
                     function: Optional['VirtualFtdiPin.Function'] = None) \
            -> None:
        self._function = function or self.Function.TRISTATE
        if self._function != self.Function.STREAM:
            if self._pipe:
                self._pipe.close()
                self._pipe = None

    def set(self, source: bool, signal: bool) -> None:
        self.log.debug('set %d <- %s', int(signal), 'ext' if source else 'int')
        if self._function == self.Function.TRISTATE:
            return
        if self._function == self.Function.GPIO:
            if self.direction:
                if self._next_pin:
                    # when propagate to another pin, always an external source
                    self._next_pin.set(True, signal)
            else:
                self._port.set_io(self, signal)
            return
        self.log.error('GPIO update on a featured pin: %s', self._function)

    def read(self, count: int) -> bytes:
        # from external
        if self._function != self.Function.STREAM:
            raise IOError('Pin is not of stream kind')
        if self._pipe is None:
            self._pipe = Pipe()
        return self._pipe.read(count)

    def push_to_pin(self, buf: bytes) -> None:
        # from internal
        if self._function != self.Function.STREAM:
            raise IOError('Pin is not of stream kind')
        if self.is_connected:
            self._next_pin.write(buf)
        if self._pipe is None:
            self._pipe = Pipe()
        self._pipe.write(buf)

    def write(self, buf: bytes) -> None:
        # from external
        if self._function != self.Function.STREAM:
            raise IOError('Pin is not of stream kind')
        if self.is_connected:
            raise IOError(f'Cannot write to connected pin ({self._position})')
        self._port.write_from_pin(self._position, buf)

    @property
    def port(self) -> 'VirtFtdiPort':
        return self._port

    @property
    def connection(self) -> Optional['VirtualFtdiPin']:
        return self._next_pin

    @property
    def is_connected(self) -> bool:
        return bool(self._next_pin)

    @property
    def position(self) -> int:
        return self._position

    @property
    def direction(self) -> bool:
        return bool(self._port.direction & (1 << self._position))

    @property
    def signal(self) -> Optional[bool]:
        if self._function == self.Function.TRISTATE:
            return None
        if self._function == self.Function.GPIO:
            return bool(self._port.gpio & (1 << self._position))
        raise IOError('No signal available from this pin')


class VirtFtdiPort:
    """Virtual FTDI port/interface

       :param iface: the interface number (start from 1)
    """

    POLL_DELAY = 1e-3
    """Delay between processing (generating) a chunk of data."""
    SLEEP_DELAY = 200e-3
    """Timeout when the worker is idle."""

    class Fifos(NamedTuple):
        rx: Fifo  # Host-to-FTDI
        tx: Fifo  # FTDI-to-host


    @unique
    class BitMode(IntEnum):
        """Function mode selection.

           It is used so many times hereafter than relying on FTDICONST is
           not very handy, so redefine it
        """

        RESET = 0x00    # switch off altnerative mode (default to UART)
        BITBANG = 0x01  # classical asynchronous bitbang mode
        MPSSE = 0x02    # MPSSE mode, available on 2232x chips
        SYNCBB = 0x04   # synchronous bitbang mode
        MCU = 0x08      # MCU Host Bus Emulation mode,
        OPTO = 0x10     # Fast Opto-Isolated Serial Interface Mode
        CBUS = 0x20     # Bitbang on CBUS pins of R-type chips
        SYNCFF = 0x40   # Single Channel Synchronous FIFO mode

    FIFO_SIZES = {
        0x0200: (128, 128),    # FT232AM: TX: 128, RX: 128
        0x0400: (128, 384),    # FT232BM: TX: 128, RX: 384
        0x0500: (128, 384),    # FT2232C: TX: 128, RX: 384
        0x0600: (256, 128),    # FT232R:  TX: 256, RX: 128
        0x0700: (4096, 4096),  # FT2232H: TX: 4KiB, RX: 4KiB
        0x0800: (2048, 2048),  # FT4232H: TX: 2KiB, RX: 2KiB
        0x0900: (1024, 1024),  # FT232H:  TX: 1KiB, RX: 1KiB
        0x1000: (512, 512),    # FT-X:    TX: 512, RX: 512
    }
    """FTDI chip internal FIFO sizes.

       TX: to-host, RX: from-host
    """

    WIDTHS = {
        0x0200: 8,
        0x0400: 8,
        0x0500: 12,
        0x0600: 8,
        0x0700: 16,
        0x0800: 8,
        0x0900: 16,
        0x1000: 8}
    """Interterface pin count."""

    UART_PINS = IntEnum('UartPins', 'TXD RXD RTS CTS DTR DSR DCD RI', start=0)
    """Function of pins in UART mode."""

    @unique
    class Command(IntEnum):
        """Command type for command queue (TX thread)."""

        TERMINATE = 0x1    # Terminate thread
        SET_BITMODE = 0x2  # Change the bitmode

    def __init__(self, parent: 'VirtFtdi', iface: int):
        self.log = getLogger(f'pyftdi.vftdi[{iface}]')
        self._parent = parent
        self._iface: int = iface
        self._bitmode = self.BitMode.RESET
        self._hispeed: bool = False
        self._baudrate: int = 0
        self._bb_clock: int = 0
        self._mpsse: Optional[VirtMpsseTracer] = None
        self._direction: int = 0
        self._gpio: int = 0
        self._width = self.WIDTHS[parent.version]
        self._pins: List[VirtualFtdiPin] = []
        self._fifos: VirtFtdiPort.Fifos = VirtFtdiPort.Fifos(
            Fifo(self.FIFO_SIZES[self._parent.version][1]),
            Fifo(self.FIFO_SIZES[self._parent.version][0]))
        self._cbus_dir: int = 0  # logical (commands)
        self._cbus: int = 0  # logical (commands)
        self._cbus_map: Optional[Mapping[int, int]] = None  # logical to phys.
        self._cbusp_gpio: int = 0  # physical (pins)
        self._cbusp_force: int = 0  # physical (pins)
        self._cbusp_active: int = 0  # physical (pins)
        self._resume: bool = True
        self._cmd_q = Fifo()
        self._last_txw_ts = 0
        for pin in range(self._width):
            self._pins.append(VirtualFtdiPin(self, pin))
        self._rx_thread = Thread(
            target=self._rx_worker,
            name=f'Ftdi-{parent.bus}:{parent.address}/{iface}-Rx',
            daemon=True)
        self._tx_thread = Thread(
            target=self._tx_worker,
            name=f'Ftdi-{parent.bus}:{parent.address}/{iface}-Tx',
            daemon=True)
        self._rx_thread.start()
        self._tx_thread.start()

    def close(self, freeze: bool = False) -> None:
        self.log.debug('> close %d', freeze)
        if self._tx_thread:
            with self._cmd_q.lock:
                self._cmd_q.q.append((self.Command.TERMINATE,))
                self._cmd_q.event.set()
        timeout = now()+.5
        while self._cmd_q.q:
            sleep(50e-3)
            if now() >= timeout:
                self.log.warning('aborting before full completion')
                break
        self._resume = False
        if self._rx_thread:
            self._rx_thread.join()
            self._rx_thread = None
        if self._tx_thread:
            self._tx_thread.join()
            self._tx_thread = None

    def terminate(self):
        self.close()

    def __getitem__(self, index: int) -> VirtualFtdiPin:
        if not isinstance(index, int):
            raise TypeError(f"sequence index must be integer, "
                            f"not '{index.__class__.__name__}'")
        try:
            return self._pins[index]
        except IndexError:
            raise IndexError(f'Invalid pin: {index}') from None

    @property
    def iface(self) -> int:
        return self._iface

    @property
    def baudrate(self) -> int:
        return self._baudrate

    @property
    def width(self) -> int:
        return self._width

    @property
    def direction(self) -> int:
        return self._direction

    @property
    def gpio(self) -> int:
        return self._gpio

    @property
    def cbus(self) -> Tuple[int, int]:
        """Emulate CBUS output (from FTDI to peripheral).

           :return: a tuple of logical value on pins, active pins.
                    non-active pins should be considered as High-Z
        """
        return self._cbus_read()

    @cbus.setter
    def cbus(self, cbus: int) -> None:
        """Emulate CBUS input (from peripheral to FTDI)."""
        self._cbus_write(cbus)

    def write(self, data: array, timeout: int) -> int:
        rx_fifo = self._fifos.rx
        with rx_fifo.lock:
            count = len(rx_fifo.q)
            rx_fifo.q.append(data)
            rx_fifo.event.set()
        if not count:
            # the FIFO was empty, so wait for the first request to complete
            # this is a hackish way to ensure a request when the device is
            # not busy handling his FIFOs responds "immediately" to the
            # first request
            timeout = now()+2  # note that verbose traces may trigger a timeout
            while self._resume and rx_fifo.q:
                sleep(self.POLL_DELAY)
                if now() >= timeout:
                    raise RuntimeError('RX queue seems stalled')
        return len(data)

    def read(self, buff: array, timeout: int) -> int:
        tx_fifo = self._fifos.tx
        count = len(buff)
        if count < 2:
            self.log.warning('Never handle buffers that cannot fit status')
            return 0
        if self._bitmode in (self.BitMode.RESET,
                             self.BitMode.BITBANG,
                             self.BitMode.SYNCBB,
                             self.BitMode.MPSSE):
            status = self.modem_status
            buff[0], buff[1] = status[0], status[1]
            pos = 2
            with tx_fifo.lock:
                while tx_fifo.q and pos < count:
                    buff[pos] = tx_fifo.q.popleft()
                    pos += 1
            return pos
        self.log.warning('Read buffer discarded, mode %s',
                         self.BitMode(self._bitmode).name)
        self.log.warning('. bbr (%d)', len(buff))
        return 0

    def set_io(self, pin: VirtualFtdiPin, signal: bool) -> None:
        position = pin.position
        if not 0 <= position < self._width:
            raise ValueError(f'Invalid pin: {position}')
        if signal:
            self._update_gpio(True, self._gpio | (1 << position))
        else:
            self._update_gpio(True, self._gpio & ~(1 << position))

    def update_gpio(self, mpsse: VirtMpsseEngine, source: bool,
                    direction: int, gpio: int) -> None:
        self._direction = direction
        self._update_gpio(source, gpio)

    @property
    def modem_status(self) -> Tuple[int, int]:
        # For some reason, B0 high nibble matches the LPC214x UART:UxMSR
        # B0.0  ?
        # B0.1  ?
        # B0.2  ?
        # B0.3  ?
        # B0.4  Clear to send (CTS)
        # B0.5  Data set ready (DTS)
        # B0.6  Ring indicator (RI)
        # B0.7  Receive line signal / Data carrier detect (RLSD/DCD)

        # For some reason, B1 exactly matches the LPC214x UART:UxLSR
        # B1.0  Data ready (DR)
        # B1.1  Overrun error (OE)
        # B1.2  Parity error (PE)
        # B1.3  Framing error (FE)
        # B1.4  Break interrupt (BI)
        # B1.5  Transmitter holding register (THRE)
        # B1.6  Transmitter empty (TEMT)
        # B1.7  Error in RCVR FIFO
        buf0 = 0x02  # magic constant, no idea for now
        if self._bitmode == self.BitMode.RESET:
            cts = 0x01 if self._gpio & 0x08 else 0
            dsr = 0x02 if self._gpio & 0x20 else 0
            ri = 0x04 if self._gpio & 0x80 else 0
            dcd = 0x08 if self._gpio & 0x40 else 0
            buf0 |= cts | dsr | ri | dcd
        else:
            # another magic constant
            buf0 |= 0x30
        buf1 = 0
        rx_fifo = self._fifos.rx
        with rx_fifo.lock:
            if not rx_fifo.q:
                # TX empty -> flag THRE & TEMT ("TX empty")
                buf1 |= 0x40 | 0x20
        return buf0, buf1

    def control_reset(self, wValue: int, wIndex: int,
                      data: array) -> None:
        reset = FTDICONST.get_name('sio_reset', wValue)
        if reset == 'sio':
            # the thruth is that FTDI does not document what this command
            # exactly does...
            for fifo in self._fifos:
                with fifo.lock:
                    fifo.q.clear()
            self.log.info('> ftdi reset is not fully implemented')
            self._gpio = 0
            self._direction = 0
            self._bitmode = self.BitMode.RESET
            return
        if reset == 'purge_tx':
            fifo = self._fifos.tx
            self.log.info('> ftdi %s: %d requests', reset, len(fifo.q))
            with fifo.lock:
                fifo.q.clear()
            return
        if reset == 'purge_rx':
            fifo = self._fifos.rx
            self.log.info('> ftdi %s: %d requests', reset, len(fifo.q))
            with fifo.lock:
                fifo.q.clear()
            return
        self.log.error('Unknown reset kind: %s', reset)

    def control_set_bitmode(self, wValue: int, wIndex: int,
                            data: array) -> None:
        direction = wValue & 0xff
        bitmode = (wValue >> 8) & 0x7F
        mode = self.BitMode(bitmode).name
        self.log.info('> ftdi bitmode %s: %s', mode, f'{direction:08b}')
        self._bitmode = bitmode
        with self._cmd_q.lock:
            self._cmd_q.q.append((self.Command.SET_BITMODE, self._bitmode))
            self._cmd_q.event.set()
        # be sure to wait for the command to be popped out before resuming
        loop = 10  # is something goes wrong, do not loop forever
        while loop:
            if not self._cmd_q.q:
                # command queue has been fully processed
                break
            if not self._resume:
                # if the worker threads are ending or have ended, abort
                self.log.warning('Premature end of worker')
                return
            loop -= 1
            # kepp some time for the commands to be processed
            sleep(0.05)
        else:
            raise RuntimeError(f'Command {self._cmd_q.q[-1][0].name} '
                               f'not handled')
        if bitmode == self.BitMode.CBUS:
            self._cbus_dir = direction >> 4
            mask = (1 << self._parent.properties.cbuswidth) - 1
            self._cbus_dir &= mask
            # clear output pins
            self._cbus &= ~self._cbus_dir & 0xF
            # update output pins
            output = direction & 0xF & self._cbus_dir
            self._cbus |= output
            self.log.info('> ftdi cbus dir %s, io %s, mask %s',
                          f'{self._cbus_dir:04b}',
                          f'{self._cbus:04b}',
                          f'{mask:04b}')
        elif bitmode == self.BitMode.RESET:
            self._direction = ((1 << VirtFtdiPort.UART_PINS.TXD) |
                               (1 << VirtFtdiPort.UART_PINS.RTS) |
                               (1 << VirtFtdiPort.UART_PINS.DTR))
            self._pins[0].set_function(VirtualFtdiPin.Function.STREAM)
            self._pins[1].set_function(VirtualFtdiPin.Function.STREAM)
            for pin in self._pins[2:]:
                pin.set_function(VirtualFtdiPin.Function.GPIO)
        else:
            self._direction = direction
            for pin in self._pins:
                pin.set_function(VirtualFtdiPin.Function.GPIO)
        if bitmode == self.BitMode.MPSSE:
            for pin in self._pins:
                pin.set_function(VirtualFtdiPin.Function.GPIO)
            if not self._mpsse:
                self._mpsse = VirtMpsseTracer(self, self._parent.version)

    def control_set_latency_timer(self, wValue: int, wIndex: int,
                                  data: array) -> None:
        self.log.debug('> ftdi latency timer: %d', wValue)

    def control_set_event_char(self, wValue: int, wIndex: int,
                               data: array) -> None:
        char = wValue & 0xFF
        enable = bool(wValue >> 8)
        self.log.info('> ftdi %sable event char: 0x%02x',
                      'en' if enable else 'dis', char)

    def control_set_error_char(self, wValue: int, wIndex: int,
                               data: array) -> None:
        char = wValue & 0xFF
        enable = bool(wValue >> 8)
        self.log.info('> ftdi %sable error char: 0x%02x',
                      'en' if enable else 'dis', char)

    def control_read_pins(self, wValue: int, wIndex: int,
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

    def control_set_baudrate(self, wValue: int, wIndex: int,
                             data: array) -> None:
        FRAC_INV_DIV = (0, 4, 2, 1, 3, 5, 6, 7)
        BAUDRATE_REF_BASE = 3.0E6  # 3 MHz
        BAUDRATE_REF_HIGH = 12.0E6  # 12 MHz
        if self._parent.is_hispeed_device or self._parent.is_x_series:
            wIndex >>= 8
        divisor = wValue | (wIndex << 16)
        div = divisor & 0x3FFF
        hispeed = bool(divisor & 0x20000)
        if not self._parent.is_hispeed_device and hispeed:
            raise ValueError('Invalid hispeed mode with non-H series')
        subdiv_code = (divisor >> 14) & 0x7
        refclock = BAUDRATE_REF_HIGH if hispeed else BAUDRATE_REF_BASE
        if div < 2 and subdiv_code:
            raise ValueError('Invalid sub-divisor with special div')
        if div == 0:
            baudrate = 3.0e6
        elif div == 1:
            baudrate = 2.0e6
        else:
            subdiv = FRAC_INV_DIV[subdiv_code]
            baudrate = round((refclock * 8) / (div * 8 + subdiv))
        self._hispeed = hispeed
        self._baudrate = int(baudrate)
        multiplier = FTDICONST.get_value('BITBANG_BAUDRATE_RATIO',
                                         'HIGH' if hispeed else 'BASE')
        self._bb_clock = self._baudrate * multiplier
        self.log.info('> ftdi set_baudrate %d bps, HS: %s, bb %d Hz',
                      baudrate, hispeed, self._bb_clock)

    def control_set_data(self, wValue: int, wIndex: int,
                         data: array) -> None:
        self.log.debug('> ftdi set_data (NOT IMPLEMENTED)')

    def control_set_flow_ctrl(self, wValue: int, wIndex: int,
                              data: array) -> None:
        self.log.debug('> ftdi set_flow_ctrl (NOT IMPLEMENTED)')

    def control_poll_modem_status(self, wValue: int, wIndex: int,
                                  data: array) -> None:
        status = self.modem_status
        self.log.info('> ftdi poll_modem_status %02x%02x',
                      status[0], status[1])
        return status

    def write_from_pin(self, pin: int, buf: bytes) -> None:
        tx_fifo = self._fifos.tx
        with tx_fifo.lock:
            free_count = tx_fifo.size - len(tx_fifo.q)
            if free_count > 0:
                tx_fifo.q.extend(buf[:free_count])
        if free_count < len(buf):
            self.log.warning('FIFO full, truncated buffer from %d', pin)

    def write_from_mpsse(self, mpsse: VirtMpsseEngine, buf: bytes) -> None:
        tx_fifo = self._fifos.tx
        with tx_fifo.lock:
            free_count = tx_fifo.size - len(tx_fifo.q)
            if free_count > 0:
                tx_fifo.q.extend(buf[:free_count])
        if free_count < len(buf):
            self.log.warning('FIFO full (%d bytes), truncated buffer',
                             len(tx_fifo.q))
        self._mpsse.receive(self._iface, buf)

    def _update_gpio(self, source: bool, gpio: int) -> None:
        changed = self._gpio ^ gpio
        self._gpio = gpio
        if source:
            # call comes from an external source, no need to propagate more
            return
        for pos, pin in enumerate(self._pins):
            bit = 1 << pos
            if not bit & self._direction:
                # do not update input pins
                continue
            if bit & changed:
                pin.set(False, bool(gpio & (1 << pos)))

    def _decode_cbus_x1000(self) -> None:
        cbus_gpio = 0
        cbus_force = 0
        cbus_active = 0
        for bix in range(4):
            value = self._parent.eeprom[0x1A + bix]
            bit = 1 << bix
            if FtdiEeprom.CBUSX(value).name == 'GPIO':
                cbus_gpio |= bit
                cbus_active |= bit
            elif FtdiEeprom.CBUSX(value).name == 'DRIVE0':
                cbus_force &= ~bit  # useless, for code symmetry
                cbus_active |= bit
            elif FtdiEeprom.CBUSX(value).name == 'DRIVE1':
                cbus_force |= bit
                cbus_active |= bit
        mask = (1 << self._parent.properties.cbuswidth) - 1
        self._cbusp_gpio = cbus_gpio & mask
        self._cbusp_force = cbus_force & mask
        self._cbusp_active = cbus_active & mask
        self.log.debug('x1000 config gpio %s, force %s, active %s',
                       f'{self._cbusp_gpio:04b}',
                       f'{self._cbusp_force:04b}',
                       f'{self._cbusp_active:04b}')

    def _decode_cbus_x0900(self) -> None:
        cbus_gpio = 0
        cbus_force = 0
        cbus_active = 0
        for bix in range(5):
            value = self._parent.eeprom[0x18 + bix]
            low, high = value & 0x0F, value >> 4
            bit = 1 << 2*bix
            if FtdiEeprom.CBUSH(low).name == 'GPIO':
                cbus_gpio |= bit
                cbus_active |= bit
            elif FtdiEeprom.CBUSH(low).name == 'DRIVE0':
                cbus_force &= ~bit  # useless, for code symmetry
                cbus_active |= bit
            elif FtdiEeprom.CBUSH(low).name == 'DRIVE1':
                cbus_force |= bit
                cbus_active |= bit
            bit <<= 1
            if FtdiEeprom.CBUSH(high).name == 'GPIO':
                cbus_gpio |= bit
                cbus_active |= bit
            elif FtdiEeprom.CBUSH(high).name == 'DRIVE0':
                cbus_force &= ~bit  # useless, for code symmetry
                cbus_active |= bit
            elif FtdiEeprom.CBUSH(high).name == 'DRIVE1':
                cbus_force |= bit
                cbus_active |= bit
        mask = (1 << self._parent.properties.cbuswidth) - 1
        self._cbusp_gpio = cbus_gpio & mask
        self._cbusp_force = cbus_force & mask
        self._cbusp_active = cbus_active & mask
        self._cbus_map = {0: 5, 1: 6, 2: 8, 3: 9}
        self.log.debug('x0900 config gpio %s, force %s, active %s',
                       f'{self._cbusp_gpio:04b}',
                       f'{self._cbusp_force:04b}',
                       f'{self._cbusp_active:04b}')

    def _decode_cbus_x0600(self) -> None:
        cbus_gpio = 0
        cbus_active = 0
        bix = 0
        while True:
            value = self._parent.eeprom[0x14 + bix]
            low, high = value & 0x0F, value >> 4
            bit = 1 << (2*bix)
            if FtdiEeprom.CBUS(low).name == 'GPIO':
                cbus_gpio |= bit
                cbus_active |= bit
            if bix == 2:
                break
            bit <<= 1
            if FtdiEeprom.CBUS(high).name == 'GPIO':
                cbus_gpio |= bit
                cbus_active |= bit
            bix += 1
        mask = (1 << self._parent.properties.cbuswidth) - 1
        self._cbusp_gpio = cbus_gpio & mask
        self._cbusp_force = 0
        self._cbusp_active = cbus_active & mask
        self.log.debug('x0600 config gpio %s, force %s, active %s',
                       f'{self._cbusp_gpio:04b}',
                       f'{self._cbusp_force:04b}',
                       f'{self._cbusp_active:04b}')

    def _cbus_write(self, cbus: int) -> None:
        # from peripheral to FTDI
        # mask out CBUS pins which are not configured as GPIOs
        cbus &= self._cbusp_active
        cbus &= self._cbusp_gpio
        self.log.debug('> cbus_write active: %s gpio: %s, force: %s, cbus: %s',
                       f'{self._cbusp_active:04b}',
                       f'{self._cbusp_gpio:04b}',
                       f'{self._cbusp_force:04b}',
                       f'{cbus:04b}')
        if self._cbus_map:
            self.log.info('cbus_write map')
            # convert physical gpio into logical gpio
            lgpio = 0
            for log, phy in self._cbus_map:
                if cbus & (1 << phy):
                    lgpio |= 1 << log
            cbus = lgpio
        # only consider logical input
        cbus &= ~self._cbus_dir
        # combine existing output with new input
        self._cbus &= ~self._cbus_dir
        self._cbus |= cbus & ~self._cbus_dir

    def _cbus_read(self) -> Tuple[int, int]:
        # from FTDI to peripheral
        cbus = self._cbus
        self.log.debug('> cbus_read active %s, gpio %s, force %s, cbus %s',
                       f'{self._cbusp_active:04b}',
                       f'{self._cbusp_gpio:04b}',
                       f'{self._cbusp_force:04b}',
                       f'{cbus:04b}')
        if self._cbus_map:
            self.log.info('cbus_read map')
            # convert logical gpio into physical gpio
            pgpio = 0
            for log, phy in self._cbus_map:
                if cbus & (1 << log):
                    pgpio |= 1 << phy
            cbus = pgpio
        # mask out CBUS pins which are not configured as GPIOs
        cbus &= self._cbusp_gpio
        # apply DRIVE1 to gpio
        cbus |= self._cbusp_force
        self.log.info('< cbus_read cbus %s, active %s',
                      f'{cbus:04b}', f'{self._cbusp_active:04b}')
        return cbus, self._cbusp_active

    def _rx_worker(self):
        """Background handling of data received from host."""
        try:
            while self._resume:
                rx_fifo = self._fifos.rx
                rx_fifo.lock.acquire()
                if not rx_fifo.q:
                    rx_fifo.lock.release()
                    if not rx_fifo.event.wait(self.POLL_DELAY):
                        continue
                    rx_fifo.lock.acquire()
                    rx_fifo.event.clear()
                    if not rx_fifo.q:
                        self.log.error('wake up w/o RX data')
                        rx_fifo.lock.release()
                        continue
                rx_fifo.event.clear()
                data = rx_fifo.q[0]
                rx_fifo.lock.release()
                if self._bitmode == self.BitMode.MPSSE:
                    self._mpsse.send(self._iface, data)
                elif self._bitmode == self.BitMode.RESET:
                    self[self.UART_PINS.TXD].push_to_pin(data)
                elif self._bitmode == self.BitMode.BITBANG:
                    for byte in data:
                        # only 8 LSBs are addressable through this command
                        gpi = self._gpio & ~self._direction & 0xFF
                        gpo = byte & self._direction & 0xFF
                        msb = self._gpio & ~0xFF
                        gpio = gpi | gpo | msb
                        self._update_gpio(False, gpio)
                        self.log.debug('. bbw %02x: %s',
                                       self._gpio, f'{self._gpio:08b}')
                elif self._bitmode == self.BitMode.SYNCBB:
                    tx_fifo = self._fifos.tx
                    lost = 0
                    for byte in data:
                        with tx_fifo.lock:
                            free_count = tx_fifo.size - len(tx_fifo.q)
                            if free_count > 0:
                                tx_fifo.q.append(self._gpio & 0xFF)
                            else:
                                lost += 1
                        # only 8 LSBs are addressable through this command
                        gpi = self._gpio & ~self._direction & 0xFF
                        gpo = byte & self._direction & 0xFF
                        msb = self._gpio & ~0xFF
                        gpio = gpi | gpo | msb
                        self._update_gpio(False, gpio)
                        self.log.debug('. bbw %02x: %s',
                                       self._gpio, f'{self._gpio:08b}')
                    if lost:
                        self.log.debug('%d samples lost, TX full', lost)
                else:
                    try:
                        mode = self.BitMode(self._bitmode).name
                    except ValueError:
                        mode = 'unknown'
                    self.log.warning('Write buffer discarded, mode %s', mode)
                    self.log.warning('. (%d) %s',
                                     len(data), hexlify(data).decode())
                with rx_fifo.lock:
                    rx_fifo.q.popleft()
            self.log.debug('End of worker %s', self._rx_thread.name)
        except Exception as exc:
            self.log.error('Dead of worker %s: %s', self._rx_thread.name, exc)
            raise
        finally:
            # ensure other threads to not run forever if this one is on error
            if self._resume:
                self.log.info('RX worker is triggering death')
            self._resume = False

    def _tx_worker(self):
        """Background handling of data sent to host."""
        bitmode = self.BitMode.RESET
        _wait_delay = self.SLEEP_DELAY
        terminated = False
        try:
            while self._resume:
                # in order not to overload CPU with real-time computation,
                # time is sliced into POLL_DELAY period; every POLL_DELAY
                # period, this thead generates as many data as the real HW
                # would have generated during this period (~ epsilon due to
                # rounding)
                if not self._cmd_q.q:
                    if not self._cmd_q.event.wait(_wait_delay):
                        self._tx_worker_generate(bitmode)
                        continue
                with self._cmd_q.lock:
                    if not self._cmd_q.q:
                        self.log.error('wake up w/o CMD')
                        continue
                    self._cmd_q.event.clear()
                    command = self._cmd_q.q.popleft()
                self.log.info('Command %s', self.Command(command[0]).name)
                if command[0] == self.Command.TERMINATE:
                    terminated = True
                    break
                if command[0] == self.Command.SET_BITMODE:
                    bitmode = command[1]
                    self._last_txw_ts = now()
                    if bitmode in (self.BitMode.BITBANG,
                                   self.BitMode.SYNCBB):
                        _wait_delay = self.POLL_DELAY
                    else:
                        _wait_delay = self.SLEEP_DELAY
                    continue
                else:
                    self.log.error('Unimplemented support for command %d',
                                   command)
                    continue
            self.log.debug('End of worker %s', self._tx_thread.name)
        except Exception as exc:
            self.log.error('Dead of worker %s: %s', self._tx_thread.name, exc)
            raise
        finally:
            # ensure other threads to not run forever if this one is on error
            if not terminated and self._resume:
                self.log.info('TX worker is triggering death')
            self._resume = False

    def _tx_worker_generate(self, bitmode) -> None:
        tx_fifo = self._fifos.tx
        if bitmode == self.BitMode.BITBANG:
            ts = now()
            # how much time has elapsed since last background action
            elapsed = ts-self._last_txw_ts
            self._last_txw_ts = ts
            # how many bytes should have been captured since last
            # action
            byte_count = round(self._baudrate*elapsed)
            # fill the TX FIFO with as many bytes, stop if full
            if byte_count:
                with tx_fifo.lock:
                    free_count = tx_fifo.size - len(tx_fifo.q)
                    push_count = min(free_count, byte_count)
                    tx_fifo.q.extend([self._gpio] * push_count)
                self.log.debug('in %.3fms -> %d',
                               elapsed*1000, push_count)


class VirtFtdi:
    """Virtual FTDI device.

       :param version: FTDI version (device kind)
       :param eeprom_size: size of external EEPROM size, if any
    """

    class Properties(NamedTuple):
        """Device properties."""
        ifcount: int  # count of interface
        ifwidth: int  # pin width of an interface
        cbuswidth: int  # pin width of the control bus

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

    PROPERTIES: Mapping[int, Properties] = {
        0x0200: Properties(1, 8, 0),   # FT232AM
        0x0400: Properties(1, 8, 0),   # FT232BM
        0x0500: Properties(1, 8, 0),   # FT2232D
        0x0600: Properties(1, 8, 5),   # FT232R
        0x0700: Properties(2, 16, 0),  # FT2232H
        0x0800: Properties(4, 8, 0),   # FT4232H
        0x0900: Properties(1, 8, 10),  # FT232H
        0x1000: Properties(1, 8, 4),   # FT231X
    }
    """Width of port/bus (regular, cbus)."""

    EEPROM_REQ_BASE = FTDICONST.get_value('SIO_REQ', 'EEPROM')
    """Base value for EEPROM request."""

    def __init__(self, version: int, bus: int, address: int,
                 eeprom: Optional[dict] = None):
        self.log = getLogger('pyftdi.vftdi')
        self._version = version
        self._bus: int = bus
        self._address: int = address
        self._eeprom: bytearray = self._build_eeprom(version, eeprom)
        self._ports: List[VirtFtdiPort] = []
        for iface in range(self.PROPERTIES[self._version].ifcount):
            self._ports.append(VirtFtdiPort(self, iface+1))

    def close(self, freeze: bool = False) -> None:
        for port in self._ports:
            port.close(freeze)

    def terminate(self):
        self.close()

    @property
    def version(self) -> int:
        return self._version

    @property
    def bus(self) -> int:
        return self._bus

    @property
    def address(self) -> int:
        return self._address

    @property
    def properties(self) -> 'VirtFtdi.Properties':
        return self.PROPERTIES[self._version]

    @property
    def is_mpsse_device(self) -> bool:
        """Tell whether the device has an MPSSE engine

           :return: True if the FTDI device supports MPSSE
        """
        return self._version in (0x0500, 0x0700, 0x0800, 0x0900)

    @property
    def is_hispeed_device(self) -> bool:
        """Tell whether the device is a high speed device

           :return: True if the FTDI device is HS
        """
        return self._version in (0x0700, 0x0800, 0x0900)

    @property
    def is_x_series(self) -> bool:
        """Tell whether the device is a FT-X device

           :return: True for FT-X device
        """
        return self._version == 0x1000

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
            if bRequest >= self.EEPROM_REQ_BASE:
                obj = self
            else:
                obj = self._ports[0xFF & (wIndex-1)] if wIndex else self
        except IndexError:
            raise ValueError(f'Invalid iface: 0x{wIndex:04x} '
                             f'for {req_name}') from None
        try:
            pre = '_' if obj == self else ''
            handler = getattr(obj, f'{pre}control_{req_name}')
        except AttributeError:
            self.log.warning('Unknown request %02x:%04x: %s for %s)',
                             bRequest, wIndex, req_name,
                             obj.__class__.__name__)
            return size
        buf = handler(wValue, wIndex, data) or b''
        size = len(buf)
        data[:size] = array('B', buf)
        self.log.debug('< (%d) %s', size, hexlify(data[:size]).decode())
        return size

    def write(self, dev_handle: 'VirtDeviceHandle', ep: int, intf: int,
              data: array, timeout: int) -> int:
        return self._ports[intf].write(data, timeout)

    def read(self, dev_handle: 'VirtDeviceHandle', ep: int, intf: int,
             buff: array, timeout: int) -> int:
        return self._ports[intf].read(buff, timeout)

    def get_port(self, iface: int):
        # iface: 1..n
        return self._ports[iface-1]

    @property
    def eeprom(self) -> bytes:
        return bytes(self._eeprom)

    @eeprom.setter
    def eeprom(self, value: bytes):
        if len(value) != len(self._eeprom):
            raise ValueError('EEPROM size mismatch')
        self._eeprom = bytearray(value)

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
                except KeyError as exc:
                    raise ValueError('Unsupported EEPROM model: {model}') \
                            from exc
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
            cbus_func = getattr(self._ports[0], cbus_dec)
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

    def _control_set_baudrate(self, wValue: int, wIndex: int,
                              data: array) -> None:
        # set_baudrate is very specific and hack-ish, as the HW is.
        # the way the interface is encoded is simply ugly (likely a legacy
        # from single-port device, then FTDI needed to support multi-port
        # ones and found a hack-ish way to implement it w/o breaking the
        # existing implementation)
        if self.is_mpsse_device:
            iface = wIndex & 0xFF
            wIndex >>= 16
        else:
            iface = 1
        return self._ports[iface-1].control_set_baudrate(wValue, wIndex, data)
