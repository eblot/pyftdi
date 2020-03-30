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
from threading import Event, Lock, Thread
from time import sleep
from typing import List, Mapping, NamedTuple, Optional, Sequence, Tuple
from pyftdi.eeprom import FtdiEeprom   # only for consts, do not use code
from pyftdi.tracer import FtdiMpsseTracer
from .consts import FTDICONST, USBCONST

# need support for f-string syntax
if version_info[:2] < (3, 6):
    raise AssertionError('Python 3.6 is required for this module')


class VirtMpsseTracer(FtdiMpsseTracer):
    """Reuse MPSSE tracer as a MPSSE command decoder engine.
    """

    def __init__(self, version: int):
        super().__init__(version)
        self.log = getLogger('pyftdi.virt.mpsse')


class Fifo:
    """Communication queues."""

    def __init__(self):
        self.q = deque()
        self.lock = Lock()
        self.event = Event()
        self.stamp: int = 0


class VirtFtdiPort:
    """Virtual FTDI port/interface

       :param iface: the interface number (start from 1)
    """

    POLL_DELAY = 1e-3

    class Fifos(NamedTuple):
        rx: Fifo  # Host-to-FTDI
        tx: Fifo  # FTDI-to-host


    def __init__(self, parent: 'VirtFtdi', iface: int):
        self.log = getLogger('pyftdi.virt.ftdi[{iface}]')
        self._parent = parent
        self._iface: int = iface
        self._bitmode = FTDICONST.get_value('bitmode', 'reset')
        self._mpsse: Optional[VirtMpsseTracer] = None
        self._direction: int = 0
        self._gpio: int = 0
        self._fifos: VirtFtdiPort.Fifos = VirtFtdiPort.Fifos(Fifo(), Fifo())
        # self._status: int = 0  # second byte of modem status
        self._cbus_dir: int = 0  # logical (commands)
        self._cbus: int = 0  # logical (commands)
        self._cbus_map: Optional[Mapping[int, int]] = None  # logical to phys.
        self._cbusp_gpio: int = 0  # physical (pins)
        self._cbusp_force: int = 0  # physical (pins)
        self._cbusp_active: int = 0  # physical (pins)
        self._resume: bool = True
        bus = parent.bus
        address = parent.address
        self._rx_thread = Thread(target=self._rx_worker,
                                 name=f'Ftdi-{bus}:{address}/{iface}',
                                 daemon=True)
        self._rx_thread.start()
        self._next_stamp = 0
        self._stream_txd = deque()  # represent the TXD pin

    def terminate(self):
        self._resume = False
        if self._rx_thread:
            self._rx_thread.join()
            self._rx_thread = None

    @property
    def direction(self) -> int:
        return self._direction

    @property
    def gpio(self) -> int:
        """Emulate GPIO output (from FTDI to peripheral)."""
        self._wait_for_sync()
        return self._gpio

    @gpio.setter
    def gpio(self, gpio: int) -> None:
        """Emulate GPIO input (from peripheral to FTDI)."""
        mask = (1 << self._parent.properties.ifwidth) - 1
        self._gpio |= gpio & ~self._direction & mask

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
        with self._fifos.rx.lock:
            self._fifos.rx.q.extend(data)
            print(f'> RX Q {hexlify(bytes(self._fifos.rx.q))}')
            self._next_stamp = self._fifos.rx.stamp + 1
            self._fifos.rx.event.set()
        return len(data)

    def read(self, buff: array, timeout: int) -> int:
        if self._bitmode == FTDICONST.get_value('bitmode', 'reset'):
            count = len(buff)
            if count < 2:
                return 0
            status = self.modem_status
            buff[0], buff[1] = status[0], status[1]
            pos = 2
            with self._fifos.tx.lock:
                while self._fifos.tx.q and pos < count:
                    buff[pos] = self._fifos.tx.q.popleft()
                    pos += 1
            return pos
        mode = FTDICONST.get_name('bitmode', self._bitmode)
        self.log.debug('Read buffer discarded, mode %s', mode)
        self.log.debug('. (%d)', len(buff))
        return 0

    def uart_write(self, buffer: bytes) -> None:
        with self._fifos.tx.lock:
            self._fifos.tx.q.extend(buffer)

    def uart_read(self, count: int) -> bytes:
        buf = bytearray()
        self._wait_for_sync()
        while self._stream_txd and count:
            buf.append(self._stream_txd.popleft())
            count -= 1
        return bytes(buf)

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
        if self._bitmode == FTDICONST.get_value('bitmode', 'reset'):
            cts = 0x01 if self._gpio & 0x08 else 0
            dsr = 0x02 if self._gpio & 0x20 else 0
            ri = 0x04 if self._gpio & 0x80 else 0
            dcd = 0x08 if self._gpio & 0x40 else 0
            buf0 |= cts | dsr | ri | dcd
        else:
            buf0 |= 0x30  # another magic constant
        buf1 = 0
        with self._fifos.rx.lock:
            if not self._fifos.rx.q:
                # TX empty -> flag THRE & TEMT ("TX empty")
                buf1 |= 0x40 | 0x20
        return buf0, buf1

    def control_reset(self, wValue: int, wIndex: int,
                      data: array) -> None:
        reset = FTDICONST.get_name('sio_reset', wValue)
        self.log.info('> ftdi reset %s', reset)

    def control_set_bitmode(self, wValue: int, wIndex: int,
                            data: array) -> None:
        direction = wValue & 0xff
        bitmode = (wValue >> 8) & 0x7F
        mode = FTDICONST.get_name('bitmode', bitmode)
        self.log.info('> ftdi bitmode %s: %s', mode, f'{direction:08b}')
        self._bitmode = bitmode
        if mode == 'cbus':
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
        else:
            self._direction = direction
        if mode == 'mpsse':
            if not self._mpsse:
                self._mpsse = VirtMpsseTracer(self._parent.version)

    def control_set_latency_timer(self, wValue: int, wIndex: int,
                                  data: array) -> None:
        self.log.info('> ftdi latency timer: %d', wValue)

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
        self.log.info('> ftdi set_baudrate (NOT IMPLEMENTED)')

    def control_set_data(self, wValue: int, wIndex: int,
                         data: array) -> None:
        self.log.info('> ftdi set_data (NOT IMPLEMENTED)')

    def control_set_flow_ctrl(self, wValue: int, wIndex: int,
                              data: array) -> None:
        self.log.info('> ftdi set_flow_ctrl (NOT IMPLEMENTED)')

    def control_poll_modem_status(self, wValue: int, wIndex: int,
                                  data: array) -> None:
        status = self.modem_status
        self.log.info('> ftdi poll_modem_status %02x%02x',
                      status[0], status[1])
        return status

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
        while self._resume:
            with self._fifos.rx.lock:
                if not self._fifos.rx.event.wait(self.POLL_DELAY):
                    continue
                self._fifos.rx.event.clear()
                if not self._fifos.rx.q:
                    self.log.warning('wake up w/o RX data')
                    continue
                # TODO: for now, handle all in once, which does not match
                # real HW
                self._fifos.rx.stamp += 1
                data = bytes(self._fifos.rx.q)
                self._fifos.rx.q.clear()
            if self._bitmode == FTDICONST.get_value('bitmode', 'mpsse'):
                self._mpsse.send(self._iface, data)
                return len(data)
            if self._bitmode == FTDICONST.get_value('bitmode', 'reset'):
                self._stream_txd.extend(data)
            elif self._bitmode == FTDICONST.get_value('bitmode', 'bitbang'):
                for byte in data:
                    # only 8 LSBs are addressable through this command
                    self._gpio &= ~0xFF
                    self._gpio |= byte & self._direction
                    self.log.info('. %02x: %s', self._gpio, f'{self._gpio:08b}')
            else:
                mode = FTDICONST.get_name('bitmode', self._bitmode)
                self.log.warning('Write buffer discarded, mode %s', mode)
                self.log.warning('. (%d) %s',
                                 len(data), hexlify(data).decode())
        self.log.debug('End of worker %s', self._rx_thread.name)

    def _wait_for_sync(self) -> None:
        """Introduce a small delay so that direct access to periphal side of
           FTDI get a chance to be updated if the worked thread has not been
           scheduled once since the last USB command received.

           This is hackish, to be improved
        """
        loop = 10
        while True:
            self._fifos.rx.lock.acquire()
            if self._fifos.rx.stamp < self._next_stamp:
                self._fifos.rx.lock.release()
                sleep(self.POLL_DELAY/2)
                loop -= 1
                if not loop:
                    raise RuntimeError('Worker thread deadlock?')
                continue
            self._fifos.rx.lock.release()
            break
        self.log.debug(f'Sync {self._fifos.rx.stamp} < {self._next_stamp}')


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
        self.log = getLogger('pyftdi.virt.ftdi')
        self._version = version
        self._bus: int = bus
        self._address: int = address
        self._eeprom: bytearray = self._build_eeprom(version, eeprom)
        self._ports: List[VirtFtdiPort] = []
        for iface in range(self.PROPERTIES[self._version].ifcount):
            self._ports.append(VirtFtdiPort(self, iface+1))

    def terminate(self):
        for port in self._ports:
            port.terminate()

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
            self.log.warning('Unknown request: %s', req_name)
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
