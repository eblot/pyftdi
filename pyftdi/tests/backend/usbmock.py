"""PyUSB virtual USB backend to intercept all USB requests.

   The role of this module is to enable PyFtdi API testing w/o any FTDI
   hardware.
"""

from array import array
from binascii import hexlify
from copy import deepcopy
from enum import IntEnum
from importlib import import_module
from logging import getLogger
from struct import calcsize as scalc, pack as spack
from typing import Optional
from usb.backend import IBackend
from pyftdi.ftdi import Ftdi
from pyftdi.tracer import FtdiMpsseTracer

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=invalid-name
#pylint: disable-msg=attribute-defined-outside-init
#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-many-arguments
#pylint: disable-msg=too-many-instance-attributes


class EasyDict(dict):
    """Dictionary whose members can be accessed as instance members
    """

    def __init__(self, dictionary=None, **kwargs):
        super().__init__(self)
        if dictionary is not None:
            self.update(dictionary)
        self.update(kwargs)

    def __getattr__(self, name):
        try:
            return self.__getitem__(name)
        except KeyError:
            raise AttributeError("'%s' object has no attribute '%s'" %
                                 (self.__class__.__name__, name))

    def __setattr__(self, name, value):
        self.__setitem__(name, value)

    @classmethod
    def copy(cls, dictionary):

        def _deep_copy(obj):
            if isinstance(obj, list):
                return [_deep_copy(v) for v in obj]
            if isinstance(obj, dict):
                return EasyDict({k: _deep_copy(obj[k]) for k in obj})
            return deepcopy(obj)
        return cls(_deep_copy(dictionary))

    def mirror(self) -> 'EasyDict':
        """Instanciate a mirror EasyDict."""
        return EasyDict({v: k for k, v in self.items()})


class UsbConstants:
    """Expose useful constants defined in PyUSB and allow reverse search, i.e.
       retrieve constant literals from integral values.
    """

    DEVICE_REQUESTS = {
        (True, 0x0): 'get_status',
        (False, 0x1): 'clear_feature',
        (False, 0x3): 'set_feature',
        (False, 0x5): 'set_address',
        (True, 0x6): 'get_descriptor',
        (False, 0x7): 'set_descriptor',
        (True, 0x8): 'get_configuration',
        (False, 0x9): 'set_configuration',
    }

    def __init__(self):
        self._desc_type = self._load_constants('desc_type')
        self._desc_type_mask = self._mask(self._desc_type)
        self._ctrl_dir = self._load_constants('ctrl')
        self._ctrl_dir_mask = self._mask(self._ctrl_dir)
        self._ctrl_type = self._load_constants('ctrl_type')
        self._ctrl_type_mask = self._mask(self._ctrl_type)
        self._ctrl_recipient = self._load_constants('ctrl_recipient')
        self._ctrl_recipient_mask = self._mask(self._ctrl_recipient)
        self._endpoint_type = self._load_constants('endpoint_type')
        self._endpoint_type_mask = self._mask(self._endpoint_type)
        self._descriptors = EasyDict({v.upper(): k
                                      for k, v in self._desc_type.items()})
        self.endpoints = self._load_constants('endpoint', True)
        self.endpoint_types = self._load_constants('endpoint_type', True)

    @property
    def descriptors(self):
        return self._descriptors

    @classmethod
    def _load_constants(cls, prefix: str, reverse=False):
        prefix = prefix.upper()
        if not prefix.endswith('_'):
            prefix = f'{prefix}_'
        mod = import_module('usb.util')
        mapping = EasyDict()
        plen = len(prefix)
        for entry in dir(mod):
            if not entry.startswith(prefix):
                continue
            if '_' in entry[plen:]:
                continue
            if not reverse:
                mapping[getattr(mod, entry)] = entry[plen:].lower()
            else:
                mapping[entry[plen:].lower()] = getattr(mod, entry)
        return mapping

    @classmethod
    def _mask(cls, mapping: dict) -> int:
        mask = 0
        for val in mapping:
            mask |= val
        return mask

    def is_req_out(self, reqtype: int) -> str:
        return not reqtype & self._ctrl_dir_mask

    def dec_req_ctrl(self, reqtype: int) -> str:
        return self._ctrl_dir[reqtype & self._ctrl_dir_mask]

    def dec_req_type(self, reqtype: int) -> str:
        return self._ctrl_type[reqtype & self._ctrl_type_mask]

    def dec_req_rcpt(self, reqtype: int) -> str:
        return self._ctrl_recipient[reqtype & self._ctrl_recipient_mask]

    def dec_req_name(self, reqtype: int, request: int) -> str:
        direction = bool(reqtype & self._ctrl_dir_mask)
        try:
            return self.DEVICE_REQUESTS[(direction, request)]
        except KeyError:
            return f'req x{request:02x}'

    def dec_desc_type(self, desctype: int) -> str:
        return self._desc_type[desctype & self._desc_type_mask]



class FtdiConstants:
    """Expose useful constants defined in Ftdi and allow reverse search, i.e.
       retrieve constant literals from integral values.
    """

    def __init__(self):
        self._cache = {}


    @classmethod
    def _load_constants(cls, prefix: str, reverse=False):
        prefix = prefix.upper()
        if not prefix.endswith('_'):
            prefix = f'{prefix}_'
        mapping = EasyDict()
        plen = len(prefix)
        for entry in dir(Ftdi):
            if not entry.startswith(prefix):
                continue
            if not reverse:
                mapping[getattr(Ftdi, entry)] = entry[plen:].lower()
            else:
                mapping[entry[plen:].lower()] = getattr(Ftdi, entry)
        return mapping

    def translate(self, prefix: str, value: int) -> str:
        if prefix not in self._cache:
            self._cache[prefix] = self._load_constants(prefix)
        try:
            return self._cache[prefix][value]
        except KeyError:
            return f'x?{value:04x}'

    def dec_req_name(self, request: int) -> str:
        return self.translate('sio_req', request)


Constants = UsbConstants()
"""Unique instance of USB constant container."""

FtdiConst = FtdiConstants()
"""Unique instances of FTDI constant container."""


class MockMpsseTracer(FtdiMpsseTracer):
    """Reuse MPSSE tracer as a MPSSE command decoder engine.
    """

    def __init__(self):
        super().__init__(self)
        self.log = getLogger('pyftdi.mock.mpsse')


class MockEndpoint:
    """Fake USB interface endpoint.
    """

    DESCRIPTOR_FORMAT = '<4BHB'

    def __init__(self, direction: str, type_: str, index: int,
                 extra: Optional[bytes] = None):
        class EndpointDescriptor(EasyDict):
            pass
        self.desc = EndpointDescriptor(
            bLength=scalc(self.DESCRIPTOR_FORMAT),
            bDescriptorType=Constants.descriptors.ENDPOINT,
            bEndpointAddress=Constants.endpoints[direction.lower()] | index,
            bmAttributes=Constants.endpoint_types[type_.lower()],
            wMaxPacketSize=64,
            bInterval=0,
            bRefresh=0,
            bSynchAddress=0,
            extra_descriptors=extra or b'')

    def get_length(self) -> int:
        return self.desc.bLength


class MockInterface:
    """Fake USB configuration interface.
    """

    DESCRIPTOR_FORMAT = '<9B'

    def __init__(self, extra: Optional[bytes] = None):
        class InterfaceDescriptor(EasyDict):
            pass
        desc = InterfaceDescriptor(
            bLength=scalc(self.DESCRIPTOR_FORMAT),
            bDescriptorType=Constants.descriptors.INTERFACE,
            bInterfaceNumber=0,
            bAlternateSetting=0,
            bNumEndpoints=0,
            bInterfaceClass=0xFF,
            bInterfaceSubClass=0xFF,
            bInterfaceProtocol=0xFF,
            iInterface=None,  # String desc index
            extra_descriptors=extra or  b'')
        self.alt = 0
        self.altsettings = [(desc, [])]

    def add_endpoint(self, endpoint: MockEndpoint):
        altsetting = self.altsettings[self.alt]
        altsetting[1].append(endpoint)
        altsetting[0].bNumEndpoints = len(altsetting[1])

    def add_bulk_pair(self):
        endpoints = self.altsettings[self.alt][1]
        ep = MockEndpoint('in', 'bulk', len(endpoints)+1)
        self.add_endpoint(ep)
        ep = MockEndpoint('out', 'bulk', len(endpoints)+1)
        self.add_endpoint(ep)

    def get_length(self) -> int:
        length = 0
        for desc, endpoints in self.altsettings:
            length += desc.bLength
            for endpoint in endpoints:
                length += endpoint.get_length()
        return length

    @property
    def num_altsetting(self):
        return len(self.altsetting)

    def __getitem__(self, item):
        if isinstance(item, int):
            return self.altsettings[item]
        raise IndexError('Invalid alternate setting')

    def __getattr__(self, name):
        return getattr(self.altsettings[self.alt][0], name)


class MockConfiguration:
    """Fake USB device configuration.
    """

    DESCRIPTOR_FORMAT = '<2BH5B'

    def __init__(self, extra: Optional[bytes] = None):
        class ConfigDescriptor(EasyDict):
            pass
        self.desc = ConfigDescriptor(
            bLength=scalc(self.DESCRIPTOR_FORMAT),
            bDescriptorType=Constants.descriptors.CONFIG,
            wTotalLength=0,
            bNumInterfaces=0,
            bConfigurationValue=0,
            iConfiguration=0,
            bmAttributes=0x80,  # bus-powered
            bMaxPower=150//2,  # 150 mA
            extra_descriptors=extra or  b'')
        self.interfaces = []

    def add_interface(self, interface: MockInterface):
        interface.bInterfaceNumber = len(self.interfaces)
        self.interfaces.append(interface)
        self.desc.bNumInterfaces = len(self.interfaces)

    def update(self):
        # wTotalLength needs to be updated to the actual length of the
        # sub-objects
        self.desc.wTotalLength = self.get_length()

    def get_length(self) -> int:
        length = self.desc.bLength
        for iface in self.interfaces:
            length += iface.get_length()
        return length

    def __getattr__(self, name):
        return getattr(self.desc, name)


class MockDevice:
    """Fake USB device.
    """

    DESCRIPTOR_FORMAT = '<2BH4B3H4B'

    DEFAULT_LANGUAGE = 0x0409  # en_US

    STRINGS = IntEnum('Strings', 'MANUFACTURER PRODUCT SERIAL_NUMBER', start=1)

    def __init__(self, **kwargs):
        class DeviceDescriptor(EasyDict):
            pass
        self.desc = DeviceDescriptor(
            bLength=scalc(self.DESCRIPTOR_FORMAT),
            bDescriptorType=Constants.descriptors.DEVICE,
            bcdUSB=0x200,  # USB 2.0
            bDeviceClass=0,
            bDeviceSubClass=0,
            bDeviceProtocol=0,
            bMaxPacketSize0=8,
            idVendor=0x403,
            idProduct=0x6014,
            bcdDevice=0x900,
            iManufacturer=self.STRINGS.MANUFACTURER,
            iProduct=self.STRINGS.PRODUCT,
            iSerialNumber=self.STRINGS.SERIAL_NUMBER,
            bNumConfigurations=0,  # updated later
            **kwargs)
        self.configurations = []

    def add_configuration(self, config: MockConfiguration):
        self.configurations.append(config)
        self.desc.bNumConfigurations = len(self.configurations)

    def get_string(self, type_: int, index: int) -> str:
        if index == 0:
            # request for list of supported languages
            # only support one
            fmt = '<BBH'
            size = scalc(fmt)
            buf = spack(fmt, size, type_, self.DEFAULT_LANGUAGE)
            # note: return b'' here to simulate unauthorized access to
            # the USB device
            return buf
        name = self.STRINGS(index).name.lower()
        value = self.get(name, '')
        ms_str = value.encode('utf-16-le')
        fmt = '<BB'
        size = scalc(fmt) + len(ms_str)
        buf = bytearray(spack('<BB', size, type_))
        buf.extend(ms_str)
        return buf

    def __getattr__(self, name):
        return getattr(self.desc, name)


class MockDeviceHandle(EasyDict):
    """Device handle wrapper as expected by PyUSB APIs.
    """

    def __init__(self, dev, handle):
        super().__init__(handle=handle,
                         devid=0,
                         device=dev)


class MockBackend(IBackend):
    """Fake PyUSB backend.

       Implement a subset of PyUSB IBackend interface so that PyFTDI can
       execute w/o a real FTDI HW.
    """

    def __init__(self):
        self._devices = list()
        self._device_handles = dict()
        self._device_handle_count = 0
        self.log = getLogger('pyftdi.mock.backend')
        interface = MockInterface()
        interface.add_bulk_pair()
        config = MockConfiguration()
        config.add_interface(interface)
        config.update()
        dev = MockDevice(bus=1, address=1, speed=3,
                         port_number=1, port_numbers=1,
                         serial_number='SN_1234', product='FT232H')
        dev.add_configuration(config)
        self._devices.append(dev)
        self._mpsse = None

    def enumerate_devices(self):
        for dev in self._devices:
            yield dev

    def open_device(self, dev: MockDevice) -> MockDeviceHandle:
        self._device_handle_count += 1
        devhdl = MockDeviceHandle(dev, self._device_handle_count)
        self._device_handles[devhdl.handle] = devhdl
        return devhdl

    def close_device(self, dev_handle: MockDeviceHandle) -> None:
        del self._device_handles[dev_handle.handle]

    def claim_interface(self, dev_handle: MockDeviceHandle, intf: int):
        self.log.info('> claim interface h:%d: if:%d',
                      dev_handle.handle, intf)

    def release_interface(self, dev_handle: MockDeviceHandle, intf: int):
        self.log.info('> release interface h:%d: if:%d',
                      dev_handle.handle, intf)

    def get_configuration(self,
                          dev_handle: MockDeviceHandle) -> MockConfiguration:
        dev = dev_handle.device
        return dev.configurations[0]

    def set_configuration(self, dev_handle: MockDeviceHandle,
                          config_value) -> None:
        # config_value = ConfigDesc.bConfigurationValue
        pass

    def get_device_descriptor(self, dev):
        return dev.desc

    def get_configuration_descriptor(self, dev, config):
        return dev.configurations[config].desc

    def get_interface_descriptor(self, dev: MockDevice,
                                 intf: int, alt: int, config: int) -> EasyDict:
        cfg = dev.configurations[config]
        iface = cfg.interfaces[intf]
        intf_desc = iface[alt][0]
        return intf_desc

    def get_endpoint_descriptor(self, dev: MockDevice,
                                ep: int, intf: int, alt: int, config: int) \
            -> EasyDict:
        cfg = dev.configurations[config]
        iface = cfg.interfaces[intf]
        endpoints = iface[alt][1]
        ep_desc = endpoints[ep].desc
        return ep_desc

    def ctrl_transfer(self,
                      dev_handle: MockDeviceHandle,
                      bmRequestType: int,
                      bRequest: int,
                      wValue: int,
                      wIndex: int,
                      data: array,
                      timeout: int) -> int:
        req_type = Constants.dec_req_type(bmRequestType)
        if req_type == 'standard':
            return self._ctrl_standard(dev_handle, bmRequestType, bRequest,
                                       wValue, wIndex, data, timeout)
        if req_type == 'vendor':
            return self._ctrl_ftdi(dev_handle, bmRequestType, bRequest,
                                   wValue, wIndex, data, timeout)
        self.log.error('Unknown request')
        return 0

    def bulk_write(self, dev_handle: MockDeviceHandle, ep: int, intf: int,
                   data: array, timeout: int) -> int:
        self.log.info('> write h:%d ep:%0x02x if:%d, d:%s, to:%d',
                      dev_handle.handle, ep, intf, hexlify(data).decode(),
                      timeout)
        if self._mpsse:
            self._mpsse.send(data)
        return len(data)

    def bulk_read(self, dev_handle: MockDeviceHandle, ep: int, intf: int,
                  buff: array, timeout: int) -> int:
        self.log.info('> read h:%d ep:0x%02x if:%d, l:%d, to:%d',
                      dev_handle.handle, ep, intf, len(buff), timeout)
        return 0

    def _ctrl_standard(self,
                       dev_handle: MockDeviceHandle,
                       bmRequestType: int,
                       bRequest: int,
                       wValue: int,
                       wIndex: int,
                       data: array,
                       timeout: int) -> int:
        req_ctrl = Constants.dec_req_ctrl(bmRequestType)
        req_type = Constants.dec_req_type(bmRequestType)
        req_rcpt = Constants.dec_req_rcpt(bmRequestType)
        req_desc = ':'.join([req_ctrl, req_type, req_rcpt])
        req_name = Constants.dec_req_name(bmRequestType, bRequest)
        dstr = (hexlify(data).decode() if Constants.is_req_out(bmRequestType)
                else f'({len(data)})')
        self.log.debug('> ctrl_transfer hdl %d, %s, %s, '
                       'val 0x%04x, idx 0x%04x, data %s, to %d',
                       dev_handle.handle, req_desc, req_name,
                       wValue, wIndex, dstr, timeout)
        size = 0
        if req_name == 'get_descriptor':
            desc_idx = wValue & 0xFF
            desc_type = wValue >> 8
            self.log.debug('  %s: 0x%02x',
                           Constants.dec_desc_type(desc_type), desc_idx)
            dev = dev_handle.device
            buf = dev.get_string(desc_type, desc_idx)
            size = len(buf)
            data[:size] = array('B', buf)
        else:
            self.log.warning('Unknown request')
        self.log.debug('< (%d) %s', size, hexlify(data[:size]).decode())
        return size

    def _ctrl_ftdi(self,
                   dev_handle: MockDeviceHandle,
                   bmRequestType: int,
                   bRequest: int,
                   wValue: int,
                   wIndex: int,
                   data: array,
                   timeout: int) -> int:
        req_ctrl = Constants.dec_req_ctrl(bmRequestType)
        req_type = Constants.dec_req_type(bmRequestType)
        req_rcpt = Constants.dec_req_rcpt(bmRequestType)
        req_desc = ':'.join([req_ctrl, req_type, req_rcpt])
        req_name = FtdiConst.dec_req_name(bRequest)
        dstr = (hexlify(data).decode() if Constants.is_req_out(bmRequestType)
                else f'({len(data)})')
        self.log.debug('> ctrl_ftdi hdl %d, %s, %s, '
                       'val 0x%04x, idx 0x%04x, data %s, to %d',
                       dev_handle.handle, req_desc, req_name,
                       wValue, wIndex, dstr, timeout)
        size = 0
        try:
            handler = getattr(self, f'_ctrl_ftdi_{req_name}')
        except AttributeError:
            self.log.warning('Unknown request: %s', req_name)
            return size
        buf = handler(wValue, wIndex, data) or b''
        size = len(buf)
        data[:size] = array('B', buf)
        self.log.debug('< (%d) %s', size, hexlify(data[:size]).decode())
        return size

    def _ctrl_ftdi_reset(self, wValue: int, wIndex: int,
                         data: array) -> None:
        reset = FtdiConst.translate('sio_reset', wValue)
        self.log.info('> ftdi reset %s', reset)

    def _ctrl_ftdi_set_bitmode(self, wValue: int, wIndex: int,
                               data: array) -> None:
        direction = wValue & 0xff
        bitmode = (wValue >> 8) & 0x7F
        mode = FtdiConst.translate('bitmode', bitmode)
        self.log.info('> ftdi bitmode %s: %s', mode, f'{direction:08b}')
        self._mpsse = FtdiMpsseTracer() if mode == 'mpsse' else None

    def _ctrl_ftdi_set_latency_timer(self, wValue: int, wIndex: int,
                                     data: array) -> None:
        self.log.info('> ftdi latency timer: %d', wValue)

    def _ctrl_ftdi_set_event_char(self, wValue: int, wIndex: int,
                                  data: array) -> None:
        char = wValue & 0xFF
        enable = bool(wValue >> 8)
        self.log.info('> ftdi %sable event char: 0x%02x',
                      'en' if enable else 'dis', char)

    def _ctrl_ftdi_set_error_char(self, wValue: int, wIndex: int,
                                  data: array) -> None:
        char = wValue & 0xFF
        enable = bool(wValue >> 8)
        self.log.info('> ftdi %sable error char: 0x%02x',
                      'en' if enable else 'dis', char)



Backend = MockBackend()
"""Unique instance of PyUSB mock backend."""


def get_backend(*args):
    """PyUSB API implementation."""
    return Backend
