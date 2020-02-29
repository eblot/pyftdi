"""PyUSB virtual USB backend to intercept all USB requests.

   The role of this module is to enable PyFtdi API testing w/o any FTDI
   hardware.
"""

from array import array
from binascii import hexlify
from copy import deepcopy
from functools import partial
from importlib import import_module
from logging import getLogger
from struct import calcsize as scalc, pack as spack
from typing import BinaryIO, Optional
from ruamel.yaml import load_all as yaml_load
from ruamel.yaml.loader import Loader
from usb.backend import IBackend
from pyftdi.ftdi import Ftdi
from pyftdi.misc import to_bool
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

    def __init__(self, defs: dict,
                 extra: Optional[bytes] = None):
        class EndpointDescriptor(EasyDict):
            pass
        if extra and not isinstance(extra, (bytes, bytearray)):
            raise ValueError('Invalid extra payload')
        self.desc = EndpointDescriptor(
            bLength=scalc(self.DESCRIPTOR_FORMAT),
            bDescriptorType=Constants.descriptors.ENDPOINT,
            # bEndpointAddress=Constants.endpoints[direction.lower()] | index,
            # bmAttributes=Constants.endpoint_types[type_.lower()],
            bEndpointAddress=0,
            bmAttributes=0,
            wMaxPacketSize=64,
            bInterval=0,
            bRefresh=0,
            bSynchAddress=0,
            extra_descriptors=extra or b'')
        self.desc.update(defs)

    def build_strings(self, func):
        func(self.desc)

    def get_length(self) -> int:
        return self.desc.bLength


class MockInterface:
    """Fake USB configuration interface.
    """

    DESCRIPTOR_FORMAT = '<9B'

    def __init__(self, defs: dict, extra: Optional[bytes] = None):
        class InterfaceDescriptor(EasyDict):
            pass
        if extra and not isinstance(extra, (bytes, bytearray)):
            raise ValueError('Invalid extra payload')
        desc = InterfaceDescriptor(
            bLength=scalc(self.DESCRIPTOR_FORMAT),
            bDescriptorType=Constants.descriptors.INTERFACE,
            bInterfaceNumber=0,
            bAlternateSetting=0,
            bNumEndpoints=0,
            bInterfaceClass=0xFF,
            bInterfaceSubClass=0xFF,
            bInterfaceProtocol=0xFF,
            iInterface=0,  # String desc index
            extra_descriptors=extra or  b'')
        desc.update(defs)
        self.alt = 0
        self.altsettings = [(desc, [])]

    def add_endpoint(self, endpoint: MockEndpoint):
        altsetting = self.altsettings[self.alt]
        altsetting[1].append(endpoint)
        altsetting[0].bNumEndpoints = len(altsetting[1])

    def build_strings(self, func):
        for desc, _ in self.altsettings:
            func(desc)
        for _, endpoints in self.altsettings:
            for endpoint in endpoints:
                endpoint.build_strings(func)

    def add_bulk_pair(self):
        endpoints = self.altsettings[self.alt][1]
        desc = {
            'bEndpointAddress': len(endpoints)+1 | Constants.endpoints['in'],
            'bmAttributes': Constants.endpoint_types['bulk']
        }
        ep = MockEndpoint(desc)
        self.add_endpoint(ep)
        desc = {
            'bEndpointAddress': len(endpoints)+1 | Constants.endpoints['out'],
            'bmAttributes': Constants.endpoint_types['bulk']
        }
        ep = MockEndpoint(desc)
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

    def __init__(self, defs: dict, extra: Optional[bytes] = None):
        class ConfigDescriptor(EasyDict):
            pass
        if extra and not isinstance(extra, (bytes, bytearray)):
            raise ValueError('Invalid extra payload')
        self.desc = ConfigDescriptor(
            bLength=scalc(self.DESCRIPTOR_FORMAT),
            bDescriptorType=Constants.descriptors.CONFIG,
            wTotalLength=0,
            bNumInterfaces=0,
            bConfigurationValue=0,
            iConfiguration=0, # string index
            bmAttributes=0x80,  # bus-powered
            bMaxPower=150//2,  # 150 mA
            extra_descriptors=extra or  b'')
        self.desc.update(defs)
        self.interfaces = []

    def add_interface(self, interface: MockInterface):
        interface.bInterfaceNumber = len(self.interfaces)
        self.interfaces.append(interface)
        self.desc.bNumInterfaces = len(self.interfaces)

    def build_strings(self, func):
        print(f'build strings on {self.desc}')
        func(self.desc)
        for iface in self.interfaces:
            iface.build_strings(func)

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

    def __init__(self, defs: dict, **kwargs):
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
            idVendor=0,
            idProduct=0,
            bcdDevice=0,
            iManufacturer=0,
            iProduct=0,
            iSerialNumber=0,
            bNumConfigurations=0,  # updated later
            port_number=None,  # unsupported
            port_numbers=None,  # unsupported
            bus=0,
            address=0,
            speed=3)
        self.desc.update(defs)
        for key in kwargs:
            if key not in defs:
                self.desc[key] = kwargs[key]
        self.configurations = []
        self.strings = ['']  # slot 0 is reserved

    def add_configuration(self, config: MockConfiguration):
        self.configurations.append(config)
        self.desc.bNumConfigurations = len(self.configurations)

    def build(self):
        func = partial(MockDevice._store_strings, self)
        self.build_strings(func)

    def build_strings(self, func):
        func(self.desc)
        for config in self.configurations:
            config.build_strings(func)

    @staticmethod
    def _store_strings(obj, desc):
        for dkey in sorted(desc):
            if isinstance(desc[dkey], str):
                stridx = len(obj.strings)
                obj.strings.append(desc[dkey])
                desc[dkey] = stridx

    def get_string(self, type_: int, index: int) -> str:
        if index == 0:
            if self.desc.get('noaccess', False):
                # simulate unauthorized access to the USB device
                return b''
            # request for list of supported languages
            # only support one
            fmt = '<BBH'
            size = scalc(fmt)
            buf = spack(fmt, size, type_, self.DEFAULT_LANGUAGE)
            return buf
        try:
            value = self.strings[index]
        except IndexError:
            return b''
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
        self._mpsse = None

    def add_device(self, device: MockDevice):
        self._devices.append(device)

    def flush_devices(self):
        self._devices.clear()

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


class MockLoader:
    """Load a virtual USB bus environment.
    """

    def __init__(self):
        self.log = getLogger('pyftdi.mock.backend')

    def load(self, yamlfp: BinaryIO) -> None:
        """Load a YaML configuration stream.

           :param yamlfp: YaML stream to be parsed
        """
        with yamlfp:
            ydefs = yaml_load(yamlfp, Loader=Loader)
            try:
                for ydef in ydefs:
                    self._build_root(ydef)
            except Exception as exc:
                raise ValueError(f'Invalid configuration: {exc}')

    def _build_root(self, container):
        _Backend.flush_devices()
        if not isinstance(container, dict):
            raise ValueError('Top-level not a dict')
        for ykey, yval in container.items():
            if ykey != 'devices':
                continue
            if not isinstance(yval, list):
                raise ValueError('Devices not a list')
            for yitem in yval:
                if not isinstance(container, dict):
                    raise ValueError('Device not a dict')
                device = self._build_device(yitem)
                device.build()
                _Backend.add_device(device)

    def _build_device(self, container):
        devdesc = None
        configs = []
        properties = {}
        for ykey, yval in container.items():
            if ykey == 'descriptor':
                if not isinstance(yval, dict):
                    raise ValueError('Device descriptor not a dict')
                devdesc = self._build_device_descriptor(yval)
                continue
            if ykey == 'configurations':
                if not isinstance(yval, list):
                    raise ValueError('Configurations not a list')
                configs = [self._build_configuration(conf) for conf in yval]
                continue
            if ykey == 'noaccess':
                yval = to_bool(yval)
            properties[ykey] = yval
        if not devdesc:
            raise ValueError('Missing device descriptor')
        if not configs:
            raise ValueError('Missing device config')
        device = MockDevice(devdesc, **properties)
        for config in configs:
            device.add_configuration(config)
        return device

    def _build_device_descriptor(self, container) -> dict:
        kmap = {
            'usb': 'bcdUSB',
            'class': 'bDeviceClass',
            'subclass': 'bDeviceSubClass',
            'protocol': 'bDeviceProtocol',
            'maxpacketsize': 'bMaxPacketSize0',
            'vid': 'idVendor',
            'pid': 'idProduct',
            'version': 'bcdDevice',
            'manufacturer': 'iManufacturer',
            'product': 'iProduct',
            'serialnumber': 'iSerialNumber',
        }
        kwargs = {}
        for ckey, cval in container.items():
            try:
                dkey = kmap[ckey]
            except KeyError:
                raise ValueError(f'Unknown descriptor field {dkey}')
            kwargs[dkey] = cval
        return kwargs

    def _build_configuration(self, container):
        if not isinstance(container, dict):
            raise ValueError('Invalid configuration entry')
        cfgdesc = None
        interfaces = []
        for ykey, yval in container.items():
            if ykey == 'descriptor':
                if not isinstance(yval, dict):
                    raise ValueError('Configuration descriptor not a dict')
                cfgdesc = self._build_config_descriptor(yval)
                continue
            if ykey == 'interfaces':
                if not isinstance(yval, list):
                    raise ValueError('Configurations not a list')
                interfaces = [self._build_interface(conf) for conf in yval]
                continue
            raise ValueError(f'Unknown config entry {ykey}')
        if not cfgdesc:
            raise ValueError('Missing config descriptor')
        if not interfaces:
            raise ValueError('Missing config interface')
        config = MockConfiguration(cfgdesc)
        for iface in interfaces:
            config.add_interface(iface)
        return config

    def _build_config_descriptor(self, container) -> dict:
        kmap = {
            'attributes': 'bmAttributes',
            'maxpower': 'bMaxPower',
            'configuration': 'iConfiguration'
        }
        kwargs = {}
        for ckey, cval in container.items():
            try:
                dkey = kmap[ckey]
            except KeyError:
                raise ValueError(f'Unknown descriptor field {ckey}')
            if ckey == 'maxpower':
                cval //= 2
            elif ckey == 'attributes':
                if not isinstance(cval, list):
                    raise ValueError('Invalid config attributes')
                aval = 0x80
                for feature in cval:
                    if feature == 'selfpowered':
                        aval |= 1 << 6
                    if feature == 'wakeup':
                        aval |= 1 << 5
                cval = aval
            elif ckey == 'configuration':
                pass
            else:
                raise ValueError(f'Unknown config descriptor {ckey}')
            kwargs[dkey] = cval
        return kwargs

    def _build_interface(self, container):
        if not isinstance(container, dict):
            raise ValueError('Invalid interface entry')
        alternatives = []
        for ikey, ival in container.items():
            if ikey != 'alternatives':
                raise ValueError(f'Invalid interface entry {ikey}')
            if not isinstance(ival, list):
                raise ValueError(f'Invalid interface entry {ikey}')
            alternatives.extend([self._build_alternative(alt) for alt in ival])
        if len(alternatives) != 1:
            raise ValueError('Unsupported alternative count')
        ifdesc, endpoints = alternatives[0]
        iface = MockInterface(ifdesc)
        for endpoint in endpoints:
            iface.add_endpoint(endpoint)
        return iface

    def _build_alternative(self, container):
        if not isinstance(container, dict):
            raise ValueError('Invalid alternative entry')
        ifdesc = None
        endpoints = []
        for ikey, ival in container.items():
            if ikey == 'descriptor':
                if not isinstance(ival, dict):
                    raise ValueError('Interface descriptor not a dict')
                ifdesc = self._build_interface_descriptor(ival)
                continue
            if ikey == 'endpoints':
                if not isinstance(ival, list):
                    raise ValueError('Interface encpoints not a list')
                endpoints = [self._build_endpoint(ep) for ep in ival]
        if not ifdesc:
            raise ValueError('Missing interface descriptor')
        if not endpoints:
            raise ValueError('Missing interface endpoint')
        return ifdesc, endpoints

    def _build_interface_descriptor(self, container) -> dict:
        kmap = {
            'class': 'bDeviceClass',
            'subclass': 'bDeviceSubClass',
            'protocol': 'bDeviceProtocol',
            'interface': 'iInterface',
        }
        kwargs = {}
        for ckey, cval in container.items():
            try:
                dkey = kmap[ckey]
            except KeyError:
                raise ValueError(f'Unknown descriptor field {ckey}')
            kwargs[dkey] = cval
        return kwargs

    def _build_endpoint(self, container):
        if not isinstance(container, dict):
            raise ValueError('Invalid endpoint entry')
        epdesc = None
        for ikey, ival in container.items():
            if ikey == 'descriptor':
                if not isinstance(ival, dict):
                    raise ValueError('Interface descriptor not a dict')
                epdesc = self._build_endpoint_descriptor(ival)
                continue
            raise ValueError(f'Unknown config entry {ikey}')
        if not epdesc:
            raise ValueError('Missing endpoint descriptor')
        endpoint = MockEndpoint(epdesc)
        return endpoint

    def _build_endpoint_descriptor(self, container) -> dict:
        kwargs = {}
        for ekey, val in container.items():
            if ekey == 'maxpacketsize':
                kwargs['wMaxPacketSize'] = val
                continue
            if ekey == 'interval':
                kwargs['bInterval'] = val
                continue
            if ekey == 'direction':
                try:
                    value = Constants.endpoints[val.lower()]
                except KeyError:
                    raise ValueError('Unknown endpoint direction')
                kwargs.setdefault('bEndpointAddress', 0)
                kwargs['bEndpointAddress'] |= value
                continue
            if ekey == 'number':
                if not isinstance(val, int) or not 0 < val < 16:
                    raise ValueError('Invalid endpoint number')
                kwargs.setdefault('bEndpointAddress', 0)
                kwargs['bEndpointAddress'] |= val
                continue
            if ekey == 'type':
                try:
                    kwargs['bmAttributes'] = \
                        Constants.endpoint_types[val.lower()]
                except KeyError:
                    raise ValueError('Unknown endpoint type')
                continue
            if ekey == 'endpoint':
                kwargs['iEndpoint'] = val
                continue
            raise ValueError(f'Unknown endpoint entry {ekey}')
        return kwargs


global _Backend
_Backend = MockBackend()
"""Unique instance of PyUSB mock backend."""


def get_backend(*args):
    """PyUSB API implementation."""
    return _Backend
