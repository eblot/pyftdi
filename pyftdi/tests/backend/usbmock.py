"""PyUSB virtual USB backend to intercept all USB requests.

   The role of this module is to enable PyFtdi API testing w/o any FTDI
   hardware.
"""

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=invalid-name
#pylint: disable-msg=attribute-defined-outside-init
#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-many-arguments
#pylint: disable-msg=too-many-instance-attributes

from array import array
from binascii import hexlify
from collections import defaultdict
from functools import partial
from logging import getLogger
from struct import calcsize as scalc, pack as spack
from sys import version_info
from typing import List, Optional
from usb.backend import IBackend
from pyftdi.misc import EasyDict
from .consts import USBCONST
from .ftdimock import MockFtdi

# need support for f-string syntax
if version_info[:2] < (3, 6):
    raise AssertionError('Python 3.6 is required for this module')


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
            bDescriptorType=USBCONST.descriptors.ENDPOINT,
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

    def __getattr__(self, name):
        return getattr(self.desc, name)


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
            bDescriptorType=USBCONST.descriptors.INTERFACE,
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

    @property
    def endpoints(self):
        return self.altsettings[self.alt][1]

    def build_strings(self, func):
        for desc, _ in self.altsettings:
            func(desc)
        for _, endpoints in self.altsettings:
            for endpoint in endpoints:
                endpoint.build_strings(func)

    def add_bulk_pair(self):
        endpoints = self.altsettings[self.alt][1]
        desc = {
            'bEndpointAddress': len(endpoints)+1 | USBCONST.endpoints['in'],
            'bmAttributes': USBCONST.endpoint_types['bulk']
        }
        ep = MockEndpoint(desc)
        self.add_endpoint(ep)
        desc = {
            'bEndpointAddress': len(endpoints)+1 | USBCONST.endpoints['out'],
            'bmAttributes': USBCONST.endpoint_types['bulk']
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
            bDescriptorType=USBCONST.descriptors.CONFIG,
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
            bDescriptorType=USBCONST.descriptors.DEVICE,
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
        config.update()
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
        self.log = getLogger('pyftdi.mock.usb')
        self._devices = list()
        self._device_handles = dict()
        self._device_handle_count = 0
        self._ftdis = defaultdict(MockFtdi)

    def add_device(self, device: MockDevice):
        self._devices.append(device)

    def flush_devices(self):
        self._devices.clear()

    def create_loader(self) -> 'MockLooader':
        """Provide the loader class to configure this virtual backend instance.

           Using this method to retrieve a loader ensure both the virtual
           backend and the loader have been loaded from the same package.

           :return: the MockLoader class
        """
        # this is a bit circumvoluted, but modules cannot cross-reference
        loader_modname = '.'.join(__name__.split('.')[:-1] + ['loader'])
        loader_mod = import_module(loader_modname)
        MockLoader = getattr(loader_mod, 'MockLoader')
        return MockLoader

    @property
    def devices(self) -> List[MockDevice]:
        return self._devices

    def get_virtual_ftdi(self, bus: int, address: int) -> MockFtdi:
        for dev in self._devices:
            if dev.bus == bus and dev.address == address:
                return self._ftdis[(bus, address)]
        raise ValueError('No FTDI @ {bus:address}')

    def enumerate_devices(self) -> MockDevice:
        for dev in self._devices:
            yield dev

    def open_device(self, dev: MockDevice) -> MockDeviceHandle:
        self._device_handle_count += 1
        devhdl = MockDeviceHandle(dev, self._device_handle_count)
        self._device_handles[devhdl.handle] = devhdl
        return devhdl

    def close_device(self, dev_handle: MockDeviceHandle) -> None:
        del self._device_handles[dev_handle.handle]

    def claim_interface(self, dev_handle: MockDeviceHandle, intf: int) \
            -> None:
        self.log.info('> claim interface h:%d: if:%d',
                      dev_handle.handle, intf)

    def release_interface(self, dev_handle: MockDeviceHandle, intf: int) \
            -> None:
        self.log.info('> release interface h:%d: if:%d',
                      dev_handle.handle, intf)

    def get_configuration(self,
                          dev_handle: MockDeviceHandle) -> MockConfiguration:
        dev = dev_handle.device
        return dev.configurations[0]

    def set_configuration(self, dev_handle: MockDeviceHandle,
                          config_value: int) -> None:
        # config_value = ConfigDesc.bConfigurationValue
        pass

    def get_device_descriptor(self, dev: MockDevice) -> EasyDict:
        return dev.desc

    def get_configuration_descriptor(self, dev: MockDevice, config: int) \
            -> EasyDict:
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
        req_type = USBCONST.dec_req_type(bmRequestType)
        if req_type == 'standard':
            return self._ctrl_standard(dev_handle, bmRequestType, bRequest,
                                       wValue, wIndex, data, timeout)
        if req_type == 'vendor':
            ftdi = self._get_ftdi_from_handle(dev_handle)
            return ftdi.control(dev_handle, bmRequestType, bRequest,
                                wValue, wIndex, data, timeout)
        self.log.error('Unknown request')
        return 0

    def bulk_write(self, dev_handle: MockDeviceHandle, ep: int, intf: int,
                   data: array, timeout: int) -> int:
        self.log.debug('> write h:%d ep:%0x02x if:%d, d:%s, to:%d',
                       dev_handle.handle, ep, intf, hexlify(data).decode(),
                       timeout)
        ftdi = self._get_ftdi_from_handle(dev_handle)
        return ftdi.write(dev_handle, ep, intf, data, timeout)

    def bulk_read(self, dev_handle: MockDeviceHandle, ep: int, intf: int,
                  buff: array, timeout: int) -> int:
        self.log.debug('> read h:%d ep:0x%02x if:%d, l:%d, to:%d',
                       dev_handle.handle, ep, intf, len(buff), timeout)
        ftdi = self._get_ftdi_from_handle(dev_handle)
        return ftdi.read(dev_handle, ep, intf, buff, timeout)

    def _ctrl_standard(self,
                       dev_handle: MockDeviceHandle,
                       bmRequestType: int,
                       bRequest: int,
                       wValue: int,
                       wIndex: int,
                       data: array,
                       timeout: int) -> int:
        req_ctrl = USBCONST.dec_req_ctrl(bmRequestType)
        req_type = USBCONST.dec_req_type(bmRequestType)
        req_rcpt = USBCONST.dec_req_rcpt(bmRequestType)
        req_desc = ':'.join([req_ctrl, req_type, req_rcpt])
        req_name = USBCONST.dec_req_name(bmRequestType, bRequest)
        dstr = (hexlify(data).decode() if USBCONST.is_req_out(bmRequestType)
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
                           USBCONST.dec_desc_type(desc_type), desc_idx)
            dev = dev_handle.device
            buf = dev.get_string(desc_type, desc_idx)
            size = len(buf)
            data[:size] = array('B', buf)
        else:
            self.log.warning('Unknown request')
        self.log.debug('< (%d) %s', size, hexlify(data[:size]).decode())
        return size

    def _get_ftdi_from_handle(self, dev_handle: MockDeviceHandle) -> MockFtdi:
        bus = dev_handle.device.bus
        address = dev_handle.device.address
        return self._ftdis[(bus, address)]


_MockBackend = MockBackend()
"""Unique instance of PyUSB mock backend."""


def get_backend(*_):
    """PyUSB API implementation."""
    return _MockBackend
