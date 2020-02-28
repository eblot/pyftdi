from array import array
from binascii import hexlify
from copy import deepcopy
from enum import IntEnum
from importlib import import_module
from logging import getLogger
from struct import calcsize as scalc, pack as spack
from usb.backend import IBackend


class EasyDict(dict):
    """Dictionary whose members can be accessed as instance members
    """

    def __init__(self, dictionary=None, **kwargs):
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


class DummyDevice(EasyDict):
    """
    """


class DummyDeviceDescriptor(EasyDict):
    """
    """

    DEFAULT_LANGUAGE = 0x0409  # en_US

    STRINGS = IntEnum('Strings', 'MANUFACTURER PRODUCT SERIAL_NUMBER', start=1)

    def __init__(self, **kwargs):
        super().__init__(bLength=18,
                         bDescriptorType=0x01,
                         bcdUSB=0x200,
                         bDeviceClass=0xff,
                         bDeviceSubClass=0,
                         bDeviceProtocol=0,
                         bMaxPacketSize0=8,
                         idVendor=0x403,
                         idProduct=0x6014,
                         bcdDevice=0x1000,
                         iManufacturer=self.STRINGS.MANUFACTURER,
                         iProduct=self.STRINGS.PRODUCT,
                         iSerialNumber=self.STRINGS.SERIAL_NUMBER,
                         bNumConfigurations=1,
                         **kwargs)

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
        print(name, value)
        ms_str = value.encode('utf-16-le')
        fmt = '<BB'
        size = scalc(fmt) + len(ms_str)
        buf = bytearray(spack('<BB', size, type_))
        buf.extend(ms_str)
        return buf


class DummyConfigDescriptor(EasyDict):
    """
    """

    FMT = '<2BH5B'

    def __init__(self, **kwargs):
        super().__init__(bLength=scalc(self.FMT),
                         bDescriptorType=0x02,
                         wTotalLength=0,  # to compute
                         bNumInterfaces=1,
                         bConfigurationValue=0,
                         iConfiguration=0,
                         bmAttributes=0x80,  # bus-powered
                         bMaxPower=150//2,  # 150 mA
                         interface=0,  # ref interface
                         extra=b'',  # byte buffer
                         extra_length=0)
        self.extra_descriptors = self.extra[:self.extra_length]


class DummyDeviceHandle(EasyDict):

    def __init__(self, dev, handle):
        super().__init__(handle=handle,
                         devid=0,
                         device=dev)


class UsbDecoder:

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

    @classmethod
    def _load_constants(cls, prefix: str):
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
            mapping[getattr(mod, entry)] = entry[plen:].lower()
        return mapping

    @classmethod
    def _mask(cls, mapping: dict) -> int:
        mask = 0
        for val in mapping:
            mask |= val
        return mask

    def is_req_out(self, reqtype: int) -> str:
        return not (reqtype & self._ctrl_dir_mask)

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


Decoder = UsbDecoder()

class DummyBackend(IBackend):
    """
    """

    def __init__(self):
        self._devices = list()
        self._device_handles = dict()
        self._device_handle_count = 0
        self.log = getLogger('pyftdi.tests.backend')
        dev = DummyDevice()
        desc = DummyDeviceDescriptor(bus=1,
                                     address=1,
                                     speed=3,
                                     port_number=1,
                                     port_numbers=1,
                                     serial_number='SN_1234',
                                     product='FT232H')
        dev.device_descriptor = desc
        self._devices.append(dev)


    def enumerate_devices(self):
        for dev in self._devices:
            yield dev

    def get_device_descriptor(self, dev):
        return dev.device_descriptor

    def get_configuration_descriptor(self, dev, config):
        cfg = DummyConfigDescriptor()
        return cfg

    def open_device(self, dev: DummyDevice) -> DummyDeviceHandle:
        self._device_handle_count += 1
        devhdl = DummyDeviceHandle(dev, self._device_handle_count)
        self._device_handles[devhdl.handle] = devhdl
        return devhdl

    def close_device(self, dev_handle: DummyDeviceHandle) -> None:
        del self._device_handles[dev_handle.handle]

    def ctrl_transfer(self,
                      dev_handle,
                      bmRequestType,
                      bRequest,
                      wValue,
                      wIndex,
                      data,
                      timeout):
        req_ctrl = Decoder.dec_req_ctrl(bmRequestType)
        req_type = Decoder.dec_req_type(bmRequestType)
        req_rcpt = Decoder.dec_req_rcpt(bmRequestType)
        req_desc = ':'.join([req_ctrl, req_type, req_rcpt])
        req_name = Decoder.dec_req_name(bmRequestType, bRequest)
        dstr = (hexlify(data).decode() if Decoder.is_req_out(bmRequestType)
                else f'({len(data)})')
        self.log.info('> ctrl_transfer hdl %d, %s, %s, '
                      'val 0x%04x, idx 0x%04x, data %s, to %d',
                      dev_handle.handle, req_desc, req_name,
                      wValue, wIndex, dstr, timeout)
        size = 0
        if req_name == 'get_descriptor':
            desc_idx = wValue & 0xFF
            desc_type = wValue >> 8
            self.log.info('  %s: 0x%02x',
                          Decoder.dec_desc_type(desc_type), desc_idx)
            devdesc = dev_handle.device.device_descriptor
            buf = devdesc.get_string(desc_type, desc_idx)
            size = len(buf)
            data[:size] = array('B', buf)
        else:
            self.log.warning('Unknown request')
        self.log.info('< (%d) %s', size, hexlify(data[:size]).decode())
        return size

    def _get_devhandle_by_handle(self, handle: int) -> DummyDeviceHandle:
        return self._device_handles[handle]

    def _get_device_by_handle(self, handle: int) -> DummyDevice:
        devhdl = self._get_devhandle_by_handle(handle)
        return devhdl.device


def get_backend(*args):

    return DummyBackend()
