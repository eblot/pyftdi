from binascii import hexlify
from importlib import import_module
from logging import getLogger
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


class DummyDevice:
    """
    """


class DummyDeviceDescriptor(EasyDict):
    """
    """

    def __init__(self, **kwargs):
        super().__init__(bLength=0,
                         bDescriptorType=0,
                         bcdUSB=0x200,
                         bDeviceClass=0,
                         bDeviceSubClass=0,
                         bDeviceProtocol=0,
                         bMaxPacketSize0=8,
                         idVendor=0x403,
                         idProduct=0x6014,
                         bcdDevice=0x1000,
                         iManufacturer=1,
                         iProduct=2,
                         iSerialNumber=3,
                         bNumConfigurations=1,
                         **kwargs)


class DummyConfigDescriptor(EasyDict):
    """
    """

    def __init__(self, **kwargs):
        super().__init__(bLength=0,
                         bDescriptorType=0,
                         wTotalLength=0,
                         bNumInterfaces=0,
                         bConfigurationValue=0,
                         iConfiguration=0,
                         bmAttributes=0,
                         bMaxPower=0,
                         interface=0,  # ref interface
                         extra=b'',  # byte buffer
                         extra_length=0)
        self.extra_descriptors = self.extra[:self.extra_length]


class DummyDeviceHandle(EasyDict):

    def __init__(self, dev):
        super().__init__(handle=1,
                         devid=0)


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

        print(f'desc_type {self._desc_type}')
        print(f'desc_type_mask {self._desc_type_mask:02x}')
        print(f'ctrl {self._ctrl_dir}')
        print(f'ctrl_mask {self._ctrl_dir_mask:02x}')
        print(f'ctrl_type {self._ctrl_type}')
        print(f'ctrl_type_mask {self._ctrl_type_mask:02x}')
        print(f'ctrl_recipient {self._ctrl_recipient}')
        print(f'ctrl_recipient_mask {self._ctrl_recipient_mask:02x}')
        print(f'endpoint_type {self._endpoint_type}')
        print(f'endpoint_type_mask {self._endpoint_type_mask:02x}')

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
            mapping[entry[plen:]] = getattr(mod, entry)
        return mapping.mirror()

    @classmethod
    def _mask(cls, mapping: dict) -> int:
        mask = 0
        for val in mapping:
            mask |= val
        return mask

    def decode_request_type(self, reqtype: int) -> str:
        ctrl = self._ctrl_dir[reqtype & self._ctrl_dir_mask]
        type_ = self._ctrl_type[reqtype & self._ctrl_type_mask]
        rcpt = self._ctrl_recipient[reqtype & self._ctrl_recipient_mask]
        return ':'.join((ctrl, type_, rcpt)).lower()

    def decode_request(self, reqtype: int, request: int) -> str:
        direction = bool(reqtype & self._ctrl_dir_mask)
        try:
            return self.DEVICE_REQUESTS[(direction, request)]
        except KeyError:
            return f'req x{request:02x}'


class DummyBackend(IBackend):
    """
    """

    Decoder = UsbDecoder()

    def __init__(self):
        self._devices = [DummyDevice()]
        self.log = getLogger('pyftdi.tests.backend')

    def enumerate_devices(self):
        for dev in self._devices:
            yield dev

    def get_device_descriptor(self, dev):
        desc = DummyDeviceDescriptor(bus=1,
                                     address=1,
                                     speed=3,
                                     port_number=None,
                                     port_numbers=None)
        return desc

    def get_configuration_descriptor(self, dev, config):
        cfg = DummyConfigDescriptor()
        return cfg

    def open_device(self, dev):
        return DummyDeviceHandle(dev)

    def close_device(self, dev_handle):
        dev_handle.handle = None


    def ctrl_transfer(self,
                      dev_handle,
                      bmRequestType,
                      bRequest,
                      wValue,
                      wIndex,
                      data,
                      timeout):
        self.log.info('ctrl_transfer hdl %d, %s, %s, '
                      'val 0x%04x, idx 0x%04x, data %s, to %d',
                      dev_handle.handle,
                      self.Decoder.decode_request_type(bmRequestType),
                      self.Decoder.decode_request(bmRequestType, bRequest),
                      wValue,
                      wIndex,
                      hexlify(data).decode(),
                      timeout)
        return 0

def get_backend(*args):

    return DummyBackend()
