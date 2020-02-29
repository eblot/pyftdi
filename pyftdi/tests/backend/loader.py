"""Virtual USB backend loader.
"""

from logging import getLogger
from typing import BinaryIO
from ruamel.yaml import load_all as yaml_load
from ruamel.yaml.loader import Loader
from pyftdi.misc import to_bool
from .usbmock import (MockConfiguration, MockDevice, MockInterface,
                      MockEndpoint, get_backend)
from .consts import USBCONST

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=too-few-public-methods


class MockLoader:
    """Load a virtual USB bus environment from a YaML description stream.
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
        backend = get_backend()
        backend.flush_devices()
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
                backend.add_device(device)

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
                    value = USBCONST.endpoints[val.lower()]
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
                        USBCONST.endpoint_types[val.lower()]
                except KeyError:
                    raise ValueError('Unknown endpoint type')
                continue
            if ekey == 'endpoint':
                kwargs['iEndpoint'] = val
                continue
            raise ValueError(f'Unknown endpoint entry {ekey}')
        return kwargs
