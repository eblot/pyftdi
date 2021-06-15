"""Constant importer from existing modules."""

# Copyright (c) 2020-2021, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=invalid-name
#pylint: disable-msg=too-many-instance-attributes

from enum import Enum
from importlib import import_module
from sys import version_info
from pyftdi.ftdi import Ftdi
from pyftdi.misc import EasyDict


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
        self.speeds = self._load_constants('speed', True)

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
        if not mapping:
            raise ValueError(f"No USB constant found for {prefix.rstrip('_')}")
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
        self._dcache = {}
        self._rcache = {}

    @classmethod
    def _load_constants(cls, prefix: str, reverse=False):
        prefix = prefix.upper()
        if not prefix.endswith('_'):
            prefix = f'{prefix}_'
        mapping = EasyDict()
        plen = len(prefix)
        for name in dir(Ftdi):
            if not name.startswith(prefix):
                continue
            if not reverse:
                mapping[getattr(Ftdi, name)] = name[plen:].lower()
            else:
                mapping[name[plen:].lower()] = getattr(Ftdi, name)
        if not mapping:
            # maybe an enum
            prefix = prefix.rstrip('_').lower()
            for name in dir(Ftdi):
                if not name.lower().startswith(prefix):
                    continue
                item = getattr(Ftdi, name)
                if issubclass(item, Enum):
                    if not reverse:
                        mapping = {en.value: en.name.lower() for en in item}
                    else:
                        mapping = {en.name.lower(): en.value for en in item}
        if not mapping:
            raise ValueError(f"No FTDI constant found for "
                             f"{prefix.rstrip('_')}")
        return mapping

    def get_name(self, prefix: str, value: int) -> str:
        if prefix not in self._dcache:
            self._dcache[prefix] = self._load_constants(prefix)
        try:
            return self._dcache[prefix][value]
        except KeyError:
            return f'x?{value:04x}'

    def get_value(self, prefix: str, name: str) -> str:
        if prefix not in self._rcache:
            self._rcache[prefix] = self._load_constants(prefix, True)
        try:
            return self._rcache[prefix][name.lower()]
        except KeyError as exc:
            raise ValueError(f'Unknown name {prefix}.{name}') from exc

    def dec_req_name(self, request: int) -> str:
        return self.get_name('sio_req', request)


USBCONST = UsbConstants()
"""Unique instance of USB constant container."""

FTDICONST = FtdiConstants()
"""Unique instances of FTDI constant container."""
