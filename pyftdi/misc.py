# Copyright (c) 2010-2020 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2008-2016, Neotion
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Neotion nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL NEOTION BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Miscelleanous helpers"""

from array import array
from copy import deepcopy
from re import match
from typing import Any, Iterable, Union

#pylint: disable-msg=invalid-name


# String values evaluated as true boolean values
TRUE_BOOLEANS = ['on', 'true', 'enable', 'enabled', 'yes', 'high', '1']
# String values evaluated as false boolean values
FALSE_BOOLEANS = ['off', 'false', 'disable', 'disabled', 'no', 'low', '0']
# ASCII or '.' filter
ASCIIFILTER = ''.join([((len(repr(chr(_x))) == 3) or (_x == 0x5c)) and chr(_x)
                       or '.' for _x in range(128)]) + '.' * 128
ASCIIFILTER = bytearray(ASCIIFILTER.encode('ascii'))


def hexdump(data: Union[bytes, bytearray, Iterable[int]],
            full: bool = False, abbreviate: bool = False) -> str:
    """Convert a binary buffer into a hexadecimal representation.

       Return a multi-line strings with hexadecimal values and ASCII
       representation of the buffer data.

       :param data: binary buffer to dump
       :param full: use `hexdump -Cv` format
       :param abbreviate: replace identical lines with '*'
       :return: the generated string
    """
    try:
        if isinstance(data, (bytes, array)):
            src = bytearray(data)
        elif not isinstance(data, bytearray):
            # data may be a list/tuple
            src = bytearray(b''.join(data))
        else:
            src = data
    except Exception:
        raise TypeError("Unsupported data type '%s'" % type(data))

    length = 16
    result = []
    last = b''
    abv = False
    for i in range(0, len(src), length):
        s = src[i:i+length]
        if abbreviate:
            if s == last:
                if not abv:
                    result.append('*\n')
                    abv = True
                continue
            else:
                abv = False
        hexa = ' '.join(["%02x" % x for x in s])
        printable = s.translate(ASCIIFILTER).decode('ascii')
        if full:
            hx1, hx2 = hexa[:3*8], hexa[3*8:]
            hl = length//2
            result.append("%08x  %-*s %-*s |%s|\n" %
                          (i, hl*3, hx1, hl*3, hx2, printable))
        else:
            result.append("%06x   %-*s  %s\n" %
                          (i, length*3, hexa, printable))
        last = s
    return ''.join(result)


def hexline(data: Union[bytes, bytearray, Iterable[int]],
            sep: str = ' ') -> str:
    """Convert a binary buffer into a hexadecimal representation.

       Return a string with hexadecimal values and ASCII representation
       of the buffer data.

       :param data: binary buffer to dump
       :param sep: the separator string/char
       :return: the formatted string
    """
    try:
        if isinstance(data, (bytes, array)):
            src = bytearray(data)
        elif not isinstance(data, bytearray):
            # data may be a list/tuple
            src = bytearray(b''.join(data))
        else:
            src = data
    except Exception:
        raise TypeError("Unsupported data type '%s'" % type(data))

    hexa = sep.join(["%02x" % x for x in src])
    printable = src.translate(ASCIIFILTER).decode('ascii')
    return "(%d) %s : %s" % (len(data), hexa, printable)


def to_int(value: Union[int, str]) -> int:
    """Parse a value and convert it into an integer value if possible.

       Input value may be:
       - a string with an integer coded as a decimal value
       - a string with an integer coded as a hexadecimal value
       - a integral value
       - a integral value with a unit specifier (kilo or mega)

       :param value: input value to convert to an integer
       :return: the value as an integer
       :rtype: int
       :raise ValueError: if the input value cannot be converted into an int
    """
    if not value:
        return 0
    if isinstance(value, int):
        return value
    mo = match(r'^\s*(\d+)\s*(?:([KMkm]i?)?B?)?\s*$', value)
    if mo:
        mult = {'K': (1000),
                'KI': (1 << 10),
                'M': (1000 * 1000),
                'MI': (1 << 20)}
        value = int(mo.group(1))
        if mo.group(2):
            value *= mult[mo.group(2).upper()]
        return value
    return int(value.strip(), value.startswith('0x') and 16 or 10)


def to_bool(value: Union[int, bool, str], permissive: bool = True,
            allow_int: bool = False) -> bool:
    """Parse a string and convert it into a boolean value if possible.

       Input value may be:
       - a string with an integer value, if `allow_int` is enabled
       - a boolean value
       - a string with a common boolean definition

       :param value: the value to parse and convert
       :param permissive: default to the False value if parsing fails
       :param allow_int: allow an integral type as the input value
       :raise ValueError: if the input value cannot be converted into an bool
    """
    if value is None:
        return False
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        if allow_int:
            return bool(value)
        else:
            if permissive:
                return False
            raise ValueError("Invalid boolean value: '%d'", value)
    if value.lower() in TRUE_BOOLEANS:
        return True
    if permissive or (value.lower() in FALSE_BOOLEANS):
        return False
    raise ValueError('"Invalid boolean value: "%s"' % value)


def to_bps(value: str) -> int:
    """Parse a string and convert it into a baudrate value.

       The function accepts common multipliers as K, M and G

       :param value: the value to parse and convert
       :type value: str or int or float
       :rtype: float
       :raise ValueError: if the input value cannot be converted into a float
    """
    if isinstance(value, float):
        return int(value)
    if isinstance(value, int):
        return value
    mo = match(r'^(?P<value>[-+]?[0-9]*\.?[0-9]+(?:[Ee][-+]?[0-9]+)?)'
               r'(?P<unit>[KkMmGg])?$', value)
    if not mo:
        raise ValueError('Invalid frequency')
    frequency = float(mo.group(1))
    if mo.group(2):
        mult = {'K': 1E3, 'M': 1E6, 'G': 1E9}
        frequency *= mult[mo.group(2).upper()]
    return int(frequency)


def xor(_a_: bool, _b_: bool) -> bool:
    """XOR logical operation.

       :param _a_: first argument
       :param _b_: second argument
       :return: xor-ed value
    """
    return bool((not(_a_) and _b_) or (_a_ and not(_b_)))


def is_iterable(obj: Any) -> bool:
    """Tells whether an instance is iterable or not.

       :param obj: the instance to test
       :type obj: object
       :return: True if the object is iterable
       :rtype: bool
    """
    try:
        iter(obj)
        return True
    except TypeError:
        return False


def pretty_size(size, sep: str = ' ',
                lim_k: int = 1 << 10, lim_m: int = 10 << 20,
                plural: bool = True, floor: bool = True) -> str:
    """Convert a size into a more readable unit-indexed size (KiB, MiB)

       :param size: integral value to convert
       :param sep: the separator character between the integral value and
            the unit specifier
       :param lim_k: any value above this limit is a candidate for KiB
            conversion.
       :param lim_m: any value above this limit is a candidate for MiB
            conversion.
       :param plural: whether to append a final 's' to byte(s)
       :param floor: how to behave when exact conversion cannot be
            achieved: take the closest, smaller value or fallback to the next
            unit that allows the exact representation of the input value
       :return: the prettyfied size
    """
    size = int(size)
    if size > lim_m:
        ssize = size >> 20
        if floor or (ssize << 20) == size:
            return '%d%sMiB' % (ssize, sep)
    if size > lim_k:
        ssize = size >> 10
        if floor or (ssize << 10) == size:
            return '%d%sKiB' % (ssize, sep)
    return '%d%sbyte%s' % (size, sep, (plural and 's' or ''))


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
