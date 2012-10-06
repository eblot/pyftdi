# Copyright (c) 2008-2012, Neotion
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

"""Miscelleanous helpers

"""

import binascii
import re

# String values evaluated as true boolean values
TRUE_BOOLEANS = ['on', 'true', 'enable', 'enabled', 'yes', 'high', '1']
# String values evaluated as false boolean values
FALSE_BOOLEANS = ['off', 'false', 'disable', 'disabled', 'no', 'low', '0']
# ASCII or '.' filter
ASCIIFILTER = ''.join([(len(repr(chr(_x)))==3) and chr(_x) or \
                    '.' for _x in range(256)])


def hexdump(data):
    """Convert a binary buffer into a hexadecimal representation.

       Return a multi-line strings with hexadecimal values and ASCII
       representation of the buffer data.
    """
    src = ''.join(data)
    length = 16
    result = []
    for i in xrange(0, len(src), length):
        s = src[i:i+length]
        hexa = ' '.join(["%02x" % ord(x) for x in s])
        printable = s.translate(ASCIIFILTER)
        result.append("%06x   %-*s   %s\n" % \
                      (i, length*3, hexa, printable))
    return ''.join(result)

def hexline(data, sep=' '):
    """Convert a binary buffer into a hexadecimal representation

       Return a string with hexadecimal values and ASCII representation
       of the buffer data
    """
    src = ''.join(data)
    hexa = sep.join(["%02x" % ord(x) for x in src])
    printable = src.translate(ASCIIFILTER)
    return "(%d) %s : %s" % (len(data), hexa, printable)

def to_int(value):
    """Parse a value and convert it into an integer value if possible.

       Input value may be:
       - a string with an integer coded as a decimal value
       - a string with an integer coded as a hexadecimal value
       - a integral value
       - a integral value with a unit specifier (kilo or mega)
    """
    if not value:
        return 0
    if isinstance(value, int):
        return value
    if isinstance(value, long):
        return int(value)
    mo = re.match('^\s*(\d+)\s*(?:([KMkm])(i?)B?)?\s*$', value)
    if mo:
        if mo.group(2) and not mo.group(3):
            unit = mo.group(2)
            import sys
            print >> sys.stderr, \
                "Obsolete unit %sB (%s), please use %siB" % \
                    (unit, value, unit.upper())
        mult = { 'K': (1<<10), 'M': (1<<20) }
        value = int(mo.group(1))
        value *= mo.group(2) and mult[mo.group(2).upper()] or 1
        return value
    return int(value.strip(), value.startswith('0x') and 16 or 10)

def to_bool(value, permissive=True, allow_int=False):
    """Parse a string and convert it into a boolean value if possible.

       :param value: the value to parse and convert
       :param permissive: default to the False value if parsing fails
       :param allow_int: allow an integral type as the input value

       Input value may be:
       - a string with an integer value, if `allow_int` is enabled
       - a boolean value
       - a string with a common boolean definition
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

def _crccomp():
    """Internal function used by crc16()"""
    try:
        from crcmod import mkCrcFun
    except ImportError:
        raise AssertionError("Python crcmod module not installed")
    crc_polynomial = 0x11021
    crc_initial = 0xFFFF
    crc = mkCrcFun(crc_polynomial, crc_initial, False)
    while True:
        yield crc

def crc16(data):
    """Compute the CCITT CRC-16 checksum"""
    crc = next(_crccomp())
    return crc(data)

def xor(_a_, _b_):
    """XOR operation"""
    return (not(_a_) and _b_) or (_a_ and not(_b_))

def is_iterable(obj):
    """Tells whether an instance is iterable or not"""
    try:
        iter(obj)
        return True
    except TypeError:
        return False

def pretty_size(size, sep=' ', lim_k=1<<10, lim_m=10<<20, plural=True,
                floor=True):
    """Convert a size into a more readable unit-indexed size (KiB, MiB)

       :param size:   integral value to convert
       :param sep:    the separator character between the integral value and
                      the unit specifier
       :param lim_k:  any value above this limit is a candidate for KiB
                      conversion.
       :param lim_m:  any value above this limit is a candidate for MiB
                      conversion.
       :param plural: whether to append a final 's' to byte(s)
       :param floor:  how to behave when exact conversion cannot be achieved:
                      take the closest, smaller value or fallback to the next
                      unit that allows the exact representation of the input
                      value
    """
    size = int(size)
    if size > lim_m:
        ssize = size>>20
        if floor or ssize<<20 == size:
            return '%d%sMiB' % (ssize, sep)
    if size > lim_k:
        ssize = size>>10
        if floor or ssize<<10 == size:
            return '%d%sKiB' % (ssize, sep)
    return '%d%sbyte%s' % (size, sep, (plural and 's' or ''))
