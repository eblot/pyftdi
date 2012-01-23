# Copyright (c) 2008-2011, Neotion
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
    """Convert a binary buffer into a hexadecimal representation."""
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

def hexline(data):
    """Convert a binary buffer into a hexadecimal representation"""
    src = ''.join(data)
    hexa = ' '.join(["%02x" % ord(x) for x in src])
    printable = src.translate(ASCIIFILTER)
    return "(%d) %s : %s" % (len(data), hexa, printable)

def to_int(value):
    """Parse a string and convert it into a value"""
    if not value:
        return 0
    if isinstance(value, int):
        return value
    if isinstance(value, long):
        return int(value)
    mo = re.match('(?i)^\s*(\d+)\s*(?:([KM])B?)?\s*$', value)
    if mo:
        mult = { 'k': (1<<10), 'm': (1<<20) }
        value = int(mo.group(1))
        value *= mo.group(2) and mult[mo.group(2).lower()] or 1
        return value
    return int(value.strip(), value.startswith('0x') and 16 or 10)

def to_bool(value, permissive=True, allow_int=False):
    """Parse a string and convert it into a boolean value"""
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
