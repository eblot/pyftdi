"""Miscelleanous helpers"""

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
