"""Bit field and sequence management"""

from numbers import Integral
from util import xor

__all__ = ['BitSequence', 'BitZSequence', 'BitError', 'BitField']

# Hints for PyLint:
#   use map(), use short variable names
#pylint: disable-msg=W0141
#pylint: disable-msg=C0103


class BitError(Exception):
    """Bit sequence error"""
    pass


class BitSequence(object):
    """Bit sequence manipulation"""

    def __init__(self, value=None, msb=False, length=0, bytes_=None, msby=True):
        self._seq = []
        if value and bytes_:
            raise BitError("Cannot inialize with both a value and bytes_")
        if bytes_:
            provider = msby and list(bytes_).__iter__() or reversed(bytes_)
            for byte in provider:
                if isinstance(byte, str):
                    byte = ord(byte)
                elif byte > 0xff:
                    raise BitError("Invalid byte value")
                b = []
                for x in xrange(8):
                    b.append(byte&0x1 and True or False)
                    byte >>= 1
                if msb:
                    b.reverse()
                self._seq.extend(b)
        else:
            value = self._tomutable(value)
        if isinstance(value, Integral):
            self._init_from_integer(value, msb, length)
        elif isinstance(value, list):
            self._init_from_sequence(value, msb)
        elif isinstance(value, BitSequence):
            self._init_from_sibling(value, msb)
        elif value is None:
            pass
        else:
            raise BitError("Cannot initialize from a %s" % type(value))
        self._update_length(length, msb)

    @staticmethod
    def _tomutable(value):
        """Convert a immutable sequence into a mutable one"""
        if isinstance(value, tuple):
            # convert immutable sequence into a list so it can be popped out
            value = list(value)
        elif isinstance(value, str):
            # convert immutable sequence into a list so it can be popped out
            if value.startswith('0b'):
                value = list(value[2:])
            else:
                value = list(value)
        return value

    def _init_from_integer(self, value, msb, length):
        """Initialize from any integer value"""
        l = length or -1
        while l:
            self._seq.append((value & 1) and True or False)
            value >>= 1
            if not value:
                break
            l -= 1
        if msb:
            self._seq.reverse()

    def _init_from_sequence(self, value, msb):
        """Initialize from a Python sequence"""
        pos = msb and -1 or 0
        smap = { '0': False, '1': True, False: False, True: True }
        while value:
            try:
                bit = value.pop(pos)
                self._seq.append(smap[bit])
            except KeyError:
                print value
                raise BitError("Invalid binary character: '%s'" % bit)

    def _init_from_sibling(self, value, msb):
        """Initialize from a fellow object"""
        self._seq = value.sequence()
        if msb:
            self._seq.reverse()

    def _update_length(self, length, msb):
        """If a specific length is specified, extend the sequence as
           expected"""
        if length and (len(self) < length):
            extra = [False] * (length-len(self))
            if msb:
                self._seq = extra + self._seq
            else:
                self._seq.extend(extra)

    def sequence(self):
        """Return the internal representation as a new mutable sequence"""
        return list(self._seq)

    def reverse(self):
        """In-place reverse"""
        self._seq.reverse()
        return self

    def invert(self):
        """In-place invert sequence values"""
        self._seq = [not x for x in self._seq]
        return self

    def append(self, seq):
        """Concatenate a new BitSequence"""
        if not isinstance(seq, BitSequence):
            seq = BitSequence(seq)
        self._seq.extend(seq.sequence())
        return self

    def lsr(self, count):
        "Left shift rotate"
        count %= len(self)
        self._seq[:] = self._seq[count:] + self._seq[:count]

    def rsr(self, count):
        "Right shift rotate"
        count %= len(self)
        self._seq[:] = self._seq[-count:] + self._seq[:-count]

    def tobit(self):
        """Degenerate the sequence into a single bit, if possible"""
        if len(self) != 1:
            raise BitError("BitSequence should be a scalar")
        return self._seq[0]

    def tobyte(self, msb=False):
        """Convert the sequence into a single byte value, if possible"""
        if len(self) > 8:
            raise BitError("Cannot fit into a single byte")
        byte = 0
        pos = not msb and -1 or 0
        # copy the sequence
        seq = self._seq[:]
        while seq:
            byte <<= 1
            byte |= seq.pop(pos)
        return byte

    def tobytes(self, msb=False, msby=False):
        """Convert the sequence into a sequence of byte values"""
        blength = (len(self)+7) & (~0x7)
        sequence = list(self._seq)
        if not msb:
            sequence.reverse()
        bytes_ = []
        for pos in xrange(0, blength, 8):
            seq = sequence[pos:pos+8]
            byte = 0
            while seq:
                byte <<= 1
                byte |= seq.pop(0)
            bytes_.append(byte)
        if msby:
            bytes_.reverse()
        return bytes_

    def __iter__(self):
        return self._seq.__iter__()

    def __reversed__(self):
        return self._seq.__reversed__()

    def __getitem__(self, index):
        if isinstance(index, slice):
            return self.__class__(value=self._seq[index])
        else:
            return self._seq[index]

    def __setitem__(self, index, value):
        if not isinstance(value, BitSequence):
            value = self.__class__(value)
        else:
            if issubclass(value.__class__, self.__class__) and \
               value.__class__ != self.__class__:
                raise BitError("Cannot set item with instance of a subclass")
        if isinstance(index, slice):
            self._seq[index] = value.sequence()
        else:
            val = value.tobit()
            if len(self._seq) < index:
                # auto-resize sequence
                extra = [False] * (index+1-len(self))
                self._seq.extend(extra)
            self._seq[index] = val

    def __len__(self):
        return len(self._seq)

    def __cmp__(self, other):
        # the bit sequence should be of the same length
        ld = len(self) - len(other)
        if ld:
            return ld
        for n, (x, y) in enumerate(zip(self._seq, other.sequence()), start=1):
            if xor(x, y):
                return n
        return 0

    def __repr__(self):
        # cannot use bin() as it truncates the MSB zero bits
        return ''.join([b and '1' or '0' for b in reversed(self._seq)])

    def __str__(self):
        chunks = []
        srepr = repr(self)
        length = len(self)
        for i in xrange(0, length, 8):
            if i:
                j = -i
            else:
                j = None
            chunks.append(srepr[-i-8:j])
        return '%d: %s' % (len(self), ' '.join(reversed(chunks)))

    def __int__(self):
        return int(long(self))

    def __long__(self):
        value = 0
        for b in reversed(self._seq):
            value <<= 1
            value |= b and 1
        return value

    def __and__(self, other):
        if type(other) is not type(self.__class__()):
            raise BitError('Need a BitSequence to combine')
        if len(self) != len(other):
            raise BitError('Sequences must be the same size')
        return self.__class__(value=map(lambda x, y: x and y,
                                        self._seq, other.sequence()))

    def __or__(self, other):
        if type(other) is not type(self.__class__()):
            raise BitError('Need a BitSequence to combine')
        if len(self) != len(other):
            raise BitError('Sequences must be the same size')
        return self.__class__(value=map(lambda x, y: x or y,
                                        self._seq, other.sequence()))

    def __add__(self, other):
        return self.__class__(value = self._seq + other.sequence())


class BitZSequence(BitSequence):
    """Tri-state bit sequence manipulation"""

    def __init__(self, value=None, msb=False, length=0):
        BitSequence.__init__(self, value=value, msb=msb, length=length)

    def _init_from_sequence(self, value, msb):
        """Initialize from a Python sequence"""
        smap = { '0': False, '1': True, 'Z': None,
                 False: False, True: True, None: None }
        pos = msb and -1 or 0
        while value:
            try:
                bit = value.pop(pos)
                self._seq.append(smap[bit])
            except KeyError:
                raise BitError("Invalid binary character: '%s'" % bit)

    def __repr__(self):
        smap = { False: '0', True: '1', None: 'Z' }
        return ''.join([smap[b] for b in reversed(self._seq)])

    def __long__(self):
        if None in self._seq:
            raise BitError("Sequence cannot be converted to Integer")
        return BitSequence.__long__(self)

    def __int__(self):
        return int(long(self))

    def invert(self):
        def invz(val):
            """Compute the inverted value of a tristate Boolean"""
            if val is None:
                return None
            else:
                return not val
        self._seq = [invz(x) for x in self._seq]
        return self

    def tobyte(self, msb=False):
        raise BitError("Type %s cannot be converted to byte" % type(self))

    def tobytes(self, msb=False, msby=False):
        raise BitError("Type %s cannot be converted to bytes" % type(self))

    def matches(self, other):
        # the bit sequence should be of the same length
        ld = len(self) - len(other)
        if ld:
            return ld
        for (x, y) in zip(self._seq, other.sequence()):
            if None in (x, y):
                continue
            if not x is y:
                return False
        return True

    def __cmp__(self, other):
        # the bit sequence should be of the same length
        ld = len(self) - len(other)
        if ld:
            return ld
        for n, (x, y) in enumerate(zip(self._seq, other.sequence()), start=1):
            if not x is y:
                return n
        return 0

    def __and__(self, other):
        if not isinstance(self, BitSequence):
            raise BitError('Need a BitSequence-compliant object to combine')
        if len(self) != len(other):
            raise BitError('Sequences must be the same size')
        def andz(x, y):
            """Compute the boolean AND operation for a tri-state boolean"""
            if None in (x, y):
                return None
            else:
                return x and y
        return self.__class__(value=map(andz, self._seq, other.sequence()))

    def __or__(self, other):
        if not isinstance(self, BitSequence):
            raise BitError('Need a BitSequence-compliant object to combine')
        if len(self) != len(other):
            raise BitError('Sequences must be the same size')
        def orz(x, y):
            """Compute the boolean OR operation for a tri-state boolean"""
            if None in (x, y):
                return None
            else:
                return x or y
        return self.__class__(value=map(orz, self._seq, other.sequence()))

    def __rand__(self, other):
        return self.__and__(other)

    def __ror__(self, other):
        return self.__or__(other)

    def __radd__(self, other):
        return self.__class__(value=other) + self


class BitField(object):
    """Bitfield manipulation
       Beware the slices does not behave as regular Python slices:
       bitfield[3:5] means b3..b5, NOT b3..b4 as with regular slices
    """

    def __init__(self, value=0):
        self._val = value

    def __getitem__(self, index):
        if isinstance(index, slice):
            if index.stop == index.start:
                return
            if index.stop < index.start:
                offset = index.stop
                count = index.start-index.stop+1
            else:
                offset = index.start
                count = index.stop-index.start+1
            mask = (1<<count)-1
            return (self._val >> offset) & mask
        else:
            return (self._val >> index) & 1

    def __setitem__(self, index, value):
        if isinstance(index, slice):
            if index.stop == index.start:
                return
            if index.stop < index.start:
                offset = index.stop
                count = index.start-index.stop+1
            else:
                offset = index.start
                count = index.stop-index.start+1
            mask = (1<<count)-1
            value = (value & mask) << offset
            mask = mask << offset
            self._val = (self._val & ~mask) | value
        else:
            value = (value&1L)<<index
            mask = (1L)<<index
            self._val = (self._val & ~mask) | value

    def __int__(self):
        return self._val

    def to_seq(self, msb=0, lsb=0):
        """Return the BitFiled as a sequence of boolean value"""
        seq = []
        count = 0
        value = self._val
        while value:
            count += 1
            value >>= 1
        for x in xrange(lsb, max(msb, count)):
            seq.append(bool((self._val >> x) & 1))
        return tuple(reversed(seq))
