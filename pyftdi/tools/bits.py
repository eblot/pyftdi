# Copyright (c) 2010-2024 Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Bit sequence helpers for JTAG.

   BitSequence handle bit manipulation for the JTAG tools.
"""

from typing import Any, Iterable, List, Union


class BitSequenceError(Exception):
    """Bit sequence error.

       >>> raise BitSequenceError('msg')
       Traceback (most recent call last):
       ...
       BitSequenceError: msg
    """


BitSequenceInitializer = Union['BitSequence', str, int, bytes, bytearray,
                               Iterable[int], Iterable[bool], None]
"""Supported types to initialize a BitSequence."""


class BitSequence:
    """Bit sequence.

       Support most of the common bit operations and conversion from and to
       integral values, as well as sequence of boolean, int and characters.

       Bit sequence objects are iterable.

       :param value:  initial value
       :param width: count of signficant bits in the bit sequence

       >>> BitSequence()
       []
       >>> BitSequence([False, True])
       [0, 1]
       >>> BitSequence([0, 1, 1])
       [0, 1, 1]
       >>> BitSequence(BitSequence([0, 1, 1]), 5)
       [0, 0, 0, 1, 1]
       >>> BitSequence('100')
       [1, 0, 0]
       >>> BitSequence(b'101')
       [1, 0, 1]
       >>> BitSequence(0xC, 4)
       [1, 1, 0, 0]
       >>> BitSequence(0xC, 3)
       Traceback (most recent call last):
       ...
       ValueError: Value cannot be stored in specified width
       >>> BitSequence(BitSequence([1, 1, 0, 0]), 3)
       Traceback (most recent call last):
       ...
       ValueError: Specified width too short
    """

    # pylint: disable=protected-access

    def __init__(self, value: BitSequenceInitializer = None,
                 width: int = 0):
        """
        >>> BitSequence()
        []
        >>> BitSequence(0, 3)
        [0, 0, 0]
        """
        if value is None:
            self._int = 0
            self._width = width
            return
        if isinstance(value, BitSequence):
            if width:
                if value._width > width:
                    raise ValueError('Specified width too short')
                width = max(width, value._width)
            else:
                width = value._width
            self._int, self._width = value._int, width
            return
        if isinstance(value, int):
            if value >= 1 << width:
                raise ValueError('Value cannot be stored in specified width')
            bseq = self.from_int(value, width)
            self._int, self._width = bseq._int, bseq._width
            return
        if is_iterable(value):
            bseq = self.from_iterable(value)
            if width and width != bseq._width:
                raise ValueError('Specified width does not match input value')
            self._int, self._width = bseq._int, bseq._width
            return
        raise BitSequenceError(f'Cannot initialize from a {type(value)}')

    @classmethod
    def from_iterable(cls, iterable: Iterable) -> 'BitSequence':
        """Instanciate a BitSequence from an iterable.
           Each element of the iterable should represent a single bit.

           See BitSequence.from_bytes to create a BitSequence for a byte stream.

        >>> BitSequence('11000100')
        [1, 1, 0, 0, 0, 1, 0, 0]
        >>> BitSequence(b'11000100')
        [1, 1, 0, 0, 0, 1, 0, 0]
        >>> BitSequence(b'\\x01\\x01\\x00\\x00\\x00\\x01\\x000')
        [1, 1, 0, 0, 0, 1, 0, 0]
        >>> BitSequence([1, 1, 0, 0, 0, 1, 0, 0])
        [1, 1, 0, 0, 0, 1, 0, 0]
        """
        # pylint: disable=duplicate-key
        smap = {
            0: 0, 1: 1,  # as int
            '0': 0, '1': 1,  # as string
            0x30: 0, 0x31: 1,  # as bytes
            False: 0, True: 1,  # as bool
        }
        value = 0
        width = 0
        for bit in iterable:
            try:
                value <<= 1
                value |= smap[bit]
                width += 1
            except KeyError as exc:
                raise ValueError(f"Invalid item value '{bit}' in iterable at "
                                 f"pos {width}") from exc
        return BitSequence.from_int(value, width)

    @classmethod
    def from_int(cls, value: int, width: int) -> 'BitSequence':
        """Instanciate a BitSequence from an integer value.

        >>> BitSequence(0xC4, 8)
        [1, 1, 0, 0, 0, 1, 0, 0]
        >>> BitSequence(-1, 9)
        [1, 1, 1, 1, 1, 1, 1, 1, 1]
        """
        bseq = BitSequence()
        bseq._int = value & ((1 << width) - 1)
        bseq._width = width
        return bseq

    @classmethod
    def from_bytes(cls, value: Union[bytes | bytearray]) -> 'BitSequence':
        """Instanciate a BitSequence from a sequence of bytes, one bit for each
           input byte.

        >>> BitSequence(b'\\x01\\x01\\x00\\x00\\x00\\x01\\x000')
        [1, 1, 0, 0, 0, 1, 0, 0]
        """
        return cls.from_iterable(value)

    @classmethod
    def from_bytestream(cls, value: Union[bytes | bytearray],
                        lsbyte: bool = False) -> 'BitSequence':
        """Instanciate a BitSequence from a sequence of bytes, 8 bits for each
           input byte.

        >>> BitSequence.from_bytestream(b'\\xca\\xfe')
        [1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0]
        >>> BitSequence.from_bytestream(b'\\xca\\xfe', True)
        [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0]
        """
        bseq = BitSequence(0, 8 * len(value))
        if lsbyte:
            for byte in reversed(value):
                bseq._int <<= 8
                bseq._int |= byte
        else:
            for byte in value:
                bseq._int <<= 8
                bseq._int |= byte
        return bseq

    def __len__(self):
        """
        >>> len(BitSequence())
        0
        >>> len(BitSequence(0, 1))
        1
        >>> len(BitSequence('010'))
        3
        """
        return self._width

    def __bool__(self) -> bool:
        """Report whether BitSequence is empty or not.

           To fold a single bit sequence into a boolean, see #to_bit

        >>> bool(BitSequence())
        False
        >>> bool(BitSequence(0, 1))
        True
        >>> bool(BitSequence('010'))
        True
        """
        return bool(self._width)

    def __int__(self) -> int:
        """Convert BitSequence to an int.

        >>> int(BitSequence())
        0
        >>> int(BitSequence(0, 1))
        0
        >>> int(BitSequence('010'))
        2
        """
        return self._int

    def __repr__(self) -> str:
        """
        >>> repr(BitSequence(0xC4, 8))
        '[1, 1, 0, 0, 0, 1, 0, 0]'
        """
        sseq = ', '.join((f'{b:0d}' for b in self))
        return f'[{sseq}]'

    def __str__(self) -> str:
        """
        >>> str(BitSequence(0xC4, 8))
        '11000100'
        """
        return f'{self._int:0{self._width}b}'

    @property
    def mask(self) -> int:
        """Bit mask.

        >>> BitSequence(0xC4, 8).mask
        255
        >>> BitSequence().mask
        0
        """
        return (1 << self._width) - 1

    def to_bit(self) -> bool:
        """Fold the sequence into a single bit, if possible.

        >>> BitSequence([0]).to_bit()
        False
        >>> BitSequence([1]).to_bit()
        True
        >>> BitSequence([0, 0]).to_bit()
        Traceback (most recent call last):
        ...
        BitSequenceError: BitSequence too large
        """
        if self._width != 1:
            raise BitSequenceError("BitSequence too large")
        return bool(self._int & 1)

    def to_byte(self, lsb: bool = False) -> int:
        """Convert the sequence into a single byte value, if possible.

        >>> hex(BitSequence(0xC4, 8).to_byte())
        '0xc4'
        >>> hex(BitSequence(0xC4, 8).to_byte(True))
        '0x23'
        """
        if self._width > 8:
            raise BitSequenceError("Cannot fit into a single byte")
        if lsb:
            bseq = BitSequence(self)
            bseq.reverse()
        else:
            bseq = self
        return bseq._int

    def to_bytes(self) -> bytes:
        """Return the internal representation as bytes.

        >>> BitSequence(0xC4, 8).to_bytes()
        b'\\x01\\x01\\x00\\x00\\x00\\x01\\x00\\x00'
        """
        return bytes((int(b) for b in self))

    def to_bool_list(self) -> List[bool]:
        """Convert the sequence into a list of boolean values.

        >>> BitSequence(0xC4, 8).to_bool_list()
        [True, True, False, False, False, True, False, False]
        """
        return list(self)

    def to_bytestream(self, msb: bool = False, msby: bool = False) -> bytes:
        """Convert the sequence into a sequence of byte values.

        >>> from binascii import hexlify

        >>> hexlify(BitSequence(0xC4A5D0, 24).to_bytestream(True, True))
        b'c4a5d0'
        >>> hexlify(BitSequence(0xC4A5D0, 24).to_bytestream(True, False))
        b'd0a5c4'
        >>> hexlify(BitSequence(0xC4A5D0, 24).to_bytestream(False, True))
        b'23a50b'
        >>> hexlify(BitSequence(0xC4A5D0, 24).to_bytestream(False, False))
        b'0ba523'
        """
        out: List[int] = []
        bseq = BitSequence(self)
        if not msb:
            bseq.reverse()
        while bseq._width:
            out.append(bseq._int & 0xff)
            bseq._int >>= 8
            bseq._width -= 8
        if not msby ^ msb:
            out.reverse()
        return bytes(out)

    def resize(self, width: int) -> 'BitSequence':
        """Change the width of the BitSequence.
           Truncate the stored bits of new width is shorter.

           :return: self

        >>> bs1 = BitSequence(0xC4, 8)

        >>> bs1.resize(10)
        [0, 0, 1, 1, 0, 0, 0, 1, 0, 0]
        >>> bs1.resize(5)
        [0, 0, 1, 0, 0]
        """
        if width > self._width:
            self._int &= (1 << width) - 1
        self._width = width
        return self

    def copy(self, reverse=False) -> 'BitSequence':
        """Duplicate bitsequence.

        >>> bs1 = BitSequence(0xC4, 8)

        >>> bs2 = bs1.copy()

        >>> bs1 == bs2
        True
        >>> id(bs1) == id(bs2)
        False
        >>> bs1._int = 0

        >>> bs1 == bs2
        False
        >>> int(bs1)
        0
        >>> int(bs2)
        196
        >>> BitSequence(0b11000100, 8).copy(True)
        [0, 0, 1, 0, 0, 0, 1, 1]
        """
        if not reverse:
            bseq = self.__class__()
            bseq._int = self._int
            bseq._width = self._width
            return bseq
        return self.__class__.from_iterable(reversed(self))

    def reverse(self) -> 'BitSequence':
        """In-place reverse.

           :return: self

        >>> BitSequence(0b11000100, 8).reverse()
        [0, 0, 1, 0, 0, 0, 1, 1]
        """
        bseq = self.__class__.from_iterable(reversed(self))
        assert bseq._width == self._width
        self._int = bseq._int
        return self

    def invert(self) -> 'BitSequence':
        """In-place invert of each sequence value.

           :return: self

        >>> BitSequence(0b11000100, 8).invert()
        [0, 0, 1, 1, 1, 0, 1, 1]
        """
        self._int = ~self._int & self.mask
        return self

    def push_right(self, *bseq: BitSequenceInitializer) -> 'BitSequence':
        """Push a bit sequence to the right side.

           :param bseq: the bit sequence to push
           :return: self

        >>> BitSequence(0b11000100, 8).push_right(0b110, 3)
        [1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0]
        """
        bseq = BitSequence(*bseq)
        self._int <<= len(bseq)
        self._int |= bseq._int
        self._width += len(bseq)
        return self

    def push_left(self, *bseq: BitSequenceInitializer) -> 'BitSequence':
        """Push a bit sequence to the left side.

           :param bseq: the bit sequence to push
           :return: self
        >>> BitSequence(0b11000100, 8).push_left(BitSequence(0b110, 3))
        [1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0]
        """
        bseq = BitSequence(*bseq)
        self._int = (bseq._int << self._width) | self._int
        self._width += len(bseq)
        return self

    def pop_right(self, count: int = 1) -> 'BitSequence':
        """Pop bits from the right side.

           :param count: how many bits to pop
           :return: popped bits a a new BitSequence

        >>> bs = BitSequence(0b11000100, 8)

        >>> bs.pop_right(2)
        [0, 0]

        >>> bs
        [1, 1, 0, 0, 0, 1]
        """
        if count > self._width:
            raise ValueError('Count too large')
        if count == 0:
            return BitSequence()
        if count < 0:
            raise ValueError('Negative shift is not defined')
        bseq = self.__class__.from_int(self._int & ((1 << count) - 1), count)
        self._int >>= count
        self._width -= count
        return bseq

    def pop_left(self, count: int = 1) -> 'BitSequence':
        """Pop bits from the left side.

           :param count: how many bits to pop
           :return: popped bits a a new BitSequence

        >>> bs = BitSequence(0b11000100, 8)

        >>> bs.pop_left(2)
        [1, 1]

        >>> bs
        [0, 0, 0, 1, 0, 0]
        """
        if count > self._width:
            raise ValueError('Count too large')
        if count == 0:
            return BitSequence()
        if count < 0:
            raise ValueError('Negative shift is not defined')
        shift = self._width - count
        bseq = self.__class__.from_int(self._int >> shift, count)
        self._int &= (1 << shift) - 1
        self._width -= count
        return bseq

    def pop_right_bit(self) -> bool:
        """Pop a single bit from the right side.

           :return: popped bit
        >>> bs = BitSequence(0b11000100, 8)

        >>> bs.pop_right_bit()
        False

        >>> bs
        [1, 1, 0, 0, 0, 1, 0]
        """
        if self._width == 0:
            raise RuntimeError('Empty bit sequence')
        bit = bool(self._int & 1)
        self._int >>= 1
        self._width -= 1
        return bit

    def pop_left_bit(self) -> bool:
        """Pop a single bit from the left side.

           :return: popped bit
        >>> bs = BitSequence(0b11000100, 8)

        >>> bs.pop_left_bit()
        True

        >>> bs
        [1, 0, 0, 0, 1, 0, 0]
        """
        if self._width == 0:
            raise RuntimeError('Empty bit sequence')
        bit = bool(self._int >> (self._width - 1))
        self._width -= 1
        self._int &= self.mask
        return bit

    def rll(self, count: int = 1) -> 'BitSequence':
        """Rotate Left Logical.

           :return: self

        >>> BitSequence(0b11000100, 8).rll(3)
        [0, 0, 1, 0, 0, 1, 1, 0]
        """
        count %= self._width
        bseq = self.pop_left(count)
        self.push_right(bseq)
        return self

    def rrl(self, count: int = 1) -> 'BitSequence':
        """Rotate Right Logical.

           :return: self

        >>> BitSequence(0b11000100, 8).rrl(3)
        [1, 0, 0, 1, 1, 0, 0, 0]
        """
        count %= self._width
        bseq = self.pop_right(count)
        self.push_left(bseq)
        return self

    def inc(self, wrap: bool = True) -> 'BitSequence':
        """Increment the sequence.

        >>> BitSequence(0b11000100, 8).inc()
        [1, 1, 0, 0, 0, 1, 0, 1]
        """
        self._int += 1
        if not wrap:
            if self._int >> self._width:
                self._width += 1
        else:
            self._int &= self.mask
        return self

    def dec(self, wrap: bool = True) -> 'BitSequence':
        """Decrement the sequence.

        >>> BitSequence(0b11000100, 8).dec()
        [1, 1, 0, 0, 0, 0, 1, 1]
        """
        self._int -= 1
        if not wrap:
            if not self._int >> self._width:
                self._width -= 1
        else:
            self._int &= self.mask
        return self

    def invariant(self) -> bool:
        """Tells whether all bits of the sequence are of the same value.

           :return: the value
           :raise ValueError: if the bits are not of the same value

        >>> BitSequence(0b111, 3).invariant()
        True
        >>> BitSequence(0b000, 3).invariant()
        False
        >>> BitSequence(0b010, 3).invariant()
        Traceback (most recent call last):
        ...
        BitSequenceError: Bits do no match
        """
        if self._int == 0:
            return False
        if self._int == self.mask:
            return True
        raise BitSequenceError('Bits do no match')

    class Iterator:
        """BitSequence iterator.

           Iterate from left to right (MSB to LSB) if reverse is not set.

           :param bseq: the BitSequence to iterate
           :param reverse: whether to create a reverse iterator

           >>> for b in BitSequence(): pass

        """

        def __init__(self, bseq: 'BitSequence', reverse: bool = False):
            """
            >>> bsit = BitSequence.Iterator(BitSequence())

            >>> is_iterable(bsit)
            True
            """
            self._bseq = bseq
            self._reverse = reverse
            self._width = bseq._width
            self._pos = 0

        def __iter__(self) -> 'BitSequence.Iterator':
            """
            >>> bsit = BitSequence.Iterator(BitSequence())

            >>> iter(bsit) == bsit
            True
            """
            return self

        def __next__(self) -> bool:
            """
            >>> bsit = BitSequence.Iterator(BitSequence([0]))

            >>> next(bsit)
            False

            >>> next(bsit)
            Traceback (most recent call last):
            ...
            StopIteration
            """
            if self._width != self._bseq._width:
                raise RuntimeError('BitSequence modified while iterating')
            if self._pos >= self._bseq._width:
                raise StopIteration()
            if self._reverse:
                bit = bool((self._bseq._int >> self._pos) & 1)
            else:
                pos = self._width - self._pos - 1
                bit = bool((self._bseq._int >> pos) & 1)
            self._pos += 1
            return bit

    def __iter__(self) -> 'BitSequence.Iterator':
        """Iterate from left to right, i.e. MSB to LSB.

           >>> [not b for b in BitSequence(0b11000100, 8)]
           [False, False, True, True, True, False, True, True]
        """
        return self.__class__.Iterator(self)

    def __reversed__(self):
        """Iterate from right to left, i.e. LSB to MSB.

           >>> [not b for b in reversed(BitSequence(0b11000100, 8))]
           [True, True, False, True, True, True, False, False]
        """
        return self.__class__.Iterator(self, reverse=True)

    def __eq__(self, other: 'BitSequence') -> bool:
        """
        >>> bs1 = BitSequence(0b10011, 5)  # 10011

        >>> bs2 = BitSequence(0b11001, 5)  # 11001

        >>> bs3 = bs2.copy().reverse()

        >>> bs1 == bs3
        True
        >>> bs1 == bs2
        False
        """
        if not isinstance(other, self.__class__):
            raise ValueError(f'Cannot compare with {type(other)}')
        return self._cmp(other) == 0

    def __ne__(self, other: 'BitSequence') -> bool:
        """
        >>> BitSequence(0b10011, 5) != BitSequence(0b11001, 5)
        True
        """
        if not isinstance(other, self.__class__):
            raise ValueError(f'Cannot compare with {type(other)}')
        return not self == other

    def __le__(self, other: 'BitSequence') -> bool:
        """
        >>> BitSequence(0b10011, 5) <= BitSequence(0b11001, 5)
        True
        >>> BitSequence(0b11001, 5) <= BitSequence(0b11001, 5)
        True
        """
        if not isinstance(other, self.__class__):
            raise ValueError(f'Cannot compare with {type(other)}')
        return self._cmp(other) <= 0

    def __lt__(self, other: 'BitSequence') -> bool:
        """
        >>> BitSequence(0b10011, 5) < BitSequence(0b11001, 5)
        True
        >>> BitSequence(0b11001, 5) < BitSequence(0b11001, 5)
        False
        """
        if not isinstance(other, self.__class__):
            raise ValueError(f'Cannot compare with {type(other)}')
        return self._cmp(other) < 0

    def __ge__(self, other: 'BitSequence') -> bool:
        """
        >>> BitSequence(0b11001, 5) >= BitSequence(0b10011, 5)
        True
        >>> BitSequence(0b11001, 5) >= BitSequence(0b11001, 5)
        True
        """
        if not isinstance(other, self.__class__):
            raise ValueError(f'Cannot compare with {type(other)}')
        return self._cmp(other) >= 0

    def __gt__(self, other: 'BitSequence') -> bool:
        """
        >>> BitSequence(0b11001, 5) > BitSequence(0b10011, 5)
        True
        >>> BitSequence(0b11001, 5) > BitSequence(0b11001, 5)
        False
        """
        if not isinstance(other, self.__class__):
            raise ValueError(f'Cannot compare with {type(other)}')
        return self._cmp(other) > 0

    def _cmp(self, other: 'BitSequence') -> int:
        # the bit sequence should be of the same length
        """
        >>> BitSequence(0b11001, 5)._cmp(BitSequence(0b10011, 5))
        6
        >>> BitSequence(0b11001, 5)._cmp(BitSequence(0b11001, 6))
        -1
        >>> BitSequence(0b10011, 5)._cmp(BitSequence(0b11001, 5))
        -6
        >>> BitSequence(0b11001, 5)._cmp(BitSequence(0b11001, 5))
        0
        """
        wdiff = self._width - other._width
        if wdiff:
            return wdiff
        return self._int - other._int

    def __and__(self, other: 'BitSequence') -> 'BitSequence':
        """
        >>> BitSequence(0b10011, 5) & BitSequence(0b11001, 5)
        [1, 0, 0, 0, 1]
        """
        if not isinstance(other, self.__class__):
            raise ValueError('Need a BitSequence to combine')
        if self._width != other._width:
            raise ValueError('Sequences must be the same size')
        value = self._int & other._int
        return self.__class__.from_int(value, self._width)

    def __or__(self, other: 'BitSequence') -> 'BitSequence':
        """
        >>> BitSequence(0b10011, 5) | BitSequence(0b11001, 5)
        [1, 1, 0, 1, 1]
        """
        if not isinstance(other, self.__class__):
            raise ValueError('Need a BitSequence to combine')
        if self._width != other._width:
            raise ValueError('Sequences must be the same size')
        value = self._int | other._int
        return self.__class__.from_int(value, self._width)

    def __xor__(self, other: 'BitSequence') -> 'BitSequence':
        """
        >>> BitSequence(0b10011, 5) ^ BitSequence(0b11001, 5)
        [0, 1, 0, 1, 0]
        """
        if not isinstance(other, self.__class__):
            raise ValueError('Need a BitSequence to combine')
        if self._width != other._width:
            raise ValueError('Sequences must be the same size')
        value = self._int ^ other._int
        return self.__class__.from_int(value, self._width)

    def __iand__(self, other: 'BitSequence') -> 'BitSequence':
        """
        >>> bs = BitSequence(0b10011, 5)

        >>> bs &= BitSequence(0b11001, 5)

        >>> bs
        [1, 0, 0, 0, 1]
        """
        if not isinstance(other, self.__class__):
            raise ValueError('Need a BitSequence to combine')
        if self._width != other._width:
            raise ValueError('Sequences must be the same size')
        self._int &= other._int
        return self

    def __ior__(self, other: 'BitSequence') -> 'BitSequence':
        """
        >>> bs = BitSequence(0b10011, 5)

        >>> bs |= BitSequence(0b11001, 5)

        >>> bs
        [1, 1, 0, 1, 1]
        """
        if not isinstance(other, self.__class__):
            raise ValueError('Need a BitSequence to combine')
        if self._width != other._width:
            raise ValueError('Sequences must be the same size')
        self._int |= other._int
        return self

    def __ixor__(self, other: 'BitSequence') -> 'BitSequence':
        """
        >>> bs = BitSequence(0b10011, 5)

        >>> bs ^= BitSequence(0b11001, 5)

        >>> bs
        [0, 1, 0, 1, 0]
        """
        if not isinstance(other, self.__class__):
            raise ValueError('Need a BitSequence to combine')
        if self._width != other._width:
            raise ValueError('Sequences must be the same size')
        self._int ^= other._int
        return self

    def __invert__(self) -> 'BitSequence':
        """
        >>> ~BitSequence(0b10011, 5)
        [0, 1, 1, 0, 0]
        """
        value = (~self._int) & self.mask
        return self.__class__.from_int(value, self._width)

    def __ilshift__(self, count) -> 'BitSequence':
        """
        >>> bs = BitSequence(0b10011, 5)

        >>> bs <<= 1

        >>> bs
        [0, 0, 1, 1, 0]
        """
        value = (self._int << count) & self.mask
        return self.__class__.from_int(value, self._width)

    def __irshift__(self, count) -> 'BitSequence':
        """
        >>> bs = BitSequence(0b10011, 5)

        >>> bs >>= 1

        >>> bs
        [0, 1, 0, 0, 1]
        """
        value = (self._int >> count) & self.mask
        return self.__class__.from_int(value, self._width)

    def __iadd__(self, other) -> 'BitSequence':
        """
        >>> bs = BitSequence(0b10011, 5)

        >>> bs += BitSequence([0, 1, 0])

        >>> len(bs)
        8

        >>> bs
        [1, 0, 0, 1, 1, 0, 1, 0]
        """
        if not isinstance(other, self.__class__):
            raise TypeError(f"unsupported operand type(s) for +=: "
                            f"'{self.__class__.__name__}' and "
                            f"'{other.__class__.__name__}'")
        self._width += other._width
        self._int <<= other._width
        self._int |= other._int
        return self

    def __getitem__(self, index) -> 'BitSequence':
        """
        >>> bs = BitSequence(0b11000100, 8)

        >>> bs[0]
        [1]
        >>> bs[1]
        [1]
        >>> bs[2]
        [0]
        >>> bs[-3]
        [1]
        >>> bs[-2]
        [0]
        >>> bs[-1]
        [0]
        >>> bs[0:5]
        [1, 1, 0, 0, 0]
        >>> bs[-4:-1]
        [0, 1, 0]
        >>> bs[:-2]
        [1, 1, 0, 0, 0, 1]
        >>> bs[::-1]
        [0, 0, 1, 0, 0, 0, 1, 1]
        >>> bs[-5::-2]
        [0, 1]
        """
        if isinstance(index, slice):
            bits: List[int] = []
            length = self._width
            start = index.start
            stop = index.stop
            step = index.step or 1
            if start is None:
                start = 0 if step > 0 else length
            elif start < 0:
                start = max(0, length+start)
            if stop is None:
                stop = length if step > 0 else 0
            elif stop < 0:
                stop = min(length+stop, length)
            off = -1 if step > 0 else 0
            for bpos in range(start, stop, step):
                bits.append((self._int >> (self._width - bpos + off)) & 1)
            return self.__class__.from_iterable(bits)
        if not isinstance(index, int):
            raise TypeError(f'{self.__class__.__name__} indices must be '
                            f'integers or slices, not {type(index)}')
        if ~index >= self._width:
            raise IndexError(f'{self.__class__.__name__} index out of range')
        if index >= 0:
            value = (self._int >> (self._width - index - 1)) & 1
        else:
            value = (self._int >> (- index - 1)) & 1
        return self.__class__.from_int(value, 1)

    def __setitem__(self, index, value) -> None:
        """
        >>> bs = BitSequence(0b11000100, 8)

        >>> bs[-1] = True

        >>> bs
        [1, 1, 0, 0, 0, 1, 0, 1]
        >>> bs[1] = 0

        >>> bs
        [1, 0, 0, 0, 0, 1, 0, 1]
        >>> bs[8] = 0
        Traceback (most recent call last):
        ...
        IndexError: BitSequence index out of range
        >>> bs[2:4] = [1, 1]

        >>> bs
        [1, 0, 1, 1, 0, 1, 0, 1]
        >>> bs[7] = False

        >>> bs
        [1, 0, 1, 1, 0, 1, 0, 0]

        >>> bs[-3:] = [0, 1, 1]

        #>> bs
        #[1, 0, 1, 1, 0, 0, 1, 1]

        >>> bs = BitSequence(0, 8)

        >>> bs[-5::-2] = [1, 1]

        >>> bs
        [0, 1, 0, 1, 0, 0, 0, 0]

        >>> bs[-5::-2] = [1, 1, 1]
        Traceback (most recent call last):
        ...
        ValueError: attempt to assign sequence of size 3 to ext. slice of size 2
        """
        if isinstance(index, slice):
            if not isinstance(value, BitSequence):
                if not is_iterable(value):
                    raise TypeError(f'Cannot set item with {type(value)}')
                value = self.from_iterable(value)
            else:
                value = value.copy()
            length = self._width
            start = index.start
            stop = index.stop
            step = index.step or 1
            if start is None:
                start = 0 if step > 0 else length
            elif start < 0:
                start = max(0, length+start)
            if stop is None:
                stop = length if step > 0 else 0
            elif stop < 0:
                stop = min(length+stop, length)
            slen = len(range(start, stop, step))
            vlen = len(value)
            if slen != len(value):
                raise ValueError(f'attempt to assign sequence of size {vlen} '
                                 f'to ext. slice of size {slen}')
            nint = self._int
            for bpos in range(start, stop, step):
                pos = length - 1 - bpos
                if value.pop_left_bit():
                    nint |= 1 << pos
                else:
                    nint &= ~(1 << pos)
            if value:
                raise ValueError('Slice assignment does not support width '
                                 'extension')
            self._int = nint
            return
        if not isinstance(index, int):
            raise TypeError(f'{self.__class__.__name__} indices must be '
                            f'integers or slices, not {type(index)}')
        if isinstance(value, BitSequence):
            value = value.tobit()
        elif isinstance(value, int):
            if value not in (0, 1):
                raise ValueError('Invalid value')
        if index >= 0:
            bpos = self._width - index - 1
        else:
            bpos = - index - 1
        if not 0 <= bpos < self._width:
            raise IndexError(f'{self.__class__.__name__} index out of range')
        if bool(value):
            self._int |= 1 << bpos
        else:
            self._int &= ~(1 << bpos)


def is_iterable(obj: Any) -> bool:
    """Tells whether an instance is iterable or not.

       :param obj: the instance to test
       :type obj: object
       :return: True if the object is iterable
       :rtype: bool

    >>> is_iterable(None)
    False
    >>> is_iterable(42)
    False
    >>> is_iterable((None,))
    True
    >>> is_iterable('abc')
    True
    >>> is_iterable([])
    True
    >>> is_iterable({})
    True
    >>> is_iterable(set())
    True
    >>> is_iterable(is_iterable)
    False
    """
    try:
        iter(obj)
        return True
    except TypeError:
        return False


if __name__ == "__main__":
    import doctest
    doctest.testmod()
