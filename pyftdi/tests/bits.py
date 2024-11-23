#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""BitSequence unit tests."""

# Copyright (c) 2010-2024 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2010-2016, Neotion
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# pylint: disable=broad-except
# pylint: disable=invalid-name
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring

from sys import modules
from unittest import TestCase, TestLoader, TestSuite, main as ut_main
from pyftdi.bits import BitSequence, BitZSequence, BitSequenceError


class BitSequenceTestCase(TestCase):

    def setUp(self):
        self.bs1 = BitSequence(0x01, msb=True, length=8)
        self.bs2 = BitSequence(0x02, msb=True, length=8)
        self.bs3 = BitSequence(0x04, msb=True, length=7)
        self.bs4 = BitSequence(0x04, msb=True, length=11)
        self.bs5 = BitSequence(299999999999998)
        self.bs6 = BitSequence(299999999999999)
        self.bs7 = BitSequence(value='10010101011111')
        self.bzs1 = BitZSequence(0x01, msb=True, length=8)
        self.bzs2 = BitZSequence('0Z1')
        self.bzs3 = BitZSequence('0Z1', length=5)
        self.bzs4 = BitZSequence('0010ZZ010Z1Z11')
        self.bzs5 = BitZSequence(value=[True, False, None, False, False, True])
        self.bzs6 = BitZSequence(value=[True, False, None, False, False, True],
                                 length=len(self.bzs4))

    def test_bitwise_ops(self):
        self.assertEqual(int(BitSequence(0x01, length=8) |
                             BitSequence(0x02, length=8)), 3)
        self.assertEqual(int(BitSequence(0x07, length=8) &
                             BitSequence(0x02, length=8)), 2)
        self.assertEqual(int(BitZSequence(0x01, length=8) |
                             BitSequence(0x02, length=8)), 3)
        self.assertEqual(int(BitSequence(0x07, length=8) &
                             BitZSequence(0x02, length=8)), 2)
        self.assertRaises(BitSequenceError, BitZSequence.__or__,
                          self.bzs4, self.bzs5)
        self.assertRaises(BitSequenceError, BitZSequence.__and__,
                          self.bzs4, self.bzs5)
        self.assertEqual(repr(self.bzs6), '00000000100Z01')
        self.assertEqual(repr(self.bzs6 | self.bzs4), '11Z1Z010ZZ0Z01')
        self.assertEqual(repr(self.bzs6 & self.bzs4), '00Z0Z000ZZ0Z00')
        self.assertEqual(repr(self.bzs4 & self.bs7), '11Z1Z010ZZ0000')
        self.assertEqual(repr(self.bs7 & self.bzs4), '11Z1Z010ZZ0000')
        self.assertEqual(repr(self.bzs4 | self.bs7), '11Z1Z010ZZ1101')
        self.assertEqual(repr(self.bs7 | self.bzs4), '11Z1Z010ZZ1101')
        self.assertEqual(repr(self.bs7.invert()), '00000101010110')
        self.assertEqual(repr(self.bzs4.invert()), '00Z0Z101ZZ1011')
        self.assertLess(self.bs5, self.bs6)
        self.assertLessEqual(self.bs5, self.bs6)
        self.assertGreater(self.bs6, self.bs5)
        self.assertGreaterEqual(self.bs6, self.bs5)

    def test_cmp(self):
        self.assertTrue(self.bs1 == self.bs1)
        self.assertTrue(self.bs1 != self.bs2)
        self.assertTrue(self.bs2 != BitSequence(0x02, msb=True, length=4))
        self.assertTrue(self.bzs2 == self.bzs2)
        self.assertTrue(self.bzs1 != self.bzs2)
        self.assertTrue(self.bs1 == self.bzs1)
        self.assertTrue(self.bzs1 == self.bs1)
        self.assertTrue(self.bzs3 == self.bzs2)
        self.assertNotEqual(self.bzs4, self.bzs5)
        bzs = BitZSequence(self.bs7)
        self.assertTrue(bzs == self.bs7)
        bzs |= BitZSequence('00Z0Z000ZZ0Z00')
        self.assertFalse(bzs == BitZSequence(self.bs7))
        self.assertTrue(bzs.matches(self.bs7))

    def test_representation(self):
        self.assertEqual(f'{self.bs1} / {self.bs1!r}',
                         '8: 10000000 / 10000000')
        self.assertEqual(f'{self.bs2} / {self.bs2!r}',
                         '8: 01000000 / 01000000')
        self.assertEqual(f'{self.bs3} / {self.bs3!r}',
                         '7: 0010000 / 0010000')
        self.assertEqual(f'{self.bs4} / {self.bs4!r}',
                         '11: 001 00000000 / 00100000000')
        self.assertEqual(f'{self.bs5} / {self.bs5!r}',
                         '49: 1 00010000 11011001 00110001 01101110 10111111 '
                         '11111110 / 100010000110110010011000101101110101111'
                         '1111111110')
        self.assertEqual(f'{self.bs6} / {self.bs6!r}',
                         '49: 1 00010000 11011001 00110001 01101110 10111111 '
                         '11111111 / 100010000110110010011000101101110101111'
                         '1111111111')
        self.assertEqual(repr(self.bzs4), '11Z1Z010ZZ0100')
        self.assertEqual(repr(self.bzs5), '100Z01')

    def test_init(self):
        self.assertEqual(int(BitSequence([0, 0, 1, 0])), 4)
        self.assertEqual(int(BitSequence((0, 1, 0, 0), msb=True)), 4)
        self.assertEqual(int(BitSequence(4, length=8)), 4)
        self.assertEqual(int(BitSequence(int(4), msb=True, length=8)), 32)
        self.assertEqual(int(BitSequence("0010")), 4)
        self.assertEqual(int(BitSequence("0100", msb=True)), 4)
        bs = BitSequence("0100", msb=True)
        self.assertEqual(bs, BitSequence(bs))
        bssub = BitSequence(bs[1:3])
        self.assertEqual(str(bssub), '2: 10')
        bs[0:3] = '11'
        self.assertEqual(str(bs), '4: 0011')
        bzs = BitZSequence(self.bzs4)
        self.assertEqual(bzs, self.bzs4)
        bs = BitSequence('11111010101001', msb=True)
        bs[8:12] = BitSequence(value='0000')
        self.assertEqual(repr(bs), '11000010101001')
        try:
            bs[8:12] = BitZSequence(value='ZZZZ')
        except BitSequenceError:
            pass
        except Exception as exc:
            self.fail(f'Unexpected exception {exc}')
        else:
            self.fail("Error was expected")
        bs = BitZSequence('1111101010100111Z1Z010ZZ0100', msb=True)
        bs[8:12] = BitZSequence(value='ZZZZ')
        self.assertEqual(repr(bs), '1111101010100111ZZZZ10ZZ0100')
        bs[8:12] = BitSequence(value='0000')
        self.assertEqual(repr(bs), '1111101010100111000010ZZ0100')
        n = 548521358
        bs = BitSequence(bin(n), msb=True)
        self.assertEqual(int(bs), n)
        bzs = BitZSequence(bin(n), msb=True)
        self.assertEqual(str(bzs), '30: 100000 10110001 11000101 10001110')
        bs = BitSequence(bytes_=[0x44, 0x66, 0xcc], msby=False)
        self.assertEqual(int(bs), 0x4466cc)
        bs = BitSequence(bytes_=(0x44, 0x66, 0xcc), msby=True)
        self.assertEqual(int(bs), 0xcc6644)
        try:
            bs = BitSequence(bytes_=[0x44, 0x666, 0xcc], msby=False)
        except BitSequenceError:
            pass
        except Exception as exc:
            self.fail(f'Unexpected exception {exc}')
        else:
            self.fail("Error was expected")

    def test_conversion(self):
        bs = BitSequence(0xCA, msb=True, length=8)
        self.assertEqual(f'{bs.tobyte(False):02x}', '53')
        self.assertEqual(f'{bs.tobyte(True):02x}', 'ca')
        self.assertEqual(bs, BitSequence(bs.tobyte(True), msb=True, length=8))
        self.assertRaises(BitSequenceError, BitZSequence.__int__, self.bzs5)
        self.assertRaises(BitSequenceError, BitZSequence.tobyte, self.bzs5)
        self.assertRaises(BitSequenceError, BitZSequence.tobytes, self.bzs5)
        bzs = BitZSequence(0xaa)
        self.assertEqual(int(bzs), 0xaa)

    def test_misc(self):
        ba = BitSequence(12, msb=True, length=16)
        bb = BitSequence(12, msb=True, length=14)
        bl = [ba, bb]
        bl.sort(key=int)
        self.assertEqual(str(bl), "[00110000000000, 0011000000000000]")
        self.assertEqual(ba.tobytes(), bytearray([48, 0]))
        self.assertEqual(ba.tobytes(True), bytearray([0, 12]))
        self.assertEqual(bb.tobytes(True), bytearray([0, 12]))

        b = BitSequence(length=254)
        b[0:4] = '1111'
        self.assertEqual(
            str(b), '254: 000000 00000000 00000000 00000000 '
            '00000000 00000000 00000000 00000000 00000000 00000000 00000000 '
            '00000000 00000000 00000000 00000000 00000000 00000000 00000000 '
            '00000000 00000000 00000000 00000000 00000000 00000000 00000000 '
            '00000000 00000000 00000000 00000000 00000000 00000000 00001111')
        self.assertEqual(
            b.tobytes(),
            bytearray([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15]))

        b = BitSequence(bytes_=[0xa0, '\x0f', 0x77], msb=False, msby=False)
        self.assertEqual(str([f'{x:02x}' for x in b.tobytes(False)]),
                         "['a0', '0f', '77']")
        b = BitSequence(bytes_=[0xa0, '\x0f', 0x77], msb=True, msby=True)
        self.assertEqual(str([f'{x:02x}' for x in b.tobytes(True)]),
                         "['a0', '0f', '77']")
        b = BitSequence(length=7)
        b[6] = '1'
        self.assertEqual(str(b), '7: 1000000')

    def test_rotations(self):
        b = BitSequence('10101110')
        b.lsr(2)
        self.assertEqual(str(b), '8: 01011101')
        b.lsr(10)
        self.assertEqual(str(b), '8: 01010111')
        b.rsr(3)
        self.assertEqual(str(b), '8: 10111010')

    def test_concat(self):
        self.assertEqual(repr(self.bzs4+self.bzs5), '100Z0111Z1Z010ZZ0100')
        self.assertEqual(repr(self.bzs4+self.bs7),
                         '1111101010100111Z1Z010ZZ0100')
        self.assertEqual(repr(self.bs7+self.bzs4),
                         '11Z1Z010ZZ010011111010101001')
    
    def test_int(self):
        self.assertEqual(int(BitSequence(65535)), 65535)
        self.assertEqual(int(BitSequence(65536)), 65536)
        self.assertEqual(int(BitSequence(65537)), 65537)
        self.assertEqual(int(BitSequence(65539)), 65539)
        self.assertEqual(int.from_bytes(BitSequence(65535).tobytes(), byteorder='big'), 65535)
        self.assertEqual(int.from_bytes(BitSequence(65536).tobytes(), byteorder='big'), 65536)
        self.assertEqual(int.from_bytes(BitSequence(65537).tobytes(), byteorder='big'), 65537)
        self.assertEqual(int.from_bytes(BitSequence(65539).tobytes(), byteorder='big'), 65539)
        self.assertEqual(int.from_bytes(BitSequence(65539).tobytes(msby=True), byteorder='little'), 65539)


def suite():
    suite_ = TestSuite()
    suite_.addTest(TestLoader().loadTestsFromModule(modules[__name__]))
    return suite_


if __name__ == '__main__':
    ut_main(defaultTest='suite')
