#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""JTAG unit test."""

# Copyright (c) 2011-2024, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from os import environ
from sys import modules
from unittest import TestCase, TestLoader, TestSuite, main as ut_main
from pyftdi.jtag import JtagEngine, JtagTool
from pyftdi.bits import BitSequence

# pylint: disable=missing-docstring


# Should match the tested device
JTAG_INSTR = {'SAMPLE': BitSequence('0001', msb=True, length=4),
              'PRELOAD': BitSequence('0001', msb=True, length=4),
              'IDCODE': BitSequence('0100', msb=True, length=4),
              'BYPASS': BitSequence('1111', msb=True, length=4)}


class JtagTestCase(TestCase):

    def setUp(self):
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self.jtag = JtagEngine(trst=True, frequency=3E6)
        self.jtag.configure(url)
        self.jtag.reset()
        self.tool = JtagTool(self.jtag)

    def tearDown(self):
        del self.jtag

    def test_idcode_reset(self):
        """Read the IDCODE right after a JTAG reset"""
        self.jtag.reset()
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        print(f'IDCODE (reset): 0x{int(idcode):x}')

    def test_idcode_sequence(self):
        """Read the IDCODE using the dedicated instruction"""
        instruction = JTAG_INSTR['IDCODE']
        self.jtag.write_ir(instruction)
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        print(f'IDCODE (idcode): 0x{int(idcode):08x}')

    def test_idcode_shift_register(self):
        """Read the IDCODE using the dedicated instruction with
           shift_and_update_register"""
        instruction = JTAG_INSTR['IDCODE']
        self.jtag.change_state('shift_ir')
        retval = self.jtag.shift_and_update_register(instruction)
        print(f'retval: 0x{int(retval):x}')
        self.jtag.go_idle()
        self.jtag.change_state('shift_dr')
        idcode = self.jtag.shift_and_update_register(BitSequence('0'*32))
        self.jtag.go_idle()
        print(f'IDCODE (idcode): 0x{int(idcode):08x}')

    def test_bypass_shift_register(self):
        """Test the BYPASS instruction using shift_and_update_register"""
        instruction = JTAG_INSTR['BYPASS']
        self.jtag.change_state('shift_ir')
        retval = self.jtag.shift_and_update_register(instruction)
        print(f'retval: 0x{int(retval):x}')
        self.jtag.go_idle()
        self.jtag.change_state('shift_dr')
        in_ = BitSequence('011011110000'*2, length=24)
        out = self.jtag.shift_and_update_register(in_)
        self.jtag.go_idle()
        print(f'BYPASS sent: {in_}, received: {out} '
              f' (should be left shifted by one)')

    def _test_detect_ir_length(self):
        """Detect the instruction register length"""
        self.jtag.go_idle()
        self.jtag.capture_ir()
        self.tool.detect_register_size()


def suite():
    suite_ = TestSuite()
    suite_.addTest(TestLoader().loadTestsFromModule(modules[__name__]))
    return suite_


if __name__ == '__main__':
    ut_main(defaultTest='suite')
