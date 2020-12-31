#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2011-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from os import environ
from unittest import TestCase, main as ut_main, makeSuite
from pyftdi.jtag import JtagEngine, JtagTool
from pyftdi.bits import BitSequence

#pylint: disable-msg=missing-docstring


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
        print("IDCODE (reset): 0x%x" % int(idcode))

    def test_idcode_sequence(self):
        """Read the IDCODE using the dedicated instruction"""
        instruction = JTAG_INSTR['IDCODE']
        self.jtag.write_ir(instruction)
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        print("IDCODE (idcode): 0x%08x" % int(idcode))

    def test_idcode_shift_register(self):
        """Read the IDCODE using the dedicated instruction with
           shift_and_update_register"""
        instruction = JTAG_INSTR['IDCODE']
        self.jtag.change_state('shift_ir')
        retval = self.jtag.shift_and_update_register(instruction)
        print("retval: 0x%x" % int(retval))
        self.jtag.go_idle()
        self.jtag.change_state('shift_dr')
        idcode = self.jtag.shift_and_update_register(BitSequence('0'*32))
        self.jtag.go_idle()
        print("IDCODE (idcode): 0x%08x" % int(idcode))

    def test_bypass_shift_register(self):
        """Test the BYPASS instruction using shift_and_update_register"""
        instruction = JTAG_INSTR['BYPASS']
        self.jtag.change_state('shift_ir')
        retval = self.jtag.shift_and_update_register(instruction)
        print("retval: 0x%x" % int(retval))
        self.jtag.go_idle()
        self.jtag.change_state('shift_dr')
        _in = BitSequence('011011110000'*2, length=24)
        out = self.jtag.shift_and_update_register(_in)
        self.jtag.go_idle()
        print("BYPASS sent: %s, received: %s  (should be left shifted by one)"
              % (_in, out))

    def _test_detect_ir_length(self):
        """Detect the instruction register length"""
        self.jtag.go_idle()
        self.jtag.capture_ir()
        self.tool.detect_register_size()


def suite():
    return makeSuite(JtagTestCase, 'test')


if __name__ == '__main__':
    ut_main(defaultTest='suite')
