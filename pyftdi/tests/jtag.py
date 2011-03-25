#!/usr/bin/env python
# Copyright (c) 2011, Emmanuel Blot <emmanuel.blot@free.fr>
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

import sys
import time
import unittest
from pyftdi.jtag import JtagEngine, JtagTool
from pyftdi.bits import BitSequence

# Should match the tested device
JTAG_INSTR = {'SAMPLE'  : BitSequence('0001', msb=True, length=4),
              'PRELOAD' : BitSequence('0001', msb=True, length=4),
              'IDCODE'  : BitSequence('0100', msb=True, length=4),
              'BYPASS'  : BitSequence('1111', msb=True, length=4) }

class JtagTestCase(unittest.TestCase):

    def setUp(self):
        self.jtag = JtagEngine()
        self.jtag.configure(interface=1)
        self.jtag.reset()
        self.tool = JtagTool(self.jtag)

    def tearDown(self):
        del self.jtag

    def _test_idcode_reset(self):
        """Read the IDCODE right after a JTAG reset"""
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        print "IDCODE: 0x%x" % int(idcode)

    def _test_idcode_sequence(self):
        """Read the IDCODE using the dedicated instruction"""
        instruction = JTAG_INSTR['IDCODE']
        self.jtag.write_ir(instruction)
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        print "IDCODE: 0x%x" % int(idcode)

    def test_detect_ir_length(self):
        """Detect the instruction register length"""
        self.jtag.reset()
        self.jtag.go_idle()
        self.jtag.capture_ir()
        self.tool.detect_register_size()

    def test_shift_register(self):
        self.jtag.reset()
        self.jtag.go_idle()
        self.jtag.capture_ir()
        for x in range(10):
            patin = BitSequence(0b1, length=3)
            patout = self.jtag.shift_register(patin)
            print patin
            print patout


def suite():
    return unittest.makeSuite(JtagTestCase, '_test')

if __name__ == '__main__':
    unittest.main(defaultTest='suite')
