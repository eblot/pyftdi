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
from pyftdi.jtag import JtagEngine
from pyftdi.bits import BitSequence

# ARM 926
JTAG_INSTR = {'EXTEST'  : BitSequence('0000', msb=True, length=4),
              'SAMPLE'  : BitSequence('0011', msb=True, length=4),
              'PRELOAD' : BitSequence('0011', msb=True, length=4),
              'SCAN_N'  : BitSequence('0010', msb=True, length=4),
              'INTEST'  : BitSequence('1100', msb=True, length=4),
              'IDCODE'  : BitSequence('1110', msb=True, length=4),
              'BYPASS'  : BitSequence('1111', msb=True, length=4),
              'RESTART' : BitSequence('0100', msb=True, length=4)}


class ArmJtag(object):
    """JTAG helper for ARM core"""

    def __init__(self, vendor, product, interface):
        self.jtag = JtagEngine()
        self.jtag.configure(vendor, product, interface)
        self.jtag.reset()

    def get_idcode_from_reset(self):
        """Read the IDCODE right after a JTAG reset"""
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        print "IDCODE: 0x%x" % int(idcode)
        return int(idcode)

    def get_idcode_from_instruction(self):
        """Read the IDCODE using the dedicated instruction"""
        instruction = JTAG_INSTR['IDCODE']
        self.jtag.write_ir(instruction)
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        revision = idcode[28:32]
        partnumber = idcode[12:28]
        manufacturer = idcode[1:12]
        ieee = idcode[0:1]
        print "IDCODE: 0x%x %s %s %s %s" % (int(idcode),
            revision, partnumber, manufacturer, ieee)
        return idcode


if __name__ == '__main__':
    jtag = ArmJtag(0x403, 0x6011, 1)
    jtag.get_idcode_from_reset()
    jtag.get_idcode_from_instruction()
