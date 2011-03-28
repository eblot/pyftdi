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

from array import array as Array
from pyftdi.misc import hexdump
from spi.serialflash import SerialFlashManager
import sys
import time
import unittest


class SerialFlashTestCase(unittest.TestCase):

    def setUp(self):
        self.manager = SerialFlashManager(0x403, 0x6010, 1)
        self.flash = self.manager.get_flash_device()

    def tearDown(self):
        del self.flash
        del self.manager

    def test_flashdevice_name(self):
        print "Flash device: %s" % self.flash

    def test_flashdevice_read_bandwidth(self):
        print "Start reading the whole device..."
        delta = time.time()
        data = self.flash.read(0, len(self.flash))
        delta = time.time()-delta
        length = len(data)
        print "%d bytes in %d seconds @ %.1f KB/s" % \
            (length, delta, length/(1024.0*delta))

    def test_flashdevice_small_rw(self):
        self.flash.erase(0x007000, 4096)
        data = self.flash.read(0x007020, 128).tostring()
        print hexdump(data)
        self.flash.write(0x007020, 'This is a serial SPI flash test')
        data = self.flash.read(0x007020, 128).tostring()
        print hexdump(data)

    def test_flashdevice_long_rw(self):
        # Fill in the whole flash with a monotonic increasing value, that is
        # the current flash 32-bit address, then verify the sequence has been
        # properly read back
        from hashlib import sha1
        buf = Array('I')
        print "Build sequence"
        for address in range(0, len(self.flash), 4):
            buf.append(address)
        # Expect to run on x86 or ARM (little endian), so swap the values
        # to ease debugging
        # A cleaner test would verify the host endianess, or use struct module
        print "Swap sequence"
        buf.byteswap()
        print "Erase flash (may take a while...)"
        self.flash.erase(0, len(self.flash))
        # Cannot use buf, as it's an I-array, and SPI expects a B-array
        bufstr = buf.tostring()
        print "Write flash", len(bufstr)
        self.flash.write(0, bufstr)
        wmd = sha1()
        wmd.update(buf.tostring())
        refdigest = wmd.hexdigest()
        print "Read flash"
        data = self.flash.read(0, len(self.flash))
        #print "Dump flash"
        #print hexdump(data.tostring())
        print "Verify flash"
        rmd = sha1()
        rmd.update(data.tostring())
        newdigest = rmd.hexdigest()
        print "Reference:", refdigest
        print "Retrieved:", newdigest
        if refdigest != newdigest:
            raise AssertionError('Data comparison mismatch')

def suite():
    return unittest.makeSuite(SerialFlashTestCase, 'test')

if __name__ == '__main__':
    unittest.main(defaultTest='suite')
