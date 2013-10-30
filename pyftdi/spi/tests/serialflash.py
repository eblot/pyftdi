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
from pyftdi.pyftdi.misc import hexdump, pretty_size
from pyftdi.spi.serialflash import SerialFlashManager
from random import randint
import sys
import time
import unittest


class SerialFlashTestCase(unittest.TestCase):

    def setUp(self):
        # FTDI device should be tweak to your actual setup
        self.flash = SerialFlashManager.get_flash_device(0x403, 0x6010, 1)

    def tearDown(self):
        del self.flash

    def test_flashdevice_1_name(self):
        """Retrieve device name
        """
        print "Flash device: %s @ SPI freq %0.1f MHz" % \
            (self.flash, self.flash.spi_frequency/1E6)

    def test_flashdevice_2_read_bandwidth(self):
        """Read the whole device to get READ bandwith
        """
        delta = time.time()
        data = self.flash.read(0, len(self.flash))
        delta = time.time()-delta
        length = len(data)
        self._report_bw('Read', length, delta)

    def test_flashdevice_3_small_rw(self):
        """Short R/W test
        """
        self.flash.unlock()
        self.flash.erase(0x007000, 4096)
        data = self.flash.read(0x007020, 128)
        ref = Array('B', [0xff] * 128)
        self.assertEqual(data, ref)
        string = 'This is a serial SPI flash test.'
        ref2 = Array('B', string)
        self.flash.write(0x007020, string)
        data = self.flash.read(0x007020, 128)
        ref2.extend(ref)
        ref2 = ref2[:128]
        self.assertEqual(data, ref2)

    def test_flashdevice_4_long_rw(self):
        """Long R/W test
        """
        # Max size to perform the test on
        size = 1<<20
        # Whether to test with random value, or contiguous values to ease debug
        randomize = True
        # Fill in the whole flash with a monotonic increasing value, that is
        # the current flash 32-bit address, then verify the sequence has been
        # properly read back
        from hashlib import sha1
        # limit the test to 1MiB to keep the test duration short, but performs
        # test at the end of the flash to verify that high addresses may be
        # reached
        length = min(len(self.flash), size)
        start = len(self.flash)-length
        print "Erase %s from flash @ 0x%06x(may take a while...)" % \
            (pretty_size(length), start)
        delta = time.time()
        self.flash.unlock()
        self.flash.erase(start, length, True)
        delta = time.time()-delta
        self._report_bw('Erased', length, delta)
        if str(self.flash).startswith('SST'):
            # SST25 flash devices are tremendously slow at writing (one or two
            # bytes per SPI request MAX...). So keep the test sequence short
            # enough
            length = 16<<10
        print "Build test sequence"
        if not randomize:
            buf = Array('I')
            back = Array('I')
            for address in range(0, length, 4):
                buf.append(address)
            # Expect to run on x86 or ARM (little endian), so swap the values
            # to ease debugging
            # A cleaner test would verify the host endianess, or use struct
            # module
            buf.byteswap()
            # Cannot use buf directly, as it's an I-array,
            # and SPI expects a B-array
        else:
            from random import seed
            seed(0)
            buf = Array('B')
            back = Array('B')
            buf.extend((randint(0, 255) for x in range(0, length)))
        bufstr = buf.tostring()
        print "Writing %s to flash (may take a while...)" % \
            pretty_size(len(bufstr))
        delta = time.time()
        self.flash.write(start, bufstr)
        delta = time.time()-delta
        length = len(bufstr)
        self._report_bw('Wrote', length, delta)
        wmd = sha1()
        wmd.update(buf.tostring())
        refdigest = wmd.hexdigest()
        print "Reading %s from flash" % pretty_size(length)
        delta = time.time()
        data = self.flash.read(start, length)
        delta = time.time()-delta
        self._report_bw('Read', length, delta)
        #print "Dump flash"
        #print hexdump(data.tostring())
        print "Verify flash"
        rmd = sha1()
        rmd.update(data.tostring())
        newdigest = rmd.hexdigest()
        print "Reference:", refdigest
        print "Retrieved:", newdigest
        if refdigest != newdigest:
            errcount = 0
            back.fromstring(data)
            for pos in xrange(len(buf)):
                if buf[pos] != data[pos]:
                    print 'Invalid byte @ offset 0x%06x: 0x%02x / 0x%02x' % \
                        (pos, buf[pos], back[pos])
                    errcount += 1
                    # Stop report after 16 errors
                    if errcount >= 32:
                        break
            raise AssertionError('Data comparison mismatch')

    @classmethod
    def _report_bw(cls, action, length, time):
        if time < 1.0:
            print "%s %s in %d ms @ %s/s" % (action, pretty_size(length),
                1000*time, pretty_size(length/time))
        else:
            print "%s %s in %d seconds @ %s/s" % (action, pretty_size(length),
                time, pretty_size(length/time))
        
def suite():
    return unittest.makeSuite(SerialFlashTestCase, 'test')

if __name__ == '__main__':
    unittest.main(defaultTest='suite')
