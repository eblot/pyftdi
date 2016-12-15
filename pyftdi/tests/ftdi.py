#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2010-2016, Emmanuel Blot <emmanuel.blot@free.fr>
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
import unittest

from doctest import testmod
from pyftdi.ftdi import Ftdi
from time import sleep


class FtdiTestCase(unittest.TestCase):
    """FTDI driver test case"""

    def test_multiple_interface(self):
        # the following calls used to create issues (several interfaces from
        # the same device). The test expects an FTDI 2232H here
        ftdi1 = Ftdi()
        ftdi1.open(vendor=0x403, product=0x6010, interface=1)
        ftdi2 = Ftdi()
        ftdi2.open(vendor=0x403, product=0x6010, interface=2)
        for x in range(5):
            print("If#1: ", hex(ftdi1.poll_modem_status()))
            print("If#2: ", ftdi2.modem_status())
            sleep(0.500)
        ftdi1.close()
        ftdi2.close()


def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(FtdiTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    testmod(sys.modules[__name__])
    unittest.main(defaultTest='suite')
