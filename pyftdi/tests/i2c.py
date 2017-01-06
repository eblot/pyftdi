#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2017, Emmanuel Blot <emmanuel.blot@free.fr>
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

import unittest
import sys
from pyftdi.i2c import I2cController, I2cIOError
from time import sleep


class I2cTest(object):
    """
    """

    def __init__(self):
        self._i2c = I2cController()

    def open(self):
        """Open an I2c connection to a slave"""
        self._i2c.configure('ftdi://ftdi:232h/1')

    def general_call(self):
        port = self._i2c.get_port(0)
        port.write([0x16])

    def close(self):
        """Close the I2C connection"""
        self._i2c.terminate()



class I2cTestCase(unittest.TestCase):
    """FTDI I2C driver test case"""

    def test_i2c(self):
        """Simple test to demonstrate I2C.

           Please ensure that the HW you connect to the FTDI port A does match
           the encoded configuration. At least, b7..b5 can be driven high or
           low, so check your HW setup before running this test as it might
           damage your HW.

           Do NOT run this test if you use FTDI port A as an UART or SPI
           bridge -or any unsupported setup!! You've been warned.
        """
        i2c = I2cTest()
        i2c.open()
        i2c.general_call()
        i2c.close()


def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(I2cTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    import doctest
    doctest.testmod(sys.modules[__name__])
    unittest.main(defaultTest='suite')
