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
from binascii import hexlify
from doctest import testmod
from logging import StreamHandler, DEBUG
from pyftdi import FtdiLogger
from pyftdi.spi import SpiController
from sys import modules, stdout


class SpiTest(object):
    """Basic test for a MX25L1606E data flash device selected as CS0,
       and an ADXL345 device selected as CS1
    """

    def __init__(self):
        self._spi = SpiController()

    def open(self):
        """Open an I2c connection to a slave"""
        self._spi.configure('ftdi://ftdi:2232h/1')

    def read_jedec_id(self):
        port = self._spi.get_port(0, freq=3E6, mode=3)
        jedec_id = port.exchange([0x9f], 3).tobytes()
        hex_jedec_id = hexlify(jedec_id).decode()
        print('JEDEC ID:', hex_jedec_id)
        return hex_jedec_id

    def read_device_id(self):
        port = self._spi.get_port(1, freq=6E6, mode=3)
        device_id = port.exchange([0x00], 1).tobytes()
        hex_device_id = hexlify(device_id).decode()
        print('DEVICE ID:', hex_device_id)
        return hex_device_id

    def close(self):
        """Close the I2C connection"""
        self._spi.terminate()


class SpiTestCase(unittest.TestCase):
    """FTDI SPI driver test case"""

    def test_spi(self):
        """Simple test to demonstrate SPI.

           Please ensure that the HW you connect to the FTDI port A does match
           the encoded configuration. GPIOs can be driven high or low, so check
           your HW setup before running this test as it might damage your HW.

           Do NOT run this test if you use FTDI port A as an UART or I2C
           bridge -or any unsupported setup!! You've been warned.
        """
        spi = SpiTest()
        spi.open()
        jedec_id = spi.read_jedec_id()
        self.assertEqual(jedec_id, 'c22016')
        device_id = spi.read_device_id()
        self.assertEqual(device_id, 'e5')
        spi.close()


def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(SpiTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(StreamHandler(stdout))
    FtdiLogger.set_level(DEBUG)
    unittest.main(defaultTest='suite')
