#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2017-2018, Emmanuel Blot <emmanuel.blot@free.fr>
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

import logging
import unittest
from binascii import hexlify
from doctest import testmod
from os import environ
from sys import modules, stdout
from pyftdi import FtdiLogger
from pyftdi.i2c import I2cController


class I2cTca9555Test(object):
    """Simple test for a TCA9555 device on I2C bus @ address 0x21
    """

    def __init__(self):
        self._i2c = I2cController()

    def open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._i2c.configure(url)

    def read_it(self):
        port = self._i2c.get_port(0x21)
        port.exchange([0x04], 1)

    def write_it(self):
        port = self._i2c.get_port(0x21)
        port.write_to(0x06, b'\x00')
        port.write_to(0x02, b'\x55')
        port.read_from(0x00, 1)

    def close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


class I2cAccelTest(object):
    """Basic test for an ADXL345 device on I2C bus @ address 0x53
    """

    def __init__(self):
        self._i2c = I2cController()

    def open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._i2c.configure(url)

    def read_device_id(self):
        port = self._i2c.get_port(0x53)
        device_id = port.exchange([0x00], 1).tobytes()
        hex_device_id = hexlify(device_id).decode()
        print('DEVICE ID:', hex_device_id)
        return hex_device_id

    def close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


class I2cReadTest(object):
    """Simple test to read a sequence of bytes I2C bus @ address 0x21
    """

    def __init__(self):
        self._i2c = I2cController()

    def open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._i2c.configure(url)

    def read(self):
        port = self._i2c.get_port(0x36)
        print(hexlify(port.read(32)).decode())

    def close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


class I2cTestCase(unittest.TestCase):
    """FTDI I2C driver test case

       Simple test to demonstrate I2C.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. GPIOs can be driven high or low, so check
       your HW setup before running this test as it might damage your HW.

       Do NOT run this test if you use FTDI port A as an UART or SPI
       bridge -or any unsupported setup!! You've been warned.
    """

    def test_i2c1(self):
        i2c = I2cTca9555Test()
        i2c.open()
        i2c.read_it()
        i2c.write_it()
        i2c.close()

    def test_i2c2(self):
        i2c = I2cAccelTest()
        i2c.open()
        i2c.read_device_id()
        i2c.close()

    def test_i2c3(self):
        i2c = I2cReadTest()
        i2c.open()
        i2c.read()
        i2c.close()


def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(I2cTestCase, 'test'))
    return suite_


def main():
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError('Invalid log level: %s', level)
    FtdiLogger.set_level(loglevel)
    unittest.main(defaultTest='suite')


if __name__ == '__main__':
    main()
