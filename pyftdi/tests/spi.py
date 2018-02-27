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
from sys import modules, stderr, stdout
from time import sleep
from pyftdi import FtdiLogger
from pyftdi.spi import SpiController, SpiIOError


class SpiDataFlashTest(object):
    """Basic test for a MX25L1606E data flash device selected as CS0,
       SPI mode 0
    """

    def __init__(self):
        self._spi = SpiController(cs_count=3)

    def open(self):
        """Open an SPI connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._spi.configure(url)

    def read_jedec_id(self):
        port = self._spi.get_port(0, freq=3E6, mode=0)
        jedec_id = port.exchange([0x9f], 3).tobytes()
        hex_jedec_id = hexlify(jedec_id).decode()
        print('JEDEC ID:', hex_jedec_id)
        return hex_jedec_id

    def close(self):
        """Close the SPI connection"""
        self._spi.terminate()


class SpiAccelTest(object):
    """Basic test for an ADXL345 device selected as CS1,
       SPI mode 3
    """

    def __init__(self):
        self._spi = SpiController(cs_count=3)

    def open(self):
        """Open an SPI connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._spi.configure(url)

    def read_device_id(self):
        port = self._spi.get_port(1, freq=6E6, mode=3)
        device_id = port.exchange([0x00], 1).tobytes()
        hex_device_id = hexlify(device_id).decode()
        print('DEVICE ID:', hex_device_id)
        return hex_device_id

    def close(self):
        """Close the SPI connection"""
        self._spi.terminate()


class SpiRfda2125Test(object):
    """Basic test for a RFDA2125 Digital Controlled Variable Gain Amplifier
       selected as CS2,
       SPI mode 0
    """

    def __init__(self):
        self._spi = SpiController(cs_count=3)

    def open(self):
        """Open an SPI connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._spi.configure(url)
        self._port = self._spi.get_port(2, freq=1E6, mode=0)

    def change_attenuation(self, value):
        if not (0.0 <= value <= 31.5):
            print('Out-of-bound attenuation', file=stderr)
        intval = 63-int(value*2)
        self._port.write(bytes([intval]), 1)

    def close(self):
        """Close the SPI connection"""
        self._spi.terminate()


class SpiTestCase(unittest.TestCase):
    """FTDI SPI driver test case

       Simple test to demonstrate SPI feature.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. GPIOs can be driven high or low, so check
       your HW setup before running this test as it might damage your HW.

       Do NOT run this test if you use FTDI port A as an UART or I2C
       bridge -or any unsupported setup!! You've been warned.
    """

    def test_spi1(self):
        spi = SpiDataFlashTest()
        spi.open()
        jedec_id = spi.read_jedec_id()
        self.assertEqual(jedec_id, 'c22016')
        spi.close()

    def test_spi2(self):
        spi = SpiAccelTest()
        spi.open()
        device_id = spi.read_device_id()
        self.assertEqual(device_id, 'e5')
        spi.close()

    def test_spi3(self):
        spi = SpiRfda2125Test()
        spi.open()
        slope = 1
        attenuation = 0.0
        for cycle in range(10):
            for step in range(63):
                attenuation += float(slope)
                print(attenuation/2.0)
                spi.change_attenuation(attenuation/2.0)
                sleep(0.05)  # 50 ms
            slope = -slope
        spi.close()


class SpiGpioTestCase(unittest.TestCase):
    """Basic test for GPIO access w/ SPI mode

       It expects the following I/O setup:

       AD4 connected t0 AC0
       AD5 connected t0 AC1
       AD6 connected t0 AC2
       AD7 connected t0 AC3
    """

    # AD0: SCLK, AD1: MOSI, AD2: MISO, AD3: /CS
    AD_OFFSET = 4
    AC_OFFSET = 8
    PIN_COUNT = 4

    def setUp(self):
        self._spi = SpiController(cs_count=1)
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._spi.configure(url)
        self._port = self._spi.get_port(0, freq=1E6, mode=0)
        self._io = self._spi.get_gpio()

    def tearDown(self):
        """Close the SPI connection"""
        self._spi.terminate()

    def test_ac_to_ad(self):
        ad_pins = ((1 << self.PIN_COUNT) - 1) << self.AD_OFFSET  # input
        ac_pins = ((1 << self.PIN_COUNT) - 1) << self.AC_OFFSET  # output
        io_pins = ad_pins | ac_pins

        def ac_to_ad(ac_output):
            ac_output &= ac_pins
            ac_output >>= self.AC_OFFSET - self.AD_OFFSET
            return ac_output & ad_pins

        self._io.set_direction(io_pins, ac_pins)
        for ac in range(1 << self.PIN_COUNT):
            ac_out = ac << self.AC_OFFSET
            ad_in = ac_to_ad(ac_out)
            self._io.write(ac_out)
            # random SPI exchange to ensure SPI does not change GPIO
            self._port.exchange([0x00, 0xff], 2)
            rd = self._io.read()
            self.assertEqual(rd, ad_in)
        self.assertRaises(SpiIOError, self._io.write, ad_pins)

    def test_ad_to_ac(self):
        ad_pins = ((1 << self.PIN_COUNT) - 1) << self.AD_OFFSET  # output
        ac_pins = ((1 << self.PIN_COUNT) - 1) << self.AC_OFFSET  # input
        io_pins = ad_pins | ac_pins

        def ad_to_ac(ad_output):
            ad_output &= ad_pins
            ad_output <<= self.AC_OFFSET - self.AD_OFFSET
            return ad_output & ac_pins

        self._io.set_direction(io_pins, ad_pins)
        for ad in range(1 << self.PIN_COUNT):
            ad_out = ad << self.AD_OFFSET
            ac_in = ad_to_ac(ad_out)
            self._io.write(ad_out)
            # random SPI exchange to ensure SPI does not change GPIO
            self._port.exchange([0x00, 0xff], 2)
            rd = self._io.read()
            self.assertEqual(rd, ac_in)
        self.assertRaises(SpiIOError, self._io.write, ac_pins)


def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(SpiTestCase, 'test'))
    suite_.addTest(unittest.makeSuite(SpiGpioTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError('Invalid log level: %s', level)
    FtdiLogger.set_level(loglevel)
    unittest.main(defaultTest='suite')
