#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2017-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=no-self-use

import logging
import unittest
from binascii import hexlify
from doctest import testmod
from os import environ
from sys import modules, stderr, stdout
from time import sleep
from pyftdi import FtdiLogger
from pyftdi.misc import to_bool
from pyftdi.spi import SpiController, SpiIOError


class SpiDataFlashTest:
    """Basic test for a MX25L1606E data flash device selected as CS0,
       SPI mode 0
    """

    def __init__(self):
        self._spi = SpiController(cs_count=3)

    def open(self):
        """Open an SPI connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        debug = to_bool(environ.get('FTDI_DEBUG', 'off'))
        self._spi.configure(url, debug=debug)

    def read_jedec_id(self):
        port = self._spi.get_port(0, freq=3E6, mode=0)
        jedec_id = port.exchange([0x9f], 3)
        hex_jedec_id = hexlify(jedec_id).decode()
        print('JEDEC ID:', hex_jedec_id)
        return hex_jedec_id

    def close(self):
        """Close the SPI connection"""
        self._spi.terminate()


class SpiAccelTest:
    """Basic test for an ADXL345 device selected as CS1,
       SPI mode 3
    """

    def __init__(self):
        self._spi = SpiController(cs_count=3)

    def open(self):
        """Open an SPI connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        debug = to_bool(environ.get('FTDI_DEBUG', 'off'))
        self._spi.configure(url, debug=debug)

    def read_device_id(self):
        port = self._spi.get_port(1, freq=6E6, mode=3)
        device_id = port.exchange([0x00], 1)
        hex_device_id = hexlify(device_id).decode()
        print('DEVICE ID:', hex_device_id)
        return hex_device_id

    def close(self):
        """Close the SPI connection"""
        self._spi.terminate()


class SpiRfda2125Test:
    """Basic test for a RFDA2125 Digital Controlled Variable Gain Amplifier
       selected as CS2,
       SPI mode 0
    """

    def __init__(self):
        self._spi = SpiController(cs_count=3)
        self._port = None

    def open(self):
        """Open an SPI connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        debug = to_bool(environ.get('FTDI_DEBUG', 'off'))
        self._spi.configure(url, debug=debug)
        self._port = self._spi.get_port(2, freq=1E6, mode=0)

    def change_attenuation(self, value):
        if not 0.0 <= value <= 31.5:
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
        self.assertIn(jedec_id, ('c22016', 'bf254a'))
        spi.close()

    def _test_spi2(self):
        spi = SpiAccelTest()
        spi.open()
        device_id = spi.read_device_id()
        self.assertEqual(device_id, 'e5')
        spi.close()

    def _test_spi3(self):
        spi = SpiRfda2125Test()
        spi.open()
        slope = 1
        attenuation = 0.0
        for _ in range(10):
            for _ in range(63):
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

    @classmethod
    def setUpClass(cls):
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        cls.debug = to_bool(environ.get('FTDI_DEBUG', 'off'))

    def setUp(self):
        self._spi = SpiController(cs_count=1)
        self._spi.configure(self.url, debug=self.debug)
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
        for ac_pin in range(1 << self.PIN_COUNT):
            ac_out = ac_pin << self.AC_OFFSET
            ad_in = ac_to_ad(ac_out)
            self._io.write(ac_out)
            # random SPI exchange to ensure SPI does not change GPIO
            self._port.exchange([0x00, 0xff], 2)
            buf = self._io.read()
            self.assertEqual(buf, ad_in)
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
        for ad_pin in range(1 << self.PIN_COUNT):
            ad_out = ad_pin << self.AD_OFFSET
            ac_in = ad_to_ac(ad_out)
            self._io.write(ad_out)
            # random SPI exchange to ensure SPI does not change GPIO
            self._port.exchange([0x00, 0xff], 2)
            buf = self._io.read()
            self.assertEqual(buf, ac_in)
        self.assertRaises(SpiIOError, self._io.write, ac_pins)


class SpiUnalignedTestCase(unittest.TestCase):
    """Basic test for SPI with non 8-bit multiple transfer

       It expects the following I/O setup:

       MOSI (AD1) connected to MISO (AD2)
    """

    @classmethod
    def setUpClass(cls):
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        cls.debug = to_bool(environ.get('FTDI_DEBUG', 'off'))

    def setUp(self):
        self._spi = SpiController(cs_count=1)
        self._spi.configure(self.url, debug=self.debug)
        self._port = self._spi.get_port(0, freq=1E6, mode=0)

    def tearDown(self):
        """Close the SPI connection"""
        self._spi.terminate()

    def test_invalid_write(self):
        buf = b'\xff\xff'
        self.assertRaises(ValueError, self._port.write, buf, droptail=8)

    def test_bit_write(self):
        buf = b'\x0f'
        for loop in range(7):
            self._port.write(buf, droptail=loop+1)

    def test_bytebit_write(self):
        buf = b'\xff\xff\x0f'
        for loop in range(7):
            self._port.write(buf, droptail=loop+1)

    def test_invalid_read(self):
        self.assertRaises(ValueError, self._port.read, 1, droptail=8)
        self.assertRaises(ValueError, self._port.read, 2, droptail=8)

    def test_bit_read(self):
        # make MOSI stay to low level, so MISO samples 0
        self._port.write([0x00])
        for loop in range(7):
            data = self._port.read(1, droptail=loop+1)
            self.assertEqual(len(data), 1)
        # make MOSI stay to high level, so MISO samples 1
        self._port.write([0x01])
        for loop in range(7):
            data = self._port.read(1, droptail=loop+1)
            self.assertEqual(len(data), 1)

    def test_bytebit_read(self):
        self._port.write([0x00])
        for loop in range(7):
            data = self._port.read(3, droptail=loop+1)
            self.assertEqual(len(data), 3)
            self.assertEqual(data[-1], 0)
        self._port.write([0x01])
        for loop in range(7):
            data = self._port.read(3, droptail=loop+1)
            self.assertEqual(len(data), 3)

    def test_invalid_duplex(self):
        buf = b'\xff\xff'
        self.assertRaises(ValueError, self._port.exchange, buf,
                          duplex=False, droptail=8)
        self.assertRaises(ValueError, self._port.exchange, buf,
                          duplex=False, droptail=8)
        self.assertRaises(ValueError, self._port.exchange, buf,
                          duplex=True, droptail=8)
        self.assertRaises(ValueError, self._port.exchange, buf,
                          duplex=True, droptail=8)

    def test_bit_duplex(self):
        buf = b'\xcf'
        for loop in range(7):
            data = self._port.exchange(buf, duplex=True, droptail=loop+1)
            self.assertEqual(len(data), 1)
            exp = buf[0] & ~((1<<(loop+1))-1)
            # print(f'{data[0]:08b} {exp:08b}')
            self.assertEqual(data[0], exp)

    def test_bytebit_duplex(self):
        buf = b'\xff\xcf'
        for loop in range(7):
            data = self._port.exchange(buf, duplex=True, droptail=loop+1)
            self.assertEqual(len(data), 2)
            exp = buf[-1] & ~((1<<(loop+1))-1)
            # print(f'{data[-1]:08b} {exp:08b}')
            self.assertEqual(data[0], 0xFF)
            self.assertEqual(data[-1], exp)


class SpiCsForceTestCase(unittest.TestCase):
    """Basic test for exercing direct /CS control.

       It requires a scope or a digital analyzer to validate the signal
       waveforms.
    """

    @classmethod
    def setUpClass(cls):
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        cls.debug = to_bool(environ.get('FTDI_DEBUG', 'off'))

    def setUp(self):
        self._spi = SpiController(cs_count=1)
        self._spi.configure(self.url, debug=self.debug)
        self._port = self._spi.get_port(0, freq=1E6, mode=0)

    def tearDown(self):
        """Close the SPI connection"""
        self._spi.terminate()

    def test_cs_default_pulse(self):
        for _ in range(5):
            self._port.force_select()

    def test_cs_long_pulse(self):
        for _ in range(5):
            self._port.force_select(cs_hold=200)

    def test_cs_manual_pulse(self):
        for _ in range(5):
            self._port.force_select(level=False)
            self._port.force_select(level=True)
            # beware that random USB bus access does not allow to create
            # precise delays. This is only the shorter bound, longer one is
            # not defined
            sleep(100e-6)

    def test_cs_pulse_write(self):
        self._port.force_select()
        self._port.write([0x00, 0x01, 0x02])

    def test_cs_default_pulse_rev_clock(self):
        if not self._spi.is_inverted_cpha_supported:
            self.skipTest('FTDI does not support mode 3')
        self._port.set_mode(3)
        for _ in range(5):
            self._port.force_select()


def suite():
    suite_ = unittest.TestSuite()
    # suite_.addTest(unittest.makeSuite(SpiTestCase, 'test'))
    # suite_.addTest(unittest.makeSuite(SpiGpioTestCase, 'test'))
    suite_.addTest(unittest.makeSuite(SpiUnalignedTestCase, 'test'))
    suite_.addTest(unittest.makeSuite(SpiCsForceTestCase, 'test'))
    return suite_


def main():
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError as exc:
        raise ValueError(f'Invalid log level: {level}') from exc
    FtdiLogger.set_level(loglevel)
    unittest.main(defaultTest='suite')


if __name__ == '__main__':
    main()
