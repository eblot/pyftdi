#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

import logging
from contextlib import redirect_stdout
from doctest import testmod
from io import StringIO
from os import environ
from sys import modules, stdout, stderr
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from urllib.parse import urlsplit
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi, FtdiMpsseError
from pyftdi.gpio import GpioController
from pyftdi.usbtools import UsbTools
from pyftdi.tests.backend.loader import MockLoader

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=no-self-use


class MockSimpleDeviceTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_enumerate(self):
        """Check simple enumeration of a single FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith('ftdi://'))
        # skip description, i.e. consider URL only
        self.assertTrue(lines[0].split(' ')[0].endswith('/1'))


class MockDualDeviceTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h_x2.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_enumerate(self):
        """Check simple enumeration of a 2-port FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 2)
        for line in lines:
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            self.assertTrue(line.split(' ')[0].endswith('/1'))


class MockTwoPortDeviceTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft2232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_enumerate(self):
        """Check simple enumeration of a 4-port FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 2)
        for pos, line in enumerate(lines, start=1):
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            self.assertTrue(line.split(' ')[0].endswith(f'/{pos}'))


class MockFourPortDeviceTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft4232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_enumerate(self):
        """Check simple enumeration of two similar FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 4)
        for pos, line in enumerate(lines, start=1):
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            self.assertTrue(line.split(' ')[0].endswith(f'/{pos}'))


class MockManyDevicesTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ftmany.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_enumerate(self):
        """Check simple enumeration of two similar FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        # print(temp_stdout.getvalue(), file=stderr)
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 9)
        for line in lines:
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            url = line.split(' ')[0]
            urlparts = urlsplit(url)
            self.assertEqual(urlparts.scheme, 'ftdi')
            parts = urlparts.netloc.split(':')
            if parts[1] == '4232':
                # def file contains no serial number, so expect bus:addr syntax
                self.assertEqual(len(parts), 4)
                self.assertRegex(parts[2], r'^\d$')
                self.assertRegex(parts[3], r'^\d$')
            else:
                # other devices are assigned a serial number
                self.assertEqual(len(parts), 3)
                self.assertTrue(parts[2].startswith('FT'))
            self.assertRegex(urlparts.path, r'^/\d$')


class MockSimpleUartTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_open_close(self):
        """Check simple open/close sequence."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self.assertEqual(ftdi.location, (1, 1))
        ftdi.close()

    def test_open_bitbang(self):
        """Check simple open/close BitBang sequence."""
        ftdi = Ftdi()
        ftdi.open_bitbang_from_url('ftdi:///1')
        ftdi.close()

    def test_open_mpsse(self):
        """Check simple MPSSE access."""
        ftdi = Ftdi()
        # FT230X is a pure UART bridge, MPSSE should not be available
        self.assertRaises(FtdiMpsseError,
                          ftdi.open_mpsse_from_url, 'ftdi:///1')


class MockSimpleMpsseTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_open_close(self):
        """Check simple open/close sequence."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self.assertEqual(ftdi.location, (4, 5))
        ftdi.close()

    def test_open_bitbang(self):
        """Check simple open/close BitBang sequence."""
        ftdi = Ftdi()
        ftdi.open_bitbang_from_url('ftdi:///1')
        ftdi.close()

    def test_open_mpsse(self):
        """Check simple MPSSE access."""
        ftdi = Ftdi()
        ftdi.open_mpsse_from_url('ftdi:///1')
        ftdi.close()


class MockSimpleGpioTestCase(TestCase):
    """
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test(self):
        """Check simple open/close sequence."""
        gpio = GpioController()
        # access to the virtual GPIO port
        out_pins = 0xAA
        gpio.configure('ftdi://:232h/1', direction=out_pins)
        bus, address = gpio.ftdi.location
        self.assertEqual((bus, address), (4, 5))
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        gpio.write_port(0xF3)
        self.assertEqual(vftdi.gpio, 0xAA & 0xF3)
        vftdi.gpio = 0x0c
        vio = gpio.read_port()
        self.assertEqual(vio, (0xAA & 0xF3) | (~0xAA & 0x0c))
        gpio.close()


def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(MockSimpleDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockDualDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockTwoPortDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockFourPortDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockManyDevicesTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleUartTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleMpsseTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleGpioTestCase, 'test'))
    return suite_


def main():
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError(f'Invalid log level: {level}')
    FtdiLogger.set_level(loglevel)
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('pyftdi.tests.backend.usbmock', )
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
