#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

import logging
from contextlib import redirect_stdout
from doctest import testmod
from io import StringIO
from os import environ
from string import ascii_letters
from sys import modules, stdout
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from urllib.parse import urlsplit
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi, FtdiMpsseError
from pyftdi.gpio import GpioController
from pyftdi.serialext import serial_for_url
from pyftdi.usbtools import UsbTools
from backend.loader import MockLoader

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=no-self-use


class MockUsbToolsTestCase(TestCase):
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
        """Enumerate FTDI devices."""
        ftdis = [(0x403, pid)
                 for pid in (0x6001, 0x6010, 0x6011, 0x6014, 0x6015)]
        count = len(UsbTools.find_all(ftdis))
        self.assertEqual(count, 6)

    def test_device(self):
        """Access and release FTDI device."""
        ftdis = [(0x403, 0x6001)]
        ft232rs = UsbTools.find_all(ftdis)
        self.assertEqual(len(ft232rs), 1)
        devdesc, ifcount = ft232rs[0]
        self.assertEqual(ifcount, 1)
        dev = UsbTools.get_device(devdesc)
        self.assertIsNotNone(dev)
        UsbTools.release_device(dev)

    def test_string(self):
        """Retrieve a string from its identifier."""
        ftdis = [(0x403, 0x6010)]
        ft2232h = UsbTools.find_all(ftdis)[0]
        devdesc, _ = ft2232h
        dev = UsbTools.get_device(devdesc)
        serialn = UsbTools.get_string(dev, dev.iSerialNumber)
        self.assertEqual(serialn, 'FT2DEF')


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
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 10)
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


class MockSimpleDirectTestCase(TestCase):
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
        self.assertEqual(ftdi.usb_path, (1, 1, 0))
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
        self.assertEqual(ftdi.usb_path, (4, 5, 0))
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
        """Check simple GPIO write and read sequence."""
        gpio = GpioController()
        # access to the virtual GPIO port
        out_pins = 0xAA
        gpio.configure('ftdi://:232h/1', direction=out_pins)
        bus, address, iface = gpio.ftdi.usb_path
        self.assertEqual((bus, address, iface), (4, 5, 0))
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        gpio.write_port(0xF3)
        self.assertEqual(vftdi.gpio, 0xAA & 0xF3)
        vftdi.gpio = 0x0c
        vio = gpio.read_port()
        self.assertEqual(vio, (0xAA & 0xF3) | (~0xAA & 0x0c))
        gpio.close()


class MockSimpleUartTestCase(TestCase):
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
        """Check simple TX/RX sequence."""
        port = serial_for_url('ftdi:///1')
        bus, address, _ = port.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        msg = ascii_letters
        port.write(msg.encode())
        buf = vftdi.uart_read(len(ascii_letters)+10).decode()
        self.assertEqual(msg, buf)
        msg = ''.join(reversed(msg))
        vftdi.uart_write(msg.encode())
        buf = port.read(len(ascii_letters)).decode()
        self.assertEqual(msg, buf)
        port.close()


def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(MockUsbToolsTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockDualDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockTwoPortDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockFourPortDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockManyDevicesTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleDirectTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleMpsseTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleGpioTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleUartTestCase, 'test'))
    return suite_


def main():
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'warning').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError(f'Invalid log level: {level}')
    FtdiLogger.set_level(loglevel)
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('backend.usbmock', )
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
