#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

import logging
from contextlib import redirect_stdout
from doctest import testmod
from io import StringIO
from os import environ
from sys import modules, stdout
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.usbtools import UsbTools
from pyftdi.tests.backend.usbmock import MockLoader

class MockTestCase(TestCase):
    """
    """

    def test_enumerate(self):
        """Check simple enumeration of a single FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 2)
        self.assertTrue(lines[1].startswith('ftdi://'))
        # skip description, i.e. consider URL only
        self.assertTrue(lines[1].split(' ')[0].endswith('/1'))

    def test_open_close(self):
        """Check simple open/close sequence."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        ftdi.close()

    def test_open_bitbang(self):
        """Check simple open/close BitBang sequence."""
        ftdi = Ftdi()
        ftdi.open_bitbang_from_url('ftdi:///1')
        ftdi.close()

    def test_open_mpsse(self):
        """Check simple open/close MPSSE sequence."""
        ftdi = Ftdi()
        ftdi.open_mpsse_from_url('ftdi:///1')
        ftdi.close()


class MockLoaderTestCase(TestCase):
    """
    """

    def test_1(self):
        loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            loader.load(yfp)
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///?')


def suite():
    suite_ = TestSuite()
    # suite_.addTest(makeSuite(MockTestCase, 'test'))
    suite_.addTest(makeSuite(MockLoaderTestCase, 'test'))
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
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('pyftdi.tests.backend.usbmock' ,)
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
