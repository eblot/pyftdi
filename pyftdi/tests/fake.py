#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

import logging
from binascii import hexlify
from contextlib import redirect_stdout
from doctest import testmod
from io import StringIO
from os import environ
from sys import modules, stderr, stdout
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.usbtools import UsbTools


class FakeTestCase(TestCase):
    """
    """

    def _test_enumerate(self):
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
        line_count = len(lines)

    def _test_open(self):
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        ftdi.close()

    def test_open_mpsse(self):
        ftdi = Ftdi()
        ftdi.open_mpsse_from_url('ftdi:///1')
        ftdi.close()

def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(FakeTestCase, 'test'))
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
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('pyftdi.tests.backend.dummy' ,)
    ut_main(defaultTest='suite')
