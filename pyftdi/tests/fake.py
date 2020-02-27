#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

import logging
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from binascii import hexlify
from doctest import testmod
from os import environ
from sys import modules, stderr, stdout
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.usbtools import UsbTools


class FakeTestCase(TestCase):
    """
    """

    def test_load(self):
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///?')


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
