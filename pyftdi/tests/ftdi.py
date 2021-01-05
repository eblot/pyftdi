#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2010-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import logging
from doctest import testmod
from os import environ
from sys import modules, stdout
from time import sleep, time as now
from unittest import TestCase, TestSuite, SkipTest, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi, FtdiError
from pyftdi.usbtools import UsbTools, UsbToolsError


class FtdiTestCase(TestCase):
    """FTDI driver test case"""

    def test_multiple_interface(self):
        # the following calls used to create issues (several interfaces from
        # the same device). The test expects an FTDI 2232H here
        ftdi1 = Ftdi()
        ftdi1.open(vendor=0x403, product=0x6010, interface=1)
        ftdi2 = Ftdi()
        ftdi2.open(vendor=0x403, product=0x6010, interface=2)
        for _ in range(5):
            print("If#1: ", hex(ftdi1.poll_modem_status()))
            print("If#2: ", ftdi2.modem_status())
            sleep(0.500)
        ftdi1.close()
        ftdi2.close()


class HotplugTestCase(TestCase):

    def test_hotplug_discovery(self):
        """Demonstrate how to connect to an hotplugged FTDI device, i.e.
           an FTDI device that is connected after the initial attempt to
           enumerate it on the USB bus."""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        ftdi = Ftdi()
        timeout = now() + 5.0  # sanity check: bail out after 10 seconds
        while now() < timeout:
            try:
                ftdi.open_from_url(url)
                break
            except UsbToolsError:
                UsbTools.flush_cache()
                sleep(0.05)
                continue
        self.assertTrue(ftdi.is_connected, 'Unable to connect to FTDI')
        print('Connected to FTDI', url)


class ResetTestCase(TestCase):

    def test_simple_reset(self):
        """Demonstrate how to connect to an hotplugged FTDI device, i.e.
           an FTDI device that is connected after the initial attempt to
           enumerate it on the USB bus."""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        ftdi = Ftdi()
        ftdi.open_from_url(url)
        self.assertTrue(ftdi.is_connected, 'Unable to connect to FTDI')
        ftdi.close()
        self.assertFalse(ftdi.is_connected, 'Unable to close connection')
        ftdi.open_from_url(url)
        self.assertTrue(ftdi.is_connected, 'Unable to connect to FTDI')
        ftdi.reset(False)

    def test_dual_if_reset(self):
        """Demonstrate how to connect to an hotplugged FTDI device, i.e.
           an FTDI device that is connected after the initial attempt to
           enumerate it on the USB bus."""
        url1 = environ.get('FTDI_DEVICE', 'ftdi:///1')
        ftdi1 = Ftdi()
        ftdi1.open_from_url(url1)
        count = ftdi1.device_port_count
        if count < 2:
            ftdi1.close()
            raise SkipTest('FTDI device is not a multi-port device')
        next_port = (int(url1[-1]) % count) + 1
        url2 = 'ftdi:///%d' % next_port
        ftdi2 = Ftdi()
        self.assertTrue(ftdi1.is_connected, 'Unable to connect to FTDI')
        ftdi2.open_from_url(url2)
        # use latenty setting to set/test configuration is preserved
        ftdi2.set_latency_timer(128)
        # should be the same value
        self.assertEqual(ftdi2.get_latency_timer(), 128)
        self.assertTrue(ftdi2.is_connected, 'Unable to connect to FTDI')
        ftdi1.close()
        self.assertFalse(ftdi1.is_connected, 'Unable to close connection')
        # closing first connection should not alter second interface
        self.assertEqual(ftdi2.get_latency_timer(), 128)
        ftdi1.open_from_url(url1)
        self.assertTrue(ftdi1.is_connected, 'Unable to connect to FTDI')
        # a FTDI reset should not alter settings...
        ftdi1.reset(False)
        self.assertEqual(ftdi2.get_latency_timer(), 128)
        # ... however performing a USB reset through any interface should alter
        # any previous settings made to all interfaces
        ftdi1.reset(True)
        self.assertNotEqual(ftdi2.get_latency_timer(), 128)


class DisconnectTestCase(TestCase):
    """This test requires user interaction to unplug/plug back the device.
    """

    def test_close_on_disconnect(self):
        """Validate close after disconnect."""
        log = logging.getLogger('pyftdi.tests.ftdi')
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        ftdi = Ftdi()
        ftdi.open_from_url(url)
        self.assertTrue(ftdi.is_connected, 'Unable to connect to FTDI')
        print('Please disconnect FTDI device')
        while ftdi.is_connected:
            try:
                ftdi.poll_modem_status()
            except FtdiError:
                break
            sleep(0.1)
        ftdi.close()
        print('Please reconnect FTDI device')
        while True:
            UsbTools.flush_cache()
            try:
                ftdi.open_from_url(url)
            except (FtdiError, UsbToolsError):
                log.debug('FTDI device not detected')
                sleep(0.1)
            except ValueError:
                log.warning('FTDI device not initialized')
                ftdi.close()
                sleep(0.1)
            else:
                log.info('FTDI device detected')
                break
        ftdi.poll_modem_status()
        ftdi.close()


def suite():
    suite_ = TestSuite()
    #suite_.addTest(makeSuite(FtdiTestCase, 'test'))
    #suite_.addTest(makeSuite(HotplugTestCase, 'test'))
    suite_.addTest(makeSuite(ResetTestCase, 'test'))
    suite_.addTest(makeSuite(DisconnectTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError as exc:
        raise ValueError(f'Invalid log level: {level}') from exc
    FtdiLogger.set_level(loglevel)
    ut_main(defaultTest='suite')
