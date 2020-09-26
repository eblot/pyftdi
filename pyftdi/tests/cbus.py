#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import sys
from doctest import testmod
from os import environ
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from pyftdi.ftdi import Ftdi, FtdiError
from pyftdi.eeprom import FtdiEeprom

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring


class CbusGpioTestCase(TestCase):
    """FTDI CBUS GPIO feature test case"""

    @classmethod
    def setUpClass(cls):
        """Default values"""
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')

    def test_output_gpio(self):
        """Simple test to demonstrate ouput bit-banging on CBUS.

           You need a CBUS-capable FTDI (FT232R/FT232H/FT230X/FT231X), whose
           EEPROM has been configured to support GPIOs on CBUS0 and CBUS3.

           Hard-wiring is required to run this test:
           * CBUS0 (output) should be connected to CTS (input)
           * CBUS3 (output) should be connected to DSR (input)
        """
        ftdi = Ftdi()
        ftdi.open_from_url(self.url)
        # sanity check: device should support CBUS feature
        self.assertEqual(ftdi.has_cbus, True)
        eeprom = FtdiEeprom()
        eeprom.connect(ftdi)
        # sanity check: device should have been configured for CBUS GPIOs
        self.assertEqual(eeprom.cbus_mask & 0b1001, 0b1001)
        # configure CBUS0 and CBUS3 as output
        ftdi.set_cbus_direction(0b1001, 0b1001)
        # no input pin available
        self.assertRaises(FtdiError, ftdi.get_cbus_gpio)
        for cycle in range(40):
            value = cycle & 0x3
            # CBUS0 and CBUS3
            cbus = ((value & 0x2) << 2) | value & 0x1
            # for now, need a digital/logic analyzer to validate output
            ftdi.set_cbus_gpio(cbus)
            # CBUS0 is connected to CTS, CBUS3 to DSR
            # need to inverse logical level as RS232 uses negative logic
            sig = int(not ftdi.get_cts()) | (int(not ftdi.get_dsr()) << 1)
            self.assertEqual(value, sig)

    def test_input_gpio(self):
        """Simple test to demonstrate input bit-banging on CBUS.

           You need a CBUS-capable FTDI (FT232R/FT232H/FT230X/FT231X), whose
           EEPROM has been configured to support GPIOs on CBUS0 and CBUS3.

           Hard-wiring is required to run this test:
           * CBUS0 (input) should be connected to RTS (output)
           * CBUS3 (input) should be connected to DTR (output)
        """
        ftdi = Ftdi()
        ftdi.open_from_url(self.url)
        # sanity check: device should support CBUS feature
        self.assertEqual(ftdi.has_cbus, True)
        eeprom = FtdiEeprom()
        eeprom.connect(ftdi)
        # sanity check: device should have been configured for CBUS GPIOs
        self.assertEqual(eeprom.cbus_mask & 0b1001, 0b1001)
        # configure CBUS0 and CBUS3 as input
        ftdi.set_cbus_direction(0b1001, 0b0000)
        # no output pin available
        self.assertRaises(FtdiError, ftdi.set_cbus_gpio, 0)
        for cycle in range(40):
            rts = bool(cycle & 0x1)
            dtr = bool(cycle & 0x2)
            ftdi.set_rts(rts)
            ftdi.set_dtr(dtr)
            # need to inverse logical level as RS232 uses negative logic
            cbus = ~ftdi.get_cbus_gpio()
            sig = (cbus & 0x1) | ((cbus & 0x8) >> 2)
            value = cycle & 0x3
            self.assertEqual(value, sig)


def suite():
    suite_ = TestSuite()
    # peak the test that matches your HW setup, see test doc for details
    # suite_.addTest(makeSuite(CbusGpioTestCase, 'test_output'))
    suite_.addTest(makeSuite(CbusGpioTestCase, 'test_input'))
    return suite_


def main():
    testmod(sys.modules[__name__])
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
