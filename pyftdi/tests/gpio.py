#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2016-2017, Emmanuel Blot <emmanuel.blot@free.fr>
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

import unittest
import sys
from os import environ
from pyftdi.gpio import GpioController, GpioException
from time import sleep


class GpioTest(object):
    """
    """

    def __init__(self):
        self._gpio = GpioController()
        self._state = 0  # SW cache of the GPIO output lines

    def open(self, out_pins):
        """Open a GPIO connection, defining which pins are configured as
           output and input"""
        out_pins &= 0xFF
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._gpio.open_from_url(url, direction=out_pins)

    def close(self):
        """Close the GPIO connection"""
        self._gpio.close()

    def set_gpio(self, line, on):
        """Set the level of a GPIO ouput pin.

           :param line: specify which GPIO to madify.
           :param on: a boolean value, True for high-level, False for low-level
        """
        if on:
            state = self._state | (1 << line)
        else:
            state = self._state & ~(1 << line)
        self._commit_state(state)

    def get_gpio(self, line):
        """Retrieve the level of a GPIO input pin

           :param line: specify which GPIO to read out.
           :return: True for high-level, False for low-level
        """
        value = self._gpio.read_port()
        return bool(value & (1 << line))

    def _commit_state(self, state):
        """Update GPIO outputs
        """
        self._gpio.write_port(state)
        # do not update cache on error
        self._state = state


class GpioTestCase(unittest.TestCase):
    """FTDI GPIO driver test case"""

    def test_gpio(self):
        """Simple test to demonstrate bit-banging.

           Please ensure that the HW you connect to the FTDI port A does match
           the encoded configuration. At least, b7..b5 can be driven high or
           low, so check your HW setup before running this test as it might
           damage your HW.

           Do NOT run this test if you use FTDI port A as an UART or SPI
           bridge -or any unsupported setup!! You've been warned.
        """
        gpio = GpioTest()
        mask = 0xE0  # Out, Out, Out, In, In, In, In, In
        gpio.open(mask)
        for gp in range(8):
            gpio.get_gpio(gp)
            try:
                gpio.set_gpio(gp, True)
            except GpioException:
                self.assertFalse(bool((1 << gp) & mask))
            else:
                self.assertTrue(bool((1 << gp) & mask))
            sleep(0.2)
        for gp in range(8):
            try:
                gpio.set_gpio(gp, False)
            except GpioException:
                self.assertFalse(bool((1 << gp) & mask))
            sleep(0.2)
        self.assertRaises(GpioException, gpio.set_gpio, gp+1, True)
        gpio.close()
        self.assertRaises(GpioException, gpio.set_gpio, gp, True)


def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(GpioTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    import doctest
    doctest.testmod(sys.modules[__name__])
    unittest.main(defaultTest='suite')
