#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2016-2020, Emmanuel Blot <emmanuel.blot@free.fr>
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
from os import environ
from time import sleep
from sys import modules, stdout
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.gpio import GpioController, GpioException

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring


class GpioTest:
    """
    """

    def __init__(self):
        self._gpio = GpioController()
        self._state = 0  # SW cache of the GPIO output lines

    def open(self, out_pins):
        """Open a GPIO connection, defining which pins are configured as
           output and input"""
        out_pins &= 0xFF
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        self._gpio.configure(url, direction=out_pins)

    def close(self):
        """Close the GPIO connection"""
        self._gpio.close()

    def set_gpio(self, line, high):
        """Set the level of a GPIO ouput pin.

           :param line: specify which GPIO to madify.
           :param high: a boolean value, True for high-level, False for low-level
        """
        if high:
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


class GpioTestCase(TestCase):
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
        for pin in range(8):
            gpio.get_gpio(pin)
            try:
                gpio.set_gpio(pin, True)
            except GpioException:
                self.assertFalse(bool((1 << pin) & mask),
                                 f'exception: pin {pin} should be Low')
            else:
                self.assertTrue(bool((1 << pin) & mask),
                                f'exception: pin {pin} should be High')
            sleep(0.2)
        for pin in range(8):
            try:
                gpio.set_gpio(pin, False)
            except GpioException:
                self.assertFalse(bool((1 << pin) & mask))
            sleep(0.2)
        self.assertRaises(GpioException, gpio.set_gpio, pin+1, True)
        gpio.close()
        self.assertRaises(GpioException, gpio.set_gpio, pin, True)


def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(GpioTestCase, 'test'))
    return suite_


def main():
    import doctest
    doctest.testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError(f'Invalid log level: {level}')
    FtdiLogger.set_level(loglevel)
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
