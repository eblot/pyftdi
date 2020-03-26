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

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring

import logging
from os import environ
from sys import modules, stdout
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.gpio import GpioAsyncController, GpioSyncController


class GpioAsyncTestCase(TestCase):
    """FTDI Asynchronous GPIO driver test case.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. At least, b7..b5 can be driven high or
       low, so check your HW setup before running this test as it might
       damage your HW.

       Low nibble is used as input, high nibble is used as output. They should
       be interconnected as follow:

       * b0 should be connected to b4
       * b1 should be connected to b5
       * b2 should be connected to b6
       * b3 should be connected to b7

       Note that this test cannot work with LC231X board, as FTDI is stupid
       enough to add ubidirectionnal output buffer on DTR and RTS, so both
       nibbles have at lest on output pin...

       Do NOT run this test if you use FTDI port A as an UART or SPI
       bridge -or any unsupported setup!! You've been warned.
    """

    @classmethod
    def setUpClass(cls):
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')

    def test_gpio_values(self):
        """Simple test to demonstrate bit-banging.
        """
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio = GpioAsyncController()
        gpio.configure(self.url, direction=direction)
        port = gpio.get_gpio()  # useless, for API duck typing
        # legacy API: peek mode, 1 byte
        ingress = port.read()
        self.assertIsInstance(ingress, int)
        # peek mode always gives a single byte output
        ingress = port.read(peek=True)
        self.assertIsInstance(ingress, int)
        # stream mode always gives a bytes buffer
        ingress = port.read(peek=False)
        self.assertIsInstance(ingress, bytes)
        self.assertEqual(len(ingress), 1)
        # direct mode is not available with multi-byte mode
        self.assertRaises(ValueError, port.read, 3, True)
        ingress = port.read(3)
        self.assertIsInstance(ingress, bytes)
        self.assertEqual(len(ingress), 3)
        port.write(0x00)
        port.write(0xFF)
        # only 8 bit values are accepted
        self.assertRaises(ValueError, port.write, 0x100)
        port.write([0x00, 0xFF, 0x00])
        port.write(bytes([0x00, 0xFF, 0x00]))
        # only 8 bit values are accepted
        self.assertRaises(ValueError, port.write, [0x00, 0x100, 0x00])
        # check direction API
        port.set_direction(0xFF, 0xFF & ~direction)
        gpio.close()

    def test_gpio_loopback(self):
        """Check I/O.
        """
        gpio = GpioAsyncController()
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio.configure(self.url, direction=direction, frequency=800000)
        for out in range(16):
            gpio.write(out << 4)
            fback = gpio.read()
            lsbs = fback & ~direction
            msbs = fback >> 4
            # check inputs match outputs
            self.assertEqual(lsbs, out)
            # check level of outputs match the ones written
            self.assertEqual(msbs, out)
        outs = list([(out & 0xf)<<4 for out in range(1000)])
        gpio.write(outs)
        gpio.ftdi.read_data(512)
        for _ in range(len(outs)):
            gpio.read(14)
        last = outs[-1] >> 4
        for _ in range(10):
            fbacks = gpio.read(1000)
            for fback in fbacks:
                lsbs = fback & ~direction
                msbs = fback >> 4
                # check inputs match last output
                self.assertEqual(lsbs, last)
                # check level of output match the last written
                self.assertEqual(msbs, last)
        gpio.close()

    def test_gpio_baudate(self):
        gpio = GpioAsyncController()
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio.configure(self.url, direction=direction)
        buf = bytes([0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00])
        gpio.set_frequency(10000) # 80 000
        gpio.write(buf)
        gpio.set_frequency(50000) # 400 000
        gpio.write(buf)
        gpio.set_frequency(200000)  # 1 700 000
        gpio.write(buf)
        gpio.close()


class GpioSyncTestCase(TestCase):
    """FTDI Synchrnous GPIO driver test case.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. At least, b7..b5 can be driven high or
       low, so check your HW setup before running this test as it might
       damage your HW.

       Low nibble is used as input, high nibble is used as output. They should
       be interconnected as follow:

       * b0 should be connected to b4
       * b1 should be connected to b5
       * b2 should be connected to b6
       * b3 should be connected to b7

       Note that this test cannot work with LC231X board, as FTDI is stupid
       enough to add ubidirectionnal output buffer on DTR and RTS, so both
       nibbles have at lest on output pin...

       Do NOT run this test if you use FTDI port A as an UART or SPI
       bridge -or any unsupported setup!! You've been warned.
    """

    @classmethod
    def setUpClass(cls):
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')

    def test_gpio_values(self):
        """Simple test to demonstrate bit-banging.
        """
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio = GpioSyncController()
        gpio.configure(self.url, direction=direction, initial=0xee)
        outs = bytes([(out & 0xf)<<4 for out in range(1000)])
        ins = gpio.exchange(outs)
        self.assertEqual(len(outs), len(ins))
        last = None
        for sout, sin in zip(outs, ins):
            if last is not None:
                # output nibble
                sin_out = sin >> 4
                # input nibble
                sin_in = sin & 0xF
                # check inputs match last output
                self.assertEqual(sin_out, last)
                # check level of output match the last written
                self.assertEqual(sin_in, last)
            # an IN sample if captured on the next clock of the OUT sample
            # keep the MSB nibble, i.e. the nibble configured as output
            last = sout >> 4
        gpio.close()

    def _test_gpio_baudate(self):
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio = GpioSyncController()
        gpio.configure(self.url, direction=direction)
        buf = bytes([0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00])
        gpio.set_frequency(10000) # 80 000
        gpio.exchange(buf)
        gpio.set_frequency(50000) # 400 000
        gpio.exchange(buf)
        gpio.set_frequency(200000)  # 1 700 000
        gpio.exchange(buf)
        gpio.close()

def suite():
    suite_ = TestSuite()
    # suite_.addTest(makeSuite(GpioAsyncTestCase, 'test'))
    suite_.addTest(makeSuite(GpioSyncTestCase, 'test'))
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
