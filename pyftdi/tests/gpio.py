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
from collections import deque
from os import environ
from sys import modules, stdout
from unittest import TestCase, TestSuite, SkipTest, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.gpio import GpioAsyncController, GpioSyncController


class GpioAsyncTestCase(TestCase):
    """FTDI Asynchronous GPIO driver test case.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. Check your HW setup before running this test
       as it might damage your HW. You've been warned.

       Low nibble is used as input, high nibble is used as output. They should
       be interconnected as follow:

       * b0 should be connected to b4
       * b1 should be connected to b5
       * b2 should be connected to b6
       * b3 should be connected to b7

       Note that this test cannot work with LC231X board, as FTDI is stupid
       enough to add ubidirectionnal output buffer on DTR and RTS, so both
       nibbles have at lest on output pin...
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
       the encoded configuration. Check your HW setup before running this test
       as it might damage your HW. You've been warned.

       Low nibble is used as input, high nibble is used as output. They should
       be interconnected as follow:

       * b0 should be connected to b4
       * b1 should be connected to b5
       * b2 should be connected to b6
       * b3 should be connected to b7

       Note that this test cannot work with LC231X board, as FTDI is stupid
       enough to add ubidirectionnal output buffer on DTR and RTS, so both
       nibbles have at lest on output pin...
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


class GpioMultiportTestCase(TestCase):
    """FTDI GPIO test for multi-port FTDI devices, i.e. FT2232H/FT4232H.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. Check your HW setup before running this test
       as it might damage your HW. You've been warned.

       Low nibble is used as input, high nibble is used as output. They should
       be interconnected as follow:

       * AD0 should be connected to BD0
       * AD1 should be connected to BD1
       * AD2 should be connected to BD2
       * AD3 should be connected to BD3
       * AD0 should be connected to BD0
       * AD1 should be connected to BD1
       * AD2 should be connected to BD2
       * AD3 should be connected to BD3
    """

    @classmethod
    def setUpClass(cls):
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        ftdi = Ftdi()
        ftdi.open_from_url(url)
        count = ftdi.device_port_count
        pos = ftdi.port_index
        ftdi.close()
        if pos != 1:
            raise ValueError("FTDI interface should be the device's first")
        if count < 2:
            raise SkipTest('FTDI device is not a multi-port device')
        url = url[:-1]
        cls.urls = [f'{url}1', f'{url}2']

    def test_gpio_peek(self):
        """Check I/O.
        """
        gpio_in, gpio_out = GpioAsyncController(), GpioAsyncController()
        gpio_in.configure(self.urls[0], direction=0x00, frequency=1e6)
        gpio_out.configure(self.urls[1], direction=0xFF, frequency=1e6)
        for out in range(256):
            gpio_out.write(out)
            outv = gpio_out.read()
            inv = gpio_in.read()
            # check inputs match outputs
            self.assertEqual(inv, out)
            # check level of outputs match the ones written
            self.assertEqual(outv, out)
            print(f'{outv:08b} --> {inv:08b}')
        gpio_in.close()
        gpio_out.close()

    def test_gpio_stream(self):
        """Check I/O.
        """
        gpio_in, gpio_out = GpioAsyncController(), GpioAsyncController()
        gpio_in.configure(self.urls[0], direction=0x00, frequency=1e4)
        gpio_out.configure(self.urls[1], direction=0xFF, frequency=1e4)
        outs = bytes(range(256))
        gpio_out.write(outs)
        # read @ same speed (and same clock source, so no jitter), flushing
        # the byffer which has been filled since the port has been opened
        ins = gpio_in.read(len(outs), flush=True)
        qout = deque(outs)
        ifirst = ins[0]
        # the inout stream should be a copy of the output stream, minus a
        # couple of missing samples that did not get captured while output
        # was streaming but read command has not been yet received.
        while qout:
            if qout[0] == ifirst:
                break
            qout.popleft()
        # offset is the count of missed bytes
        offset = len(ins)-len(qout)
        self.assertLess(offset, 16) # seems to be in the 6..12 range
        # print('OFFSET', offset)
        # check that the remaining sequence match
        for sout, sin in zip(qout, ins):
            #print(f'{sout:08b} --> {sin:08b}')
            # check inputs match outputs
            self.assertEqual(sout, sin)
        gpio_in.close()
        gpio_out.close()


class GpioMpsseTestCase(TestCase):
    """FTDI GPIO test for 16-bit port FTDI devices, i.e. FT2232H.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. Check your HW setup before running this test
       as it might damage your HW. You've been warned.

       Low nibble is used as input, high nibble is used as output. They should
       be interconnected as follow:

       * AD0 should be connected to BD0
       * AD1 should be connected to BD1
       * AD2 should be connected to BD2
       * AD3 should be connected to BD3
       * AD0 should be connected to BD0
       * AD1 should be connected to BD1
       * AD2 should be connected to BD2
       * AD3 should be connected to BD3
       * AC0 should be connected to BC0
       * AC1 should be connected to BC1
       * AC2 should be connected to BC2
       * AC3 should be connected to BC3
       * AC0 should be connected to BC0
       * AC1 should be connected to BC1
       * AC2 should be connected to BC2
       * AC3 should be connected to BC3
    """


def suite():
    suite_ = TestSuite()
    # suite_.addTest(makeSuite(GpioAsyncTestCase, 'test'))
    # suite_.addTest(makeSuite(GpioSyncTestCase, 'test'))
    suite_.addTest(makeSuite(GpioMultiportTestCase, 'test'))
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
