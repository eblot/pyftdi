#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2016-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=invalid-name
#pylint: disable-msg=global-statement

import logging
from collections import deque
from os import environ
from sys import modules, stdout
from time import sleep
from unittest import TestCase, TestSuite, SkipTest, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.gpio import (GpioAsyncController,
                         GpioSyncController,
                         GpioMpsseController)
from pyftdi.misc import to_bool

# When USB virtualization is enabled, this loader is instanciated
VirtLoader = None


class FtdiTestCase(TestCase):
    """Common features for all tests.
    """

    @classmethod
    def setUpClass(cls):
        cls.debug = to_bool(environ.get('FTDI_DEBUG', 'off'), permissive=False)
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        cls.loader = None

    @classmethod
    def tearDownClass(cls):
        if cls.loader:
            cls.loader.unload()

    def setUp(self):
        if self.debug:
            print('.'.join(self.id().split('.')[-2:]))


class GpioAsyncTestCase(FtdiTestCase):
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
        FtdiTestCase.setUpClass()
        if VirtLoader:
            cls.loader = VirtLoader()
            with open('pyftdi/tests/resources/ft232r.yaml', 'rb') as yfp:
                cls.loader.load(yfp)
            vftdi = cls.loader.get_virtual_ftdi(1, 1)
            vport = vftdi.get_port(1)
            # create virtual connections as real HW
            in_pins = [vport[pos] for pos in range(4)]
            out_pins = [vport[pos] for pos in range(4, 8)]
            for in_pin, out_pin in zip(in_pins, out_pins):
                out_pin.connect_to(in_pin)
        if cls.url == 'ftdi:///1':
            # assumes that if not specific device is used, and a multiport
            # device is connected, there is no loopback wires between pins of
            # the same port. This hack allows to run the same test with a
            # FT232H, then a FT2232H for ex, to that with two test sessions
            # the whole test set is run. If a specific device is selected
            # assume the HW always match the expected configuration.
            ftdi = Ftdi()
            ftdi.open_from_url(cls.url)
            count = ftdi.device_port_count
            ftdi.close()
            cls.skip_loopback = count > 1
        else:
            cls.skip_loopback = False

    def test_gpio_freeze(self):
        """Simple test to demonstrate freeze on close.

           For now, it requires a logic analyzer to verify the output,
           this is not automatically validated by SW
        """
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio = GpioAsyncController()
        gpio.configure(self.url, direction=direction, frequency=1e3,
                       initial=0x0)
        port = gpio.get_gpio()
        # emit a sequence as a visual marker on b3,b2,b1,b0
        port.write([x<<4 for x in range(16)])
        sleep(0.01)
        # write 0b0110 to the port
        port.write(0x6<<4)
        sleep(0.001)
        # close w/o freeze: all the outputs should be reset (usually 0b1111)
        # it might need pull up (or pull down) to observe the change as
        # output are not high-Z.
        gpio.close()
        sleep(0.01)
        gpio.configure(self.url, direction=direction, frequency=1e3,
                       initial=0x0)
        port = gpio.get_gpio()
        # emit a sequence as a visual marker with on b3 and b1
        port.write([(x<<4)&0x90 for x in range(16)])
        sleep(0.01)
        # write 0b0110 to the port
        port.write(0x6<<4)
        sleep(0.01)
        # close w/ freeze: outputs should not be reset (usually 0b0110)
        gpio.close(True)


    def test_gpio_values(self):
        """Simple test to demonstrate bit-banging.
        """
        if self.skip_loopback:
            raise SkipTest('Skip loopback test on multiport device')
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio = GpioAsyncController()
        gpio.configure(self.url, direction=direction, frequency=1e6,
                       initial=0x0)
        port = gpio.get_gpio()  # useless, for API duck typing
        # legacy API: peek mode, 1 byte
        ingress = port.read()
        self.assertIsInstance(ingress, int)
        # peek mode always gives a single byte output
        ingress = port.read(peek=True)
        self.assertIsInstance(ingress, int)
        # stream mode always gives a bytes buffer
        port.write([0xaa for _ in range(256)])
        ingress = port.read(100, peek=False, noflush=False)
        self.assertIsInstance(ingress, bytes)
        if not VirtLoader:
            # the virtual task may sometimes not be triggered soon enough
            self.assertGreater(len(ingress), 2)
        # direct mode is not available with multi-byte mode
        self.assertRaises(ValueError, port.read, 3, True)
        ingress = port.read(3)
        self.assertIsInstance(ingress, bytes)
        if not VirtLoader:
            # the virtual task may sometimes not be triggered soon enough
            self.assertGreater(len(ingress), 0)
        self.assertLessEqual(len(ingress), 3)
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

    def test_gpio_initial(self):
        """Check initial values.
        """
        if self.skip_loopback:
            raise SkipTest('Skip initial test on multiport device')
        if not self.loader:
            raise SkipTest('Skip initial test on physical device')
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        vftdi = self.loader.get_virtual_ftdi(1, 1)
        vport = vftdi.get_port(1)
        gpio = GpioAsyncController()
        for initial in (0xaf, 0xf0, 0x13, 0x00):
            gpio.configure(self.url, direction=direction, frequency=1e6,
                           initial=initial)
            expect = (initial & 0xF0) | (initial >> 4)
            self.assertEqual(vport.gpio, expect)
            gpio.close()

    def test_gpio_loopback(self):
        """Check I/O.
        """
        if self.skip_loopback:
            raise SkipTest('Skip loopback test on multiport device')
        gpio = GpioAsyncController()
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio.configure(self.url, direction=direction, frequency=800000)
        for out in range(16):
            # print(f'Write {out:04b} -> {out << 4:08b}')
            gpio.write(out << 4)
            fback = gpio.read()
            lsbs = fback & ~direction
            msbs = fback >> 4
            # check inputs match outputs
            self.assertEqual(lsbs, out)
            # check level of outputs match the ones written
            self.assertEqual(msbs, out)
        outs = list([(out & 0xf) << 4 for out in range(1000)])
        gpio.write(outs)
        gpio.ftdi.read_data(512)
        for _ in range(len(outs)):
            _ = gpio.read(14)
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
        # this test requires an external device (logic analyser or scope) to
        # check the bitbang read and bitbang write signal (BB_RD, BB_WR) and
        # mesure their frequency. The EEPROM should be configured to enable
        # those signal on some of the CBUS pins, for example.
        gpio = GpioAsyncController()
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio.configure(self.url, direction=direction)
        buf = bytes([0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00])
        freqs = [50e3, 200e3, 1e6, 3e6]
        if gpio.ftdi.is_H_series:
            freqs.extend([6e6, 10e6, 12e6])
        gpio.read(128)
        for freq in freqs:
            # set the bitbang refresh rate
            gpio.set_frequency(freq)
            self.assertEqual(gpio.frequency, freq)
            # be sure to leave enough time to purge buffers (HW FIFO) or
            # the frequency changes occur on the current buffer...
            gpio.write(buf)
            gpio.read(128)
            sleep(0.01)
        gpio.close()


class GpioSyncTestCase(FtdiTestCase):
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
        FtdiTestCase.setUpClass()
        if VirtLoader:
            cls.loader = VirtLoader()
            with open('pyftdi/tests/resources/ft232r.yaml', 'rb') as yfp:
                cls.loader.load(yfp)
            vftdi = cls.loader.get_virtual_ftdi(1, 1)
            vport = vftdi.get_port(1)
            # create virtual connections as real HW
            in_pins = [vport[pos] for pos in range(4)]
            out_pins = [vport[pos] for pos in range(4, 8)]
            for in_pin, out_pin in zip(in_pins, out_pins):
                out_pin.connect_to(in_pin)
        if cls.url == 'ftdi:///1':
            # assumes that if not specific device is used, and a multiport
            # device is connected, there is no loopback wires between pins of
            # the same port. This hack allows to run the same test with a
            # FT232H, then a FT2232H for ex, to that with two test sessions
            # the whole test set is run. If a specific device is selected
            # assume the HW always match the expected configuration.
            ftdi = Ftdi()
            ftdi.open_from_url(cls.url)
            count = ftdi.device_port_count
            ftdi.close()
            cls.skip_loopback = count > 1
        else:
            cls.skip_loopback = False

    def test_gpio_values(self):
        """Simple test to demonstrate bit-banging.
        """
        if self.skip_loopback:
            raise SkipTest('Skip loopback test on multiport device')
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio = GpioSyncController()
        gpio.configure(self.url, direction=direction, initial=0xee)
        outs = bytes([(out & 0xf)<<4 for out in range(1000)])
        ins = gpio.exchange(outs)
        exp_in_count = min(len(outs), gpio.ftdi.fifo_sizes[0])
        self.assertEqual(len(ins), exp_in_count)
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

    def test_gpio_baudate(self):
        # this test requires an external device (logic analyser or scope) to
        # check the bitbang read and bitbang write signal (BB_RD, BB_WR) and
        # mesure their frequency. The EEPROM should be configured to enable
        # those signal on some of the CBUS pins, for example.
        gpio = GpioSyncController()
        direction = 0xFF & ~((1 << 4) - 1) # 4 Out, 4 In
        gpio.configure(self.url, direction=direction)
        buf = bytes([0xf0, 0x00] * 64)
        freqs = [50e3, 200e3, 1e6, 3e6]
        if gpio.ftdi.is_H_series:
            freqs.extend([6e6, 10e6, 12e6])
        for freq in freqs:
            # set the bitbang refresh rate
            gpio.set_frequency(freq)
            self.assertEqual(gpio.frequency, freq)
            # be sure to leave enough time to purge buffers (HW FIFO) or
            # the frequency changes occur on the current buffer...
            gpio.exchange(buf)
            sleep(0.01)
        gpio.close()


class GpioMultiportTestCase(FtdiTestCase):
    """FTDI GPIO test for multi-port FTDI devices, i.e. FT2232H/FT4232H.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. Check your HW setup before running this test
       as it might damage your HW. You've been warned.

       First port is used as input, second port is used as output. They should
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
        FtdiTestCase.setUpClass()
        if VirtLoader:
            cls.loader = VirtLoader()
            with open('pyftdi/tests/resources/ft2232h.yaml', 'rb') as yfp:
                cls.loader.load(yfp)
            vftdi = cls.loader.get_virtual_ftdi(1, 1)
            vport1 = vftdi.get_port(1)
            vport2 = vftdi.get_port(2)
            # create virtual connections as real HW
            in_pins = [vport1[pos] for pos in range(8)]
            out_pins = [vport2[pos] for pos in range(8)]
            for in_pin, out_pin in zip(in_pins, out_pins):
                out_pin.connect_to(in_pin)
        ftdi = Ftdi()
        ftdi.open_from_url(cls.url)
        count = ftdi.device_port_count
        pos = ftdi.port_index
        ftdi.close()
        if pos != 1:
            raise ValueError("FTDI interface should be the device's first")
        if count < 2:
            raise SkipTest('FTDI device is not a multi-port device')
        url = cls.url[:-1]
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
        gpio_in.close()
        gpio_out.close()

    def test_gpio_stream(self):
        """Check I/O streaming
        """
        if VirtLoader:
            # this would require to synchronize virtual clock between all ports
            # which is not supported by the virtual framework
            raise SkipTest('Skip gpio stream with virtual device')
        gpio_in, gpio_out = GpioAsyncController(), GpioAsyncController()
        gpio_in.configure(self.urls[0], direction=0x00, frequency=1e4)
        gpio_out.configure(self.urls[1], direction=0xFF, frequency=1e4)
        outs = bytes(range(256))
        gpio_out.write(outs)
        # read @ same speed (and same clock source, so no jitter), flushing
        # the byffer which has been filled since the port has been opened
        ins = gpio_in.read(len(outs))
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
        self.assertGreater(offset, 0) # no more output than input
        self.assertLess(offset, 16) # seems to be in the 6..12 range
        # print('Offset', offset)
        # check that the remaining sequence match
        for sout, sin in zip(qout, ins):
            #print(f'{sout:08b} --> {sin:08b}')
            # check inputs match outputs
            self.assertEqual(sout, sin)
        gpio_in.close()
        gpio_out.close()

    def test_direction(self):
        b5 = 1 << 5
        for controller in (GpioAsyncController,
                           GpioSyncController,
                           GpioMpsseController):
            gpio_in, gpio_out = controller(), controller()
            gpio_in.configure(self.urls[0], direction=0x00, frequency=1e6,
                              debug=self.debug)
            gpio_out.configure(self.urls[1], direction=0xFF, frequency=1e6,
                               debug=self.debug)
            for direction in None, 0xFF, b5, 0xF0:
                if direction is not None:
                    gpio_out.set_direction(0xFF, direction)
                for out in 0 << 5, 1 << 5:
                    if controller != GpioSyncController:
                        gpio_out.write(out)
                        outp = gpio_out.read(1)
                        inp = gpio_in.read(1)
                        if controller == GpioMpsseController:
                            outp = outp[0]
                            inp = inp[0]
                    else:
                        # write twice the output value, only the second value
                        # matters (see Sync bitbang for details)
                        outp = gpio_out.exchange(bytes([out, out]))[1]
                        # write anything as we just need the input value which
                        # is sampled on each write
                        inp = gpio_in.exchange(b'\x00')[0]
                    self.assertEqual(outp & b5, out)
                    self.assertEqual(inp & b5, out)


class GpioMpsseTestCase(FtdiTestCase):
    """FTDI GPIO test for 16-bit port FTDI devices, i.e. FT2232H.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. Check your HW setup before running this test
       as it might damage your HW. You've been warned.

       First port is used as input, second port is used as output. They should
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

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        if VirtLoader:
            cls.loader = VirtLoader()
            with open('pyftdi/tests/resources/ft2232h.yaml', 'rb') as yfp:
                cls.loader.load(yfp)
            vftdi = cls.loader.get_virtual_ftdi(1, 1)
            vport1 = vftdi.get_port(1)
            vport2 = vftdi.get_port(2)
            # create virtual connections as real HW
            in_pins = [vport1[pos] for pos in range(16)]
            out_pins = [vport2[pos] for pos in range(16)]
            for in_pin, out_pin in zip(in_pins, out_pins):
                out_pin.connect_to(in_pin)
            # prevent from using the tracer twice (Ftdi & VirtualFtdi)
            cls.debug_mpsse = False
        else:
            cls.debug_mpsse = cls.debug
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        ftdi = Ftdi()
        ftdi.open_from_url(url)
        count = ftdi.device_port_count
        width = ftdi.port_width
        ftdi.close()
        if count < 2:
            raise SkipTest('FTDI device is not a multi-port device')
        if width < 2:
            raise SkipTest('FTDI device does not support wide ports')
        url = url[:-1]
        cls.urls = [f'{url}1', f'{url}2']

    def test_default_gpio(self):
        """Check I/O.
        """
        gpio_in, gpio_out = GpioMpsseController(), GpioMpsseController()
        gpio_in.configure(self.urls[0], direction=0x0000, frequency=10e6,
                          debug=self.debug_mpsse)
        gpio_out.configure(self.urls[1], direction=0xFFFF, frequency=10e6,
                           debug=self.debug_mpsse)
        for out in range(3, 0x10000, 29):
            gpio_out.write(out)
            outv = gpio_out.read()[0]
            inv = gpio_in.read()[0]
            # check inputs match outputs
            self.assertEqual(inv, out)
            # check level of outputs match the ones written
            self.assertEqual(outv, out)
        gpio_in.close()
        gpio_out.close()

    def test_peek_gpio(self):
        """Check I/O peeking
        """
        gpio_in, gpio_out = GpioMpsseController(), GpioMpsseController()
        gpio_in.configure(self.urls[0], direction=0xFF00, frequency=10e6,
                          debug=self.debug)
        gpio_out.configure(self.urls[1], direction=0x00FF, frequency=10e6,
                           debug=self.debug)
        for out in range(256):
            gpio_out.write(out)
            outv = gpio_out.read()[0]
            inv = gpio_in.read(peek=True)
            # check inputs match outputs
            self.assertEqual(inv, out)
            #print(f'{out} {inv}')
            # check level of outputs match the ones written
            self.assertEqual(outv, out)
        gpio_in.close()
        gpio_out.close()

    def test_stream_gpio(self):
        """Check I/O streaming.

           Beware this test is CPU intensive w/ virtual framework
        """
        gpio_in, gpio_out = GpioMpsseController(), GpioMpsseController()
        gpio_in.configure(self.urls[0], direction=0x0000, frequency=10e6,
                          debug=self.debug)
        gpio_out.configure(self.urls[1], direction=0xFFFF, frequency=10e6,
                           debug=self.debug)
        outv = list(range(0, 0x10000, 29))
        max_count = min(gpio_out.ftdi.fifo_sizes[0],
                        gpio_in.ftdi.fifo_sizes[1])//2  # 2 bytes/value
        outv = outv[:max_count]
        gpio_out.write(outv)
        inv = gpio_in.read(len(outv))
        # for now, it is hard to test value exactness
        self.assertEqual(len(outv), len(inv))
        gpio_in.close()
        gpio_out.close()


def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(GpioAsyncTestCase, 'test'))
    suite_.addTest(makeSuite(GpioSyncTestCase, 'test'))
    suite_.addTest(makeSuite(GpioMpsseTestCase, 'test'))
    suite_.addTest(makeSuite(GpioMultiportTestCase, 'test'))
    return suite_


def virtualize():
    if not to_bool(environ.get('FTDI_VIRTUAL', 'off')):
        return
    from pyftdi.usbtools import UsbTools
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('backend.usbvirt', )
    # Ensure the virtual backend can be found and is loaded
    backend = UsbTools.find_backend()
    try:
        # obtain the loader class associated with the virtual backend
        global VirtLoader
        VirtLoader = backend.create_loader()
    except AttributeError as exc:
        raise AssertionError('Cannot load virtual USB backend') from exc


def main():
    import doctest
    doctest.testmod(modules[__name__])
    debug = to_bool(environ.get('FTDI_DEBUG', 'off'))
    if debug:
        formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-7s'
                                      ' %(name)-20s [%(lineno)4d] %(message)s',
                                      '%H:%M:%S')
    else:
        formatter = logging.Formatter('%(message)s')
    level = environ.get('FTDI_LOGLEVEL', 'warning').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError as exc:
        raise ValueError(f'Invalid log level: {level}') from exc
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    FtdiLogger.set_level(loglevel)
    FtdiLogger.set_formatter(formatter)
    virtualize()
    try:
        ut_main(defaultTest='suite')
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    # Useful environment variables:
    #  FTDI_DEVICE: a specific FTDI URL, default to ftdi:///1
    #  FTDI_LOGLEVEL: a Logger debug level, to define log verbosity
    #  FTDI_DEBUG: to enable/disable debug mode
    #  FTDI_VIRTUAL: to use a virtual device rather than a physical device
    main()
