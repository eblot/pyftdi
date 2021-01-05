#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2017-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import logging
from doctest import testmod
from os import environ
from multiprocessing import Process, set_start_method
from random import choice, seed
from string import printable
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from sys import modules, platform, stdout
from time import sleep, time as now
from threading import Thread
from unittest import TestCase, TestSuite, skipIf, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.misc import to_bool
from pyftdi.serialext import serial_for_url

#pylint: disable-msg=missing-docstring
#pylint: disable-msg=protected-access

# Specify the second port for multi port device
# Unfortunately, auto detection triggers some issue in multiprocess test
URL_BASE = environ.get('FTDI_DEVICE', 'ftdi:///2')
URL = ''.join((URL_BASE[:-1], '1'))
IFCOUNT = int(URL_BASE[-1])


class FtdiTestCase(TestCase):
    """Common features for all tests.
    """

    @classmethod
    def setUpClass(cls):
        cls.debug = to_bool(environ.get('FTDI_DEBUG', 'off'),
                                 permissive=False)

    def setUp(self):
        if self.debug:
            print('.'.join(self.id().split('.')[-2:]))


class UartTestCase(FtdiTestCase):
    """FTDI UART driver test case

       Simple test to demonstrate UART feature.

       Depending on your FTDI device, you need to either:
         * connect RXD and TXD on the unique port for 1-port FTDI device
           (AD0 and AD1)
         * connect RXD1 and TXD2, and RXD2 and TXD1 on the two first port of
           2- or 4- port FTDI devices (AD0 - BD1 and AD1 - BD0)

       Do NOT run this test if you use FTDI port(s) as an SPI or I2C
       bridge -or any unsupported setup!! You've been warned.
    """

    COUNT = 512

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        seed()

    @skipIf(IFCOUNT < 2, 'Device has not enough UART interfaces')
    def test_uart_cross_talk_sp(self):
        """Exchange a random byte stream between the two first UART interfaces
           of the same FTDI device, from the same process

           This also validates PyFtdi support to use several interfaces on the
           same FTDI device from the same Python process
        """
        something_out = self.generate_bytes()
        urla = URL
        urlb = self.build_next_url(urla)
        porta = serial_for_url(urla, baudrate=1000000)
        portb = serial_for_url(urlb, baudrate=1000000)
        try:
            if not porta.is_open:
                porta.open()
            if not portb.is_open:
                portb.open()
            # print("porta: %d:%d:%d" % porta.usb_path)
            # print("portb: %d:%d:%d" % portb.usb_path)
            porta.timeout = 1.0
            portb.timeout = 1.0
            something_out = self.generate_bytes()
            porta.write(something_out)
            something_in = portb.read(len(something_out))
            self.assertEqual(len(something_in), len(something_out))
            self.assertEqual(something_in, something_out)
            something_out = self.generate_bytes()
            portb.write(something_out)
            something_in = porta.read(len(something_out))
            self.assertEqual(len(something_in), len(something_out))
            self.assertEqual(something_in, something_out)
        finally:
            porta.close()
            portb.close()

    @skipIf(IFCOUNT < 2, 'Device has not enough UART interfaces')
    @skipIf(platform == 'win32', 'Not tested on Windows')
    def test_uart_cross_talk_mp(self):
        if IFCOUNT > 4:
            raise IOError('No FTDI device')
        urla = URL
        urlb = self.build_next_url(urla)
        something_out = self.generate_bytes()
        proca = Process(target=self._cross_talk_write_then_read,
                        args=(urla, something_out))
        procb = Process(target=self._cross_talk_read_then_write,
                        args=(urlb, something_out))
        # start B first to ensure RX port is open before TX port starts
        # emitting
        procb.start()
        sleep(0.25)
        proca.start()
        proca.join(2)
        procb.join(2)
        # although the failure is reported (and traceback shown) in the
        # subprocess, we still need to fail the main process test in this case
        exita = proca.exitcode
        exitb = procb.exitcode
        self.assertEqual(exita, 0)
        self.assertEqual(exitb, 0)

    @skipIf(IFCOUNT != 1, 'Test reserved for single-port FTDI device')
    def test_uart_loopback(self):
        """Exchange a random byte stream between the two first UART interfaces
           of the same FTDI device, from the same process

           This also validates PyFtdi support to use several interfaces on the
           same FTDI device from the same Python process
        """
        something_out = self.generate_bytes()
        port = serial_for_url(URL, baudrate=1000000)
        for _ in range(10):
            try:
                if not port.is_open:
                    port.open()
                port.timeout = 1.0
                something_out = self.generate_bytes()
                port.write(something_out)
                something_in = port.read(len(something_out))
                self.assertEqual(len(something_in), len(something_out))
                self.assertEqual(something_in, something_out)
            finally:
                port.close()

    @skipIf(IFCOUNT < 2, 'Device has not enough UART interfaces')
    def test2_uart_cross_talk_speed(self):
        urla = URL
        urlb = self.build_next_url(urla)
        porta = serial_for_url(urla, baudrate=6000000)
        portb = serial_for_url(urlb, baudrate=6000000)
        size = int(1e6)
        results = [None, None]
        chunk = 537
        source = Thread(target=self._stream_source,
                        args=(porta, chunk, size, results),
                        daemon=True)
        sink = Thread(target=self._stream_sink,
                      args=(portb, size, results),
                      daemon=True)
        sink.start()
        source.start()
        source.join()
        sleep(0.5)
        sink.join()
        if isinstance(results[1], Exception):
            #pylint: disable-msg=raising-bad-type
            raise results[1]
        #pylint: disable-msg=unpacking-non-sequence
        tsize, tdelta = results[0]
        rsize, rdelta = results[1]
        self.assertGreater(rsize, 0, 'Not data received')
        if self.debug:
            print(f'TX: {tsize} bytes, {tdelta*1000:.3f} ms, '
                  f'{int(8*tsize/tdelta):d} bps')
            print(f'RX: {rsize} bytes, {rdelta*1000:.3f} ms, '
                  f'{int(8*rsize/rdelta):d} bps')
        self.assertTrue(rsize >= tsize-4*chunk, 'Data loss')

    @skipIf(IFCOUNT != 1, 'Test reserved for single-port FTDI device')
    def test_loopback_talk_speed(self):
        port = serial_for_url(URL, baudrate=6000000)
        size = int(1e6)
        results = [None, None]
        chunk = 537
        source = Thread(target=self._stream_source,
                        args=(port, chunk, size, results),
                        daemon=True)
        sink = Thread(target=self._stream_sink,
                      args=(port, size, results),
                      daemon=True)
        sink.start()
        source.start()
        source.join()
        sleep(0.5)
        sink.join()
        if isinstance(results[1], Exception):
            #pylint: disable-msg=raising-bad-type
            raise results[1]
        #pylint: disable-msg=unpacking-non-sequence
        tsize, tdelta = results[0]
        rsize, rdelta = results[1]
        self.assertGreater(rsize, 0, 'Not data received')
        if self.debug:
            print(f'TX: {tsize} bytes, {tdelta*1000:.3f} ms, '
                  f'{int(8*tsize/tdelta):d} bps')
            print(f'RX: {rsize} bytes, {rdelta*1000:.3f} ms, '
                  f'{int(8*rsize/rdelta):d} bps')
        self.assertTrue(rsize >= tsize-4*chunk, 'Data loss')

    @classmethod
    def _stream_source(cls, port, chunk, size, results):
        pos = 0
        tx_size = 0
        start = now()
        while tx_size < size:
            samples = spack('>%dI' % chunk, *range(pos, pos+chunk))
            pos += chunk
            port.write(samples)
            tx_size += len(samples)
            if results[1] is not None:
                break
        delta = now()-start
        results[0] = tx_size, delta

    @classmethod
    def _stream_sink(cls, port, size, results):
        pos = 0
        first = None
        data = bytearray()
        sample_size = scalc('>I')
        rx_size = 0
        port.timeout = 1.0
        start = now()
        while rx_size < size:
            buf = port.read(1024)
            if not buf:
                print('T')
                break
            rx_size += len(buf)
            data.extend(buf)
            sample_count = len(data)//sample_size
            length = sample_count*sample_size
            samples = sunpack('>%dI' % sample_count, data[:length])
            data = data[length:]
            for sample in samples:
                if first is None:
                    first = sample
                    pos = sample
                    continue
                pos += 1
                if sample != pos:
                    results[1] = AssertionError('Lost byte @ %d', pos-first)
                    return
        delta = now()-start
        results[1] = (rx_size, delta)

    @classmethod
    def _cross_talk_write_then_read(cls, url, refstream):
        # use classmethod & direct AssertionError to avoid pickler issues
        # with multiprocessing:
        # "TypeError: cannot serialize '_io.TextIOWrapper' object"
        port = serial_for_url(url, baudrate=1000000)
        try:
            if not port.is_open:
                port.open()
            port.timeout = 5.0
            port.write(refstream)
            instream = port.read(len(refstream))
            if len(instream) != len(refstream):
                raise AssertionError('Stream length differ')
            # we expect the peer to return the same stream, inverted
            localstream = bytes(reversed(instream))
            if localstream != refstream:
                raise AssertionError('Stream content differ')
        finally:
            port.close()

    @classmethod
    def _cross_talk_read_then_write(cls, url, refstream):
        # use classmethod & direct AssertionError to avoid pickler issues
        # with multiprocessing:
        # "TypeError: cannot serialize '_io.TextIOWrapper' object"
        port = serial_for_url(url, baudrate=1000000)
        try:
            if not port.is_open:
                port.open()
            port.timeout = 5.0
            instream = port.read(len(refstream))
            if len(instream) != len(refstream):
                raise AssertionError('Stream length differ')
            if instream != refstream:
                raise AssertionError('Stream content differ')
            # the peer expect us to return the same stream, inverted
            outstream = bytes(reversed(instream))
            port.write(outstream)
        finally:
            port.close()

    @classmethod
    def generate_bytes(cls, count=0):
        return ''.join([choice(printable)
                        for x in range(count or cls.COUNT)]).encode()

    @classmethod
    def build_next_url(cls, url):
        iface = int(url[-1])
        iface = (iface + 1) % 3
        return '%s%d' % (url[:-1], iface)


class BaudrateTestCase(FtdiTestCase):
    """Simple test to check clock stretching cannot be overwritten with
       GPIOs.
    """

    BAUDRATES = (300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
                 230400, 460800, 576000, 921600, 1000000, 1500000, 1843200,
                 2000000, 2500000, 3000000, 4000000, 6000000, 8000000,
                 12000000)

    def test(self):
        ftdi = Ftdi()
        ftdi.open_from_url(URL)
        for baudrate in self.BAUDRATES:
            actual, _, _ = ftdi._convert_baudrate(baudrate)
            ratio = baudrate/actual
            self.assertTrue(0.97 <= ratio <= 1.03, "Invalid baudrate")


def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(BaudrateTestCase, 'test'))
    suite_.addTest(makeSuite(UartTestCase, 'test'))
    return suite_


def main():
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError as exc:
        raise ValueError('Invalid log level: %s' % level) from exc
    FtdiLogger.set_level(loglevel)
    ut_main(defaultTest='suite')

if __name__ == '__main__':
    if platform == 'darwin':
        # avoid the infamous "The process has forked and you cannot use this
        # CoreFoundation functionality safely. You MUST exec()." error on macOS
        set_start_method('spawn')
    main()
