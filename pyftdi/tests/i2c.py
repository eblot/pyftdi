#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2017-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import logging
from unittest import TestCase, TestSuite, main as ut_main, makeSuite
from binascii import hexlify
from doctest import testmod
from os import environ
from sys import modules, stdout
from pyftdi import FtdiLogger
from pyftdi.i2c import I2cController, I2cIOError
from pyftdi.misc import pretty_size

#pylint: disable-msg=attribute-defined-outside-init
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=no-self-use


class I2cTca9555TestCase(TestCase):
    """Simple test for a TCA9555 device on I2C bus @ address 0x21
    """

    def test(self):
        self._i2c = I2cController()
        self._open()
        self._read_it()
        self._write_it()
        self._close()

    def _open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        self._i2c.configure(url)

    def _read_it(self):
        port = self._i2c.get_port(0x21)
        port.exchange([0x04], 1)

    def _write_it(self):
        port = self._i2c.get_port(0x21)
        port.write_to(0x06, b'\x00')
        port.write_to(0x02, b'\x55')
        port.read_from(0x00, 1)

    def _close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


class I2cAccelTest(TestCase):
    """Basic test for an ADXL345 device on I2C bus @ address 0x53
    """

    def test(self):
        self._i2c = I2cController()
        self._open()
        self._read_device_id()
        self._close()

    def _open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        self._i2c.configure(url)

    def _read_device_id(self):
        port = self._i2c.get_port(0x53)
        device_id = port.exchange([0x00], 1)
        hex_device_id = hexlify(device_id).decode()
        print('DEVICE ID:', hex_device_id)
        return hex_device_id

    def _close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


class I2cReadTest(TestCase):
    """Simple test to read a sequence of bytes I2C bus @ address 0x36
    """

    def test(self):
        self._i2c = I2cController()
        self._open()
        self._read()
        self._close()

    def _open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        self._i2c.configure(url)

    def _read(self):
        address = environ.get('I2C_ADDRESS', '0x36').lower()
        addr = int(address, 16 if address.startswith('0x') else 10)
        port = self._i2c.get_port(addr)
        data = port.read(32)
        print(hexlify(data).decode(), data.decode('utf8', errors='replace'))

    def _close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


class I2cEepromTest(TestCase):
    """Simple test to read a sequence of bytes I2C bus @ address 0x50,
       from an I2C data flash
    """

    @classmethod
    def setUpClass(cls):
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        address = environ.get('I2C_ADDRESS', '0x50').lower()
        cls.address = int(address, 16 if address.startswith('0x') else 10)

    def setUp(self):
        self._i2c = I2cController()
        self._i2c.configure(self.url, frequency=400e3,
                            clockstretching=False, debug=False, rdoptim=True)

    def tearDown(self):
        self._i2c.terminate()

    def test_short(self):
        port = self._i2c.get_port(self.address)
        # select start address
        port.write(b'\x00\x08')
        data = port.read(4)
        text = data.decode('utf8', errors='replace')
        # print(hexlify(data).decode(), text)
        self.assertEqual(text, 'Worl')

    def test_long(self):
        port = self._i2c.get_port(self.address)
        # select start address
        #print('RC', self._i2c.ftdi.read_data_get_chunksize())
        #print('WC', self._i2c.ftdi.write_data_get_chunksize())
        from time import time as now
        size = 4096
        port.write(b'\x00\x00')
        start = now()
        data = port.read(size)
        stop = now()
        text = data.decode('utf8', errors='replace')
        delta = stop-start
        byterate = pretty_size(len(data)/delta)
        print(f'Exec time: {1000*delta:.3f} ms, {byterate}/s')
        self.assertEqual(text[8:12], 'Worl')


class I2cReadGpioTest(TestCase):
    """Simple test to exercise I2C + GPIO mode.

       A slave device (such as EEPROM) should be connected to the I2C bus
       at either the default 0x36 address or defined with the I2C_ADDRESS
       ebvironment variable.

       AD0: SCL, AD1+AD2: SDA, AD3 connected to AC0, AD4 connected to AC1

       I2C EEPROM is read, and values are written to AD3:AD4 and read back
       from AC0:AC1.
    """

    GPIO_WIDTH = 2  # use 2 GPIOs for output, 2 GPIOs for input (loopback)
    GPIO_OUT_OFFSET = 3  # GPIO output are b3..b4
    GPIO_IN_OFFSET = 8  # GPIO input are b8..b9

    def test(self):
        self._i2c = I2cController()
        self._open()
        self._execute_sequence()
        self._execute_interleave()
        self._close()

    def _open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        self._i2c.configure(url)
        address = environ.get('I2C_ADDRESS', '0x36').lower()
        addr = int(address, 16 if address.startswith('0x') else 10)
        self._port = self._i2c.get_port(addr)
        self._gpio = self._i2c.get_gpio()
        mask = (1 << self.GPIO_WIDTH) - 1
        self._gpio.set_direction(mask << self.GPIO_OUT_OFFSET |
                                 mask << self.GPIO_IN_OFFSET,
                                 mask << self.GPIO_OUT_OFFSET)

    def _execute_sequence(self):
        # reset EEPROM read pointer position
        self._port.write(b'\x00\x00')
        ref = self._port.read(32)
        for dout in range(1 << self.GPIO_WIDTH):
            self._gpio.write(dout << self.GPIO_OUT_OFFSET)
            din = self._gpio.read() >> self.GPIO_IN_OFFSET
            if dout != din:
                raise AssertionError(f'GPIO mismatch {din:04x} != {dout:04x}')
        self._gpio.write(0)
        # reset EEPROM read pointer position
        self._port.write(b'\x00\x00')
        data = self._port.read(32)
        if data != ref:
            raise AssertionError("I2C data mismatch")

    def _execute_interleave(self):
        # reset EEPROM read pointer position
        self._port.write(b'\x00\x00')
        ref = self._port.read(32)
        for dout in range(1 << self.GPIO_WIDTH):
            self._gpio.write(dout << self.GPIO_OUT_OFFSET)
            # reset EEPROM read pointer position
            self._port.write(b'\x00\x00')
            data = self._port.read(32)
            din = self._gpio.read() >> self.GPIO_IN_OFFSET
            if data != ref:
                raise AssertionError("I2C data mismatch")
            if dout != din:
                raise AssertionError(f'GPIO mismatch {din:04x} != {dout:04x}')
        self._gpio.write(0)

    def _close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


class I2cClockStrechingGpioCheck(TestCase):
    """Simple test to check clock stretching cannot be overwritten with
       GPIOs.
    """

    def test(self):
        self._i2c = I2cController()
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        self._i2c.configure(url, clockstretching=True)
        gpio = self._i2c.get_gpio()
        self.assertRaises(I2cIOError, gpio.set_direction, 1 << 7, 0)


class I2cDualMaster(TestCase):
    """Check the behaviour of 2 I2C masters. Requires a multi port FTDI device,
       i.e. FT2232H or FT4232H. See issue #159.
    """

    def test(self):
        url1 = environ.get('FTDI_DEVICE', 'ftdi:///1')
        i2c1 = I2cController()
        i2c1.configure(url1, frequency=100000)
        url2 = '%s%d' % (url1[:-1], int(url1[-1])+1)
        i2c2 = I2cController()
        i2c2.configure(url2, frequency=100000)
        port = i2c2.get_port(0x76)
        print(port.read_from(0x00, 2))
        print(port.read_from(0x00, 2))


class I2cIssue143(TestCase):
    """#143.
    """

    def test(self):
        url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        address = environ.get('I2C_ADDRESS', '0x50').lower()
        addr = int(address, 16 if address.startswith('0x') else 10)
        i2c = I2cController()
        i2c.configure(url)
        slave = i2c.get_port(addr)
        gpio = i2c.get_gpio()
        gpio.set_direction(0x0010, 0x0010)
        gpio.write(0)
        gpio.write(1<<4)
        gpio.write(0)
        slave.write([0x12, 0x34])
        gpio.write(0)
        gpio.write(1<<4)
        gpio.write(0)


def suite():
    """FTDI I2C driver test suite

       Simple test to demonstrate I2C.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. GPIOs can be driven high or low, so check
       your HW setup before running this test as it might damage your HW.

       Do NOT run this test if you use FTDI port A as an UART or SPI
       bridge -or any unsupported setup!! You've been warned.
    """
    ste = TestSuite()
    #ste.addTest(I2cTca9555TestCase('test'))
    #ste.addTest(I2cAccelTest('test'))
    #ste.addTest(I2cReadTest('test'))
    ste.addTest(makeSuite(I2cEepromTest, 'test'))
    #ste.addTest(I2cReadGpioTest('test'))
    ste.addTest(I2cClockStrechingGpioCheck('test'))
    #ste.addTest(I2cDualMaster('test'))
    ste.addTest(I2cIssue143('test'))
    return ste


def main():
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError as exc:
        raise ValueError(f'Invalid log level: {level}') from exc
    FtdiLogger.set_level(loglevel)
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
