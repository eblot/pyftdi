#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2017-2019, Emmanuel Blot <emmanuel.blot@free.fr>
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
from unittest import TestCase, TestSuite, makeSuite, main as testmain
from binascii import hexlify
from doctest import testmod
from os import environ
from sys import modules, stdout
from pyftdi import FtdiLogger
from pyftdi.i2c import I2cController, I2cIOError

#pylint: disable-msg=attribute-defined-outside-init
#pylint: disable-msg=missing-docstring


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
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
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
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._i2c.configure(url)

    def _read_device_id(self):
        port = self._i2c.get_port(0x53)
        device_id = port.exchange([0x00], 1).tobytes()
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

    def ___init__(self):
        self._i2c = I2cController()

    def _open(self):
        """Open an I2c connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._i2c.configure(url)

    def _read(self):
        address = environ.get('I2C_ADDRESS', '0x36').lower()
        addr = int(address, 16 if address.startswith('0x') else 10)
        port = self._i2c.get_port(addr)
        data = port.read(32).tobytes()
        print(hexlify(data).decode(), data.decode('utf8', errors='replace'))

    def _close(self):
        """Close the I2C connection"""
        self._i2c.terminate()


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
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
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
        ref = self._port.read(32).tobytes()
        for dout in range(1 << self.GPIO_WIDTH):
            self._gpio.write(dout << self.GPIO_OUT_OFFSET)
            din = self._gpio.read() >> self.GPIO_IN_OFFSET
            if dout != din:
                raise AssertionError("GPIO mismatch 0x%04x != 0x%04x",
                                     din, dout)
        self._gpio.write(0)
        # reset EEPROM read pointer position
        self._port.write(b'\x00\x00')
        data = self._port.read(32).tobytes()
        if data != ref:
            raise AssertionError("I2C data mismatch")

    def _execute_interleave(self):
        # reset EEPROM read pointer position
        self._port.write(b'\x00\x00')
        ref = self._port.read(32).tobytes()
        for dout in range(1 << self.GPIO_WIDTH):
            self._gpio.write(dout << self.GPIO_OUT_OFFSET)
            # reset EEPROM read pointer position
            self._port.write(b'\x00\x00')
            data = self._port.read(32).tobytes()
            din = self._gpio.read() >> self.GPIO_IN_OFFSET
            if data != ref:
                raise AssertionError("I2C data mismatch")
            if dout != din:
                raise AssertionError("GPIO mismatch 0x%04x != 0x%04x",
                                     din, dout)
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
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        self._i2c.configure(url, clockstretching=True)
        gpio = self._i2c.get_gpio()
        self.assertRaises(I2cIOError, gpio.set_direction, 1 << 7, 0)


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
    ste.addTest(I2cTca9555TestCase('test'))
    ste.addTest(I2cAccelTest('test'))
    ste.addTest(I2cReadTest('test'))
    ste.addTest(I2cReadGpioTest('test'))
    ste.addTest(I2cClockStrechingGpioCheck('test'))
    return ste


def main():
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError('Invalid log level: %s' %level)
    FtdiLogger.set_level(loglevel)
    testmain(defaultTest='suite')


if __name__ == '__main__':
    main()
