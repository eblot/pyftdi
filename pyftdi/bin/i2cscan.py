#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2018-2020, Emmanuel Blot <emmanuel.blot@free.fr>
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

"""Tiny I2C bus scanner."""

#pylint: disable-msg=broad-except
#pylint: disable-msg=too-few-public-methods

from logging import ERROR, getLogger
from os import environ
from sys import stderr
from pyftdi.i2c import I2cController, I2cNackError


class I2cBusScanner:
    """Scan I2C bus to find slave.

       Emit the I2C address message, but no data. Detect any ACK on each valid
       address.
    """

    @staticmethod
    def scan():
        """Open an I2c connection to a slave."""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')
        i2c = I2cController()
        slaves = []
        getLogger('pyftdi.i2c').setLevel(ERROR)
        try:
            i2c.set_retry_count(1)
            i2c.configure(url)
            for addr in range(i2c.HIGHEST_I2C_ADDRESS+1):
                port = i2c.get_port(addr)
                try:
                    port.read(0)
                    slaves.append('X')
                except I2cNackError:
                    slaves.append('.')
        finally:
            i2c.terminate()
        columns = 16
        row = 0
        print('   %s' % ''.join(' %01X ' % col for col in range(columns)))
        while True:
            chunk = slaves[row:row+columns]
            if not chunk:
                break
            print(' %1X:' % (row//columns), '  '.join(chunk))
            row += columns


def main():
    """Entry point"""
    I2cBusScanner.scan()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(str(exc), file=stderr)
