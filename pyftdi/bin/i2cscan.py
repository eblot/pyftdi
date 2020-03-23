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

from argparse import ArgumentParser, FileType
from logging import Formatter, StreamHandler, getLogger, DEBUG, ERROR
from os import environ
from sys import modules, stderr
from traceback import format_exc
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.i2c import I2cController, I2cNackError
from pyftdi.misc import add_custom_devices


class I2cBusScanner:
    """Scan I2C bus to find slave.

       Emit the I2C address message, but no data. Detect any ACK on each valid
       address.
    """

    @staticmethod
    def scan(url):
        """Open an I2c connection to a slave."""
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
    """Entry point."""
    debug = False
    try:
        argparser = ArgumentParser(description=modules[__name__].__doc__)
        argparser.add_argument('device', nargs='?', default='ftdi:///?',
                               help='serial port device name')
        argparser.add_argument('-P', '--vidpid', action='append',
                               help='specify a custom VID:PID device ID, '
                                    'may be repeated')
        argparser.add_argument('-V', '--virtual', type=FileType('r'),
                               help='use a virtual device, specified as YaML')
        argparser.add_argument('-v', '--verbose', action='count', default=0,
                               help='increase verbosity')
        argparser.add_argument('-d', '--debug', action='store_true',
                               help='enable debug mode')
        args = argparser.parse_args()
        debug = args.debug

        if not args.device:
            argparser.error('Serial device not specified')

        loglevel = max(DEBUG, ERROR - (10 * args.verbose))
        loglevel = min(ERROR, loglevel)
        if debug:
            formatter = Formatter('%(asctime)s.%(msecs)03d %(name)-20s '
                                  '%(message)s', '%H:%M:%S')
        else:
            formatter = Formatter('%(message)s')
        FtdiLogger.set_formatter(formatter)
        FtdiLogger.set_level(loglevel)
        FtdiLogger.log.addHandler(StreamHandler(stderr))

        if args.virtual:
            from pyftdi.usbtools import UsbTools
            # Force PyUSB to use PyFtdi test framework for USB backends
            UsbTools.BACKENDS = ('pyftdi.tests.backend.usbvirt', )
            # Ensure the virtual backend can be found and is loaded
            backend = UsbTools.find_backend()
            loader = backend.create_loader()()
            loader.load(args.virtual)

        try:
            add_custom_devices(Ftdi, args.vidpid)
        except ValueError as exc:
            argparser.error(str(exc))

        I2cBusScanner.scan(args.device)

    except (ImportError, IOError, NotImplementedError, ValueError) as exc:
        print('\nError: %s' % exc, file=stderr)
        if debug:
            print(format_exc(chain=False), file=stderr)
        exit(1)
    except KeyboardInterrupt:
        exit(2)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(str(exc), file=stderr)
