#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2018-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Tiny I2C bus scanner."""

#pylint: disable-msg=broad-except
#pylint: disable-msg=too-few-public-methods

from argparse import ArgumentParser, FileType
from logging import Formatter, StreamHandler, getLogger, DEBUG, ERROR
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

    SMB_READ_RANGE = list(range(0x30, 0x38)) + list(range(0x50, 0x60))

    HIGHEST_I2C_SLAVE_ADDRESS = 0x78

    @classmethod
    def scan(cls, url: str, smb_mode: bool = True) -> None:
        """Scan an I2C bus to detect slave device.

           :param url: FTDI URL
           :param smb_mode: whether to use SMBbus restrictions or regular I2C
                            mode.
        """
        i2c = I2cController()
        slaves = []
        getLogger('pyftdi.i2c').setLevel(ERROR)
        try:
            i2c.set_retry_count(1)
            i2c.configure(url)
            for addr in range(cls.HIGHEST_I2C_SLAVE_ADDRESS+1):
                port = i2c.get_port(addr)
                if smb_mode:
                    try:
                        if addr in cls.SMB_READ_RANGE:
                            port.read(0)
                            slaves.append('R')
                        else:
                            port.write([])
                            slaves.append('W')
                    except I2cNackError:
                        slaves.append('.')
                else:
                    try:
                        port.read(0)
                        slaves.append('R')
                        continue
                    except I2cNackError:
                        pass
                    try:
                        port.write([])
                        slaves.append('W')
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
        argparser.add_argument('-S', '--no-smb', action='store_true',
                               default=False,
                               help='use regular I2C mode vs. SMBbus scan')
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
        FtdiLogger.log.addHandler(StreamHandler(stderr))
        FtdiLogger.set_formatter(formatter)
        FtdiLogger.set_level(loglevel)

        if args.virtual:
            #pylint: disable-msg=import-outside-toplevel
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

        I2cBusScanner.scan(args.device, not args.no_smb)

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
