#!/usr/bin/env python3

# Copyright (c) 2019-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""List valid FTDI device URLs and descriptors."""

from argparse import ArgumentParser, FileType
from logging import Formatter, StreamHandler, DEBUG, ERROR
from sys import modules, stderr
from traceback import format_exc
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.misc import add_custom_devices


def main():
    """Entry point."""
    debug = False
    try:
        argparser = ArgumentParser(description=modules[__name__].__doc__)
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

        Ftdi.show_devices()

    except (ImportError, IOError, NotImplementedError, ValueError) as exc:
        print('\nError: %s' % exc, file=stderr)
        if debug:
            print(format_exc(chain=False), file=stderr)
        exit(1)
    except KeyboardInterrupt:
        exit(2)


if __name__ == '__main__':
    main()
