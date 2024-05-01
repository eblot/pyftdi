#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2024, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""JTAG IDCODE retriever."""

from argparse import ArgumentParser
from logging import getLogger
from os import linesep
from traceback import format_exc
import sys

from pyftdi.ftdi import Ftdi
from pyftdi.jtag import JtagFtdiController
from pyftdi.tools.bits import BitSequence
from pyftdi.tools.jtag import JtagEngine
from pyftdi.log import configure_loggers
from pyftdi.misc import add_custom_devices


def idcode(engine: JtagEngine, _ir_length: int) -> int:
    """Retrieve ID code."""
    engine.controller.tap_reset()
    value = engine.exchange_dr(BitSequence.from_int(-1, 32))
    engine.go_idle()
    getLogger('pyftdi.jtag').info('IDCODE: %s', value)
    return int(value)


def main():
    """Entry point."""
    debug = True
    default_ir_length = 5
    try:
        argparser = ArgumentParser(
            description=sys.modules[__name__].__doc__.split('.')[0])
        argparser.add_argument('device', nargs='?', default='ftdi:///?',
                               help='FTDI device URL')
        argparser.add_argument('-P', '--vidpid', action='append',
                               help='specify a custom VID:PID device ID, '
                                    'may be repeated')
        argparser.add_argument('-l', '--ir-length', type=int,
                               default=default_ir_length,
                               help=f'bit length of the IR register '
                                    f'(default: {default_ir_length})')
        argparser.add_argument('-v', '--verbose', action='count',
                               help='increase verbosity')
        argparser.add_argument('-d', '--debug', action='store_true',
                               help='enable debug mode')

        args = argparser.parse_args()
        debug = args.debug

        configure_loggers(args.verbose, 'pyftdi.jtag', -1, 'jtag', 'pyftdi')

        try:
            add_custom_devices(Ftdi, args.vidpid, force_hex=True)
        except ValueError as exc:
            argparser.error(str(exc))

        ctrl = JtagFtdiController()
        ctrl.configure(args.device, frequency=100000, direction=0x60eb,
                       initial=0x00e8)
        eng = JtagEngine(ctrl)
        code = idcode(eng, args.ir_length)
        print(f'IDCODE:    0x{code:x}')

    # pylint: disable=broad-except
    except Exception as exc:
        print(f'{linesep}Error: {exc}', file=sys.stderr)
        if debug:
            print(format_exc(chain=False), file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(2)


if __name__ == '__main__':
    main()
