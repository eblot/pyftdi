#!/usr/bin/env python3

"""Simple FTDI EEPROM configurator.
"""

# Copyright (c) 2019, Emmanuel Blot <emmanuel.blot@free.fr>
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

from argparse import ArgumentParser, FileType
from logging import Formatter, StreamHandler, DEBUG, ERROR
from sys import modules, stderr
from traceback import format_exc
from pyftdi import FtdiLogger
from pyftdi.eeprom import FtdiEeprom
from pyftdi.misc import hexdump

def main():
    """Main routine"""
    debug = False
    try:
        argparser = ArgumentParser(description=modules[__name__].__doc__)
        argparser.add_argument('device', nargs='?', default='ftdi:///?',
                               help='serial port device name')
        argparser.add_argument('-x', '--hexdump', action='store_true',
                               help='dump EEPROM content as ASCII')
        argparser.add_argument('-o', '--output', type=FileType('wt'),
                               help='output ini file to save EEPROM content')
        argparser.add_argument('-s', '--serial-number',
                               help='set serial number')
        argparser.add_argument('-m', '--manufacturer',
                               help='set manufacturer name')
        argparser.add_argument('-p', '--product',
                               help='set product name')
        argparser.add_argument('-e', '--erase', action='store_true',
                               help='erase the whole EEPROM content')
        argparser.add_argument('-u', '--update', action='store_true',
                               help='perform actual update, use w/ care')
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

        eeprom = FtdiEeprom()
        eeprom.open(args.device)
        if args.erase:
            eeprom.erase()
        if args.serial_number:
            eeprom.set_serial_number(args.serial_number)
        if args.manufacturer:
            eeprom.set_manufacturer_name(args.manufacturer)
        if args.product:
            eeprom.set_product_name(args.product)
        if args.hexdump:
            print(hexdump(eeprom.data))
        if args.update:
            eeprom.commit(False)
        if args.verbose > 0:
            eeprom.dump_config()
        if args.output:
            eeprom.save_config(args.output)

    except (IOError, ValueError) as exc:
        print('\nError: %s' % exc, file=stderr)
        if debug:
            print(format_exc(chain=False), file=stderr)
        exit(1)
    except KeyboardInterrupt:
        exit(2)


if __name__ == '__main__':
    main()
