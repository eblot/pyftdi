#!/usr/bin/env python3

"""Simple FTDI EEPROM configurator.
"""

# Copyright (c) 2019-2022, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from argparse import ArgumentParser, FileType
from io import StringIO
from logging import Formatter, StreamHandler, DEBUG, ERROR
from sys import modules, stderr, stdout
from textwrap import fill
from traceback import format_exc
from pyftdi import FtdiLogger
from pyftdi.eeprom import FtdiEeprom
from pyftdi.ftdi import Ftdi
from pyftdi.misc import add_custom_devices, hexdump

#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-many-branches
#pylint: disable-msg=too-many-statements


def main():
    """Main routine"""
    debug = False
    try:
        argparser = ArgumentParser(description=modules[__name__].__doc__)
        argparser.add_argument('device', nargs='?', default='ftdi:///?',
                               help='serial port device name')

        files = argparser.add_argument_group(title='Files')
        files.add_argument('-i', '--input', type=FileType('rt'),
                           help='input ini file to load EEPROM content')
        files.add_argument('-l', '--load', default='all',
                           choices=('all', 'raw', 'values'),
                           help='section(s) to load from input file')
        files.add_argument('-o', '--output',
                           help='output ini file to save EEPROM content')
        files.add_argument('-V', '--virtual', type=FileType('r'),
                           help='use a virtual device, specified as YaML')

        device = argparser.add_argument_group(title='Device')
        device.add_argument('-P', '--vidpid', action='append',
                            help='specify a custom VID:PID device ID '
                                 '(search for FTDI devices)')
        device.add_argument('-M', '--eeprom',
                            help='force an EEPROM model')
        device.add_argument('-S', '--size', type=int,
                            choices=FtdiEeprom.eeprom_sizes,
                            help='force an EEPROM size')

        fmt = argparser.add_argument_group(title='Format')
        fmt.add_argument('-x', '--hexdump', action='store_true',
                         help='dump EEPROM content as ASCII')
        fmt.add_argument('-X', '--hexblock', type=int,
                         help='dump EEPROM as indented hexa blocks')

        config = argparser.add_argument_group(title='Configuration')
        config.add_argument('-s', '--serial-number',
                            help='set serial number')
        config.add_argument('-m', '--manufacturer',
                            help='set manufacturer name')
        config.add_argument('-p', '--product',
                            help='set product name')
        config.add_argument('-c', '--config', action='append',
                            help='change/configure a property as key=value '
                                 'pair')
        config.add_argument('--vid', type=lambda x: int(x, 16),
                            help='shortcut to configure the USB vendor ID')
        config.add_argument('--pid', type=lambda x: int(x, 16),
                            help='shortcut to configure the USB product ID')

        action = argparser.add_argument_group(title='Action')
        action.add_argument('-e', '--erase', action='store_true',
                            help='erase the whole EEPROM content')
        action.add_argument('-E', '--full-erase', action='store_true',
                            default=False,
                            help='erase the whole EEPROM content, including '
                                 'the CRC')
        action.add_argument('-u', '--update', action='store_true',
                            help='perform actual update, use w/ care')

        extra = argparser.add_argument_group(title='Extras')
        extra.add_argument('-v', '--verbose', action='count', default=0,
                               help='increase verbosity')
        extra.add_argument('-d', '--debug', action='store_true',
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
            #pylint: disable-msg=import-outside-toplevel
            from pyftdi.usbtools import UsbTools
            # Force PyUSB to use PyFtdi test framework for USB backends
            UsbTools.BACKENDS = ('pyftdi.tests.backend.usbvirt', )
            # Ensure the virtual backend can be found and is loaded
            backend = UsbTools.find_backend()
            loader = backend.create_loader()()
            loader.load(args.virtual)

        try:
            add_custom_devices(Ftdi, args.vidpid, force_hex=True)
        except ValueError as exc:
            argparser.error(str(exc))

        eeprom = FtdiEeprom()
        eeprom.open(args.device, size=args.size, model=args.eeprom)
        if args.erase or args.full_erase:
            eeprom.erase()
        if args.input:
            eeprom.load_config(args.input, args.load)
        if args.serial_number:
            eeprom.set_serial_number(args.serial_number)
        if args.manufacturer:
            eeprom.set_manufacturer_name(args.manufacturer)
        if args.product:
            eeprom.set_product_name(args.product)
        for conf in args.config or []:
            if conf in ('?', 'help'):
                helpstr = ', '.join(sorted(eeprom.properties))
                print(fill(helpstr, initial_indent='  ',
                           subsequent_indent='  '))
                exit(1)
            for sep in ':=':
                if sep in conf:
                    name, value = conf.split(sep, 1)
                    if not value:
                        argparser.error('Configuration %s without value' %
                                        conf)
                    if value == 'help':
                        value = '?'
                    helpio = StringIO()
                    eeprom.set_property(name, value, helpio)
                    helpstr = helpio.getvalue()
                    if helpstr:
                        print(fill(helpstr, initial_indent='  ',
                                   subsequent_indent='  '))
                        exit(1)
                    break
            else:
                argparser.error('Missing name:value separator in %s' % conf)
        if args.vid:
            eeprom.set_property('vendor_id', args.vid)
        if args.pid:
            eeprom.set_property('product_id', args.pid)
        if args.hexdump:
            print(hexdump(eeprom.data))
        if args.hexblock is not None:
            indent = ' ' * args.hexblock
            for pos in range(0, len(eeprom.data), 16):
                hexa = ' '.join(['%02x' % x for x in eeprom.data[pos:pos+16]])
                print(indent, hexa, sep='')
        if args.update:
            if eeprom.commit(False, no_crc=args.full_erase):
                eeprom.reset_device()
        if args.verbose > 0:
            eeprom.dump_config()
        if args.output:
            if args.output == '-':
                eeprom.save_config(stdout)
            else:
                with open(args.output, 'wt') as ofp:
                    eeprom.save_config(ofp)

    except (ImportError, IOError, NotImplementedError, ValueError) as exc:
        print('\nError: %s' % exc, file=stderr)
        if debug:
            print(format_exc(chain=False), file=stderr)
        exit(1)
    except KeyboardInterrupt:
        exit(2)


if __name__ == '__main__':
    main()
