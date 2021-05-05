#!/usr/bin/env python3

"""Simple Python serial terminal
"""

# Copyright (c) 2010-2020, Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2016, Emmanuel Bouaziz <ebouaziz@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#pylint: disable-msg=too-many-instance-attributes
#pylint: disable-msg=too-many-arguments
#pylint: disable-msg=too-many-nested-blocks
#pylint: disable-msg=too-many-branches
#pylint: disable-msg=too-many-statements
#pylint: disable-msg=too-few-public-methods
#pylint: disable-msg=broad-except
#pylint: disable-msg=wrong-import-position

from argparse import ArgumentParser, FileType
from atexit import register
from collections import deque
from logging import Formatter, StreamHandler, DEBUG, ERROR
from os import environ, linesep, stat
from re import search
from sys import exit as sysexit, modules, platform, stderr, stdout
from time import sleep
from threading import Event, Thread
from traceback import format_exc
from _thread import interrupt_main

#pylint: disable-msg=import-error
#pylint: disable-msg=import-outside-toplevel

from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.misc import to_bps, add_custom_devices
from pyftdi.term import Terminal


class MiniTerm:
    """A mini serial terminal to demonstrate pyserial extensions"""

    DEFAULT_BAUDRATE = 115200

    def __init__(self, device, baudrate=None, parity=None, rtscts=False,
                 debug=False):
        self._terminal = Terminal()
        self._device = device
        self._baudrate = baudrate or self.DEFAULT_BAUDRATE
        self._port = self._open_port(self._device, self._baudrate, parity,
                                     rtscts, debug)
        self._resume = False
        self._silent = False
        self._rxq = deque()
        self._rxe = Event()
        self._debug = debug
        register(self._cleanup)

    def run(self, fullmode=False, loopback=False, silent=False,
            localecho=False, autocr=False):
        """Switch to a pure serial terminal application"""

        self._terminal.init(fullmode)
        print('Entering minicom mode @ %d bps' % self._port.baudrate)
        stdout.flush()
        self._resume = True
        # start the reader (target to host direction) within a dedicated thread
        args = [loopback]
        if self._device.startswith('ftdi://'):
            # with pyftdi/pyusb/libusb stack, there is no kernel buffering
            # which means that a UART source with data burst may overflow the
            # FTDI HW buffer while the SW stack is dealing with formatting
            # and console output. Use an intermediate thread to pop out data
            # out from the HW as soon as it is made available, and use a deque
            # to serve the actual reader thread
            args.append(self._get_from_source)
            sourcer = Thread(target=self._sourcer)
            sourcer.setDaemon(1)
            sourcer.start()
        else:
            # regular kernel buffered device
            args.append(self._get_from_port)
        reader = Thread(target=self._reader, args=tuple(args))
        reader.setDaemon(1)
        reader.start()
        # start the writer (host to target direction)
        self._writer(fullmode, silent, localecho, autocr)

    def _sourcer(self):
        try:
            while self._resume:
                data = self._port.read(4096)
                if not data:
                    continue
                self._rxq.append(data)
                self._rxe.set()
        except Exception as ex:
            self._resume = False
            print(str(ex), file=stderr)
            interrupt_main()

    def _get_from_source(self):
        while not self._rxq and self._resume:
            if self._rxe.wait(0.1):
                self._rxe.clear()
                break
        if not self._rxq:
            return bytearray()
        return self._rxq.popleft()

    def _get_from_port(self):
        try:
            return self._port.read(4096)
        except OSError as ex:
            self._resume = False
            print(str(ex), file=stderr)
            interrupt_main()
        except Exception as ex:
            print(str(ex), file=stderr)
            return bytearray()

    def _reader(self, loopback, getfunc):
        """Loop forever, processing received serial data in terminal mode"""
        try:
            # Try to read as many bytes as possible at once, and use a short
            # timeout to avoid blocking for more data
            self._port.timeout = 0.050
            while self._resume:
                if self._silent:
                    sleep(0.25)
                    continue
                data = getfunc()
                if data:
                    stdout.write(data.decode('utf8', errors='replace'))
                    stdout.flush()
                if loopback:
                    self._port.write(data)
        except KeyboardInterrupt:
            return
        except Exception as exc:
            print("Exception: %s" % exc)
            if self._debug:
                print(format_exc(chain=False), file=stderr)
            interrupt_main()

    def _writer(self, fullmode, silent, localecho, crlf=0):
        """Loop and copy console->serial until EOF character is found"""
        while self._resume:
            try:
                char = self._terminal.getkey()
                if fullmode and ord(char) == 0x2:    # Ctrl+B
                    self._cleanup(True)
                    return
                if self._terminal.IS_MSWIN:
                    if ord(char) in (0, 224):
                        char = self._terminal.getkey()
                        self._port.write(self._terminal.getch_to_escape(char))
                        continue
                    if ord(char) == 0x3:    # Ctrl+C
                        raise KeyboardInterrupt('Ctrl-C break')
                if silent:
                    if ord(char) == 0x6:    # Ctrl+F
                        self._silent = True
                        print('Silent\n')
                        continue
                    if ord(char) == 0x7:    # Ctrl+G
                        self._silent = False
                        print('Reg\n')
                        continue
                if localecho:
                    stdout.write(char.decode('utf8', errors='replace'))
                    stdout.flush()
                if crlf:
                    if char == b'\n':
                        self._port.write(b'\r')
                        if crlf > 1:
                            continue
                self._port.write(char)
            except KeyError:
                continue
            except KeyboardInterrupt:
                if fullmode:
                    if self._terminal.IS_MSWIN:
                        self._port.write(b'\x03')
                    continue
                self._cleanup(True)

    def _cleanup(self, *args):
        """Cleanup resource before exiting"""
        if args and args[0]:
            print('%sAborting...' % linesep)
        try:
            self._resume = False
            if self._port:
                # wait till the other thread completes
                sleep(0.5)
                try:
                    rem = self._port.inWaiting()
                except IOError:
                    # maybe a bug in underlying wrapper...
                    rem = 0
                # consumes all the received bytes
                for _ in range(rem):
                    self._port.read()
                self._port.close()
                self._port = None
                print('Bye.')
        except Exception as ex:
            print(str(ex), file=stderr)
        finally:
            if self._terminal:
                self._terminal.reset()
                self._terminal = None

    @staticmethod
    def _open_port(device, baudrate, parity, rtscts, debug=False):
        """Open the serial communication port"""
        try:
            from serial.serialutil import SerialException
            from serial import PARITY_NONE
        except ImportError as exc:
            raise ImportError("Python serial module not installed") from exc
        try:
            from serial import serial_for_url, VERSION as serialver
            # use a simple regex rather than adding a new dependency on the
            # more complete 'packaging' module
            vmo = search(r'^(\d+)\.(\d+)', serialver)
            if not vmo:
                # unable to parse version
                raise ValueError()
            if tuple([int(x) for x in vmo.groups()]) < (3, 0):
                # pysrial version is too old
                raise ValueError()
        except (ValueError, IndexError, ImportError) as exc:
            raise ImportError("pyserial 3.0+ is required") from exc
        # the following import enables serial protocol extensions
        if device.startswith('ftdi:'):
            try:
                from pyftdi import serialext
                serialext.touch()
            except ImportError as exc:
                raise ImportError("PyFTDI module not installed") from exc
        try:
            port = serial_for_url(device,
                                  baudrate=baudrate,
                                  parity=parity or PARITY_NONE,
                                  rtscts=rtscts,
                                  timeout=0)
            if not port.is_open:
                port.open()
            if not port.is_open:
                raise IOError('Cannot open port "%s"' % device)
            if debug:
                backend = port.BACKEND if hasattr(port, 'BACKEND') else '?'
                print("Using serial backend '%s'" % backend)
            return port
        except SerialException as exc:
            raise IOError(str(exc)) from exc


def get_default_device() -> str:
    """Return the default comm device, depending on the host/OS."""
    envdev = environ.get('FTDI_DEVICE', '')
    if envdev:
        return envdev
    if platform == 'win32':
        device = 'COM1'
    elif platform == 'darwin':
        device = '/dev/cu.usbserial'
    elif platform == 'linux':
        device = '/dev/ttyS0'
    else:
        device = ''
    try:
        stat(device)
    except OSError:
        device = 'ftdi:///1'
    return device



def main():
    """Main routine"""
    debug = False
    try:
        default_device = get_default_device()
        argparser = ArgumentParser(description=modules[__name__].__doc__)
        argparser.add_argument('-f', '--fullmode', dest='fullmode',
                                   action='store_true',
                                   help='use full terminal mode, exit with '
                                        '[Ctrl]+B')
        argparser.add_argument('device', nargs='?', default=default_device,
                               help='serial port device name (default: %s)' %
                               default_device)
        argparser.add_argument('-b', '--baudrate',
                               help='serial port baudrate (default: %d)' %
                               MiniTerm.DEFAULT_BAUDRATE,
                               default='%s' % MiniTerm.DEFAULT_BAUDRATE)
        argparser.add_argument('-w', '--hwflow',
                               action='store_true',
                               help='hardware flow control')
        argparser.add_argument('-e', '--localecho',
                               action='store_true',
                               help='local echo mode (print all typed chars)')
        argparser.add_argument('-r', '--crlf',
                               action='count', default=0,
                               help='prefix LF with CR char, use twice to '
                                    'replace all LF with CR chars')
        argparser.add_argument('-l', '--loopback',
                               action='store_true',
                               help='loopback mode (send back all received '
                                    'chars)')
        argparser.add_argument('-s', '--silent', action='store_true',
                               help='silent mode')
        argparser.add_argument('-P', '--vidpid', action='append',
                               help='specify a custom VID:PID device ID, '
                                    'may be repeated')
        argparser.add_argument('-V', '--virtual', type=FileType('r'),
                               help='use a virtual device, specified as YaML')
        argparser.add_argument('-v', '--verbose', action='count',
                               help='increase verbosity')
        argparser.add_argument('-d', '--debug', action='store_true',
                               help='enable debug mode')
        args = argparser.parse_args()
        debug = args.debug

        if not args.device:
            argparser.error('Serial device not specified')

        loglevel = max(DEBUG, ERROR - (10 * (args.verbose or 0)))
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

        miniterm = MiniTerm(device=args.device,
                            baudrate=to_bps(args.baudrate),
                            parity='N',
                            rtscts=args.hwflow,
                            debug=args.debug)
        miniterm.run(args.fullmode, args.loopback, args.silent, args.localecho,
                     args.crlf)

    except (IOError, ValueError) as exc:
        print('\nError: %s' % exc, file=stderr)
        if debug:
            print(format_exc(chain=False), file=stderr)
        sysexit(1)
    except KeyboardInterrupt:
        sysexit(2)


if __name__ == '__main__':
    main()
