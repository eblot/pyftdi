#!/usr/bin/env python3

"""Simple Python serial terminal
"""

# Copyright (c) 2010-2017, Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2016, Emmanuel Bouaziz <ebouaziz@free.fr>
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

from _thread import interrupt_main
from argparse import ArgumentParser
from array import array
from atexit import register
from collections import deque
from logging import Formatter, DEBUG, ERROR
from os import linesep, name as osname, stat, uname
from pyftdi import FtdiLogger
from pyftdi.misc import to_int
from sys import modules, platform, stderr, stdin, stdout
from term import getkey
from time import sleep
from threading import Event, Thread
from traceback import format_exc
mswin = platform == 'win32'
if not mswin:
    from termios import TCSANOW, tcgetattr, tcsetattr


class MiniTerm:
    """A mini serial terminal to demonstrate pyserial extensions"""

    DEFAULT_BAUDRATE = 115200

    def __init__(self, device, baudrate=None, parity=None, rtscts=False,
                 debug=False):
        self._termstates = []
        if not mswin and stdout.isatty():
            self._termstates = [(fd, tcgetattr(fd)) for fd in
                                (stdin.fileno(), stdout.fileno(),
                                 stderr.fileno())]
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

        # wait forever, although Windows is stupid and does not signal Ctrl+C,
        # so wait use a 1/2-second timeout that gives some time to check for a
        # Ctrl+C break then polls again...
        print('Entering minicom mode')
        stdout.flush()
        self._port.timeout = 0.5
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
            return array('B')
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
            return array('B')

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
        except Exception as e:
            print("Exception: %s" % e)
            if self._debug:
                print(format_exc(chain=False), file=stderr)
            interrupt_main()

    def _writer(self, fullmode, silent, localecho, crlf=0):
        """Loop and copy console->serial until EOF character is found"""
        while self._resume:
            try:
                c = getkey(fullmode)
                if mswin:
                    if ord(c) == 0x3:
                        raise KeyboardInterrupt()
                if fullmode and ord(c) == 0x2:  # Ctrl+B
                    self._cleanup()
                    return
                if silent:
                    if ord(c) == 0x6:  # Ctrl+F
                        self._silent = True
                        print('Silent\n')
                        continue
                    if ord(c) == 0x7:  # Ctrl+G
                        self._silent = False
                        print('Reg\n')
                        continue
                else:
                    if localecho:
                        stdout.write(c.decode('utf8', errors='replace'))
                        stdout.flush()
                    if crlf:
                        if c == b'\n':
                            self._port.write(b'\r')
                            if crlf > 1:
                                continue
                    self._port.write(c)
            except KeyboardInterrupt:
                if fullmode:
                    continue
                print('%sAborting...' % linesep)
                self._cleanup()
                return

    def _cleanup(self):
        """Cleanup resource before exiting"""
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
            for fd, att in self._termstates:
                tcsetattr(fd, TCSANOW, att)
        except Exception as ex:
            print(str(ex), file=stderr)

    @staticmethod
    def _open_port(device, baudrate, parity, rtscts, debug=False):
        """Open the serial communication port"""
        try:
            from serial.serialutil import SerialException
            from serial import PARITY_NONE
        except ImportError:
            raise ImportError("Python serial module not installed")
        try:
            from serial import serial_for_url, VERSION as serialver
            version = tuple([int(x) for x in serialver.split('.')])
            if version < (2, 6):
                raise ValueError
        except (ValueError, IndexError, ImportError):
            raise ImportError("pyserial 2.6+ is required")
        # the following import enables serial protocol extensions
        if device.startswith('ftdi:'):
            try:
                from pyftdi import serialext
                serialext.touch()
            except ImportError:
                raise ImportError("PyFTDI module not installed")
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
                backend = hasattr(port, 'BACKEND') and port.BACKEND or '?'
                print("Using serial backend '%s'" % backend)
            return port
        except SerialException as e:
            raise IOError(str(e))


def get_default_device():
    if osname == 'nt':
        device = 'COM1'
    elif osname == 'posix':
        (system, _, _, _, _) = uname()
        if system.lower() == 'darwin':
            device = '/dev/cu.usbserial'
        else:
            device = '/dev/ttyS0'
        try:
            stat(device)
        except OSError:
            device = 'ftdi:///1'
    else:
        device = None
    return device


def main():
    """Main routine"""
    debug = False
    try:
        default_device = get_default_device()
        argparser = ArgumentParser(description=modules[__name__].__doc__)
        if osname in ('posix', ):
            argparser.add_argument('-f', '--fullmode', dest='fullmode',
                                   action='store_true',
                                   help='use full terminal mode, exit with '
                                        '[Ctrl]+B')
        argparser.add_argument('-p', '--device', default=default_device,
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

        miniterm = MiniTerm(device=args.device,
                            baudrate=to_int(args.baudrate),
                            parity='N',
                            rtscts=args.hwflow,
                            debug=args.debug)
        miniterm.run(args.fullmode, args.loopback, args.silent, args.localecho,
                     args.crlf)

    except (IOError, ValueError) as e:
        print('\nError: %s' % e, file=stderr)
        if debug:
            print(format_exc(chain=False), file=stderr)
        exit(1)
    except KeyboardInterrupt:
        exit(2)


if __name__ == '__main__':
    main()
