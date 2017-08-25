#!/usr/bin/env python3

"""Pure python simple serial terminal
"""

# Copyright (c) 2010-2016, Emmanuel Blot <emmanuel.blot@free.fr>
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

import os
import time
import threading
from _thread import interrupt_main
from argparse import ArgumentParser
from pyftdi import serialext
from pyftdi.misc import to_bool, to_int
from sys import modules, platform, stdin, stdout, stderr
from term import getkey
from traceback import format_exc
if platform != 'win32':
    import termios


class MiniTerm(object):
    """A mini serial terminal to demonstrate pyserial extensions"""

    def __init__(self, device, baudrate=115200, logfile=None, debug=False):
        self._termstates = []
        if platform != 'win32' and stdout.isatty():
            self._termstates = [(fd, termios.tcgetattr(fd)) for fd in
                                (stdin.fileno(), stdout.fileno(),
                                 stderr.fileno())]

        self._device = device
        self._baudrate = baudrate
        self._logfile = logfile
        self._port = self._open_port(self._device, self._baudrate,
                                     self._logfile, debug)
        self._resume = False
        self._debug = debug

    def __del__(self):
        try:
            self._cleanup()
        except Exception:
            pass

    def run(self, fullmode=False, reset=None, select=None):
        """Switch to a pure serial terminal application"""
        if select is not None:
            selmode = to_bool(select)
            self._port.setRTS(selmode)
        if reset is not None:
            hwreset = to_bool(reset)
            self._port.setDTR(hwreset)
            time.sleep(0.200)
            self._port.setDTR(not hwreset)
            time.sleep(0.100)
        # wait forever, although Windows is stupid and does not signal Ctrl+C,
        # so wait use a 1/2-second timeout that gives some time to check for a
        # Ctrl+C break then polls again...
        print('Entering minicom mode')
        stdout.flush()
        self._port.timeout = 0.5
        self._resume = True
        # start the reader (target to host direction) within a dedicated thread
        r = threading.Thread(target=self._reader)
        r.setDaemon(1)
        r.start()
        # start the writer (host to target direction)
        self._writer(fullmode)

    def _reader(self):
        """Loop forever, processing received serial data in terminal mode"""
        try:
            # Try to read as many bytes as possible at once, and use a short
            # timeout to avoid blocking for more data
            self._port.timeout = 0.050
            while self._resume:
                data = self._port.read(4096)
                if data:
                    try:
                        stdout.write(data.decode('utf8'))
                    except UnicodeDecodeError:
                        pass
                    stdout.flush()
        except KeyboardInterrupt:
            return
        except Exception as e:
            print("Exception: %s" % e)
            if self._debug:
                print(format_exc(), file=stderr)
            interrupt_main()

    def _writer(self, fullmode=False):
        """Loop and copy console->serial until EOF character is found"""
        while self._resume:
            try:
                c = getkey(fullmode)
                if platform == 'win32':
                    if ord(c) == 0x3:
                        raise KeyboardInterrupt()
                if fullmode and ord(c) == 0x1:  # Ctrl+A
                    self._cleanup()
                    return
                else:
                    self._port.write(c)
            except KeyboardInterrupt:
                print('%sAborting...' % os.linesep)
                self._cleanup()
                return

    def _cleanup(self):
        """Cleanup resource before exiting"""
        self._resume = False
        if self._port:
            # wait till the other thread completes
            time.sleep(0.5)
            try:
                rem = self._port.in_waiting()
            except Exception:
                # maybe a bug in underlying wrapper...
                rem = 0
            # consumes all the received bytes
            for _ in range(rem):
                self._port.read()
            self._port.close()
            self._port = None
            print('Bye.')
        for fd, att in self._termstates:
            termios.tcsetattr(fd, termios.TCSANOW, att)

    @staticmethod
    def _open_port(device, baudrate, logfile=False, debug=False):
        """Open the serial communication port"""
        # the following import enables serial protocol extensions
        try:
            if logfile:
                port = serialext.serial_for_url(device, do_not_open=True)
                basecls = port.__class__
                from pyftdi.serialext.logger import SerialLogger
                cls = type('Spy%s' % basecls.__name__,
                           (SerialLogger, basecls), {})
                port = cls(device, baudrate=baudrate,
                           timeout=0, logfile=logfile)
            else:
                port = serialext.serial_for_url(device,
                                                baudrate=baudrate,
                                                timeout=0,
                                                do_not_open=True)
            port.open()
            if not port.is_open:
                raise IOError('Cannot open port "%s"' % device)
            if debug:
                print("Using serial backend '%s'" % port.BACKEND)
            return port
        except IOError as ex:
            # SerialException derives from IOError
            raise


def main():
    """Main routine"""
    debug = False
    try:
        argparser = ArgumentParser(description=modules[__name__].__doc__)
        argparser.add_argument('-p', '--device', required=True,
                               help="serial port device name "
                                    "(list available ports with 'ftdi:///?')")
        argparser.add_argument(
            '-b', '--baudrate', dest='baudrate',
            help='serial port baudrate', default='115200')
        argparser.add_argument(
            '-r', '--reset', dest='reset',
            help='HW reset on DTR line', default=None)
        argparser.add_argument(
            '-s', '--select', dest='select',
            help='Mode selection on RTS line', default=None)
        argparser.add_argument(
            '-o', '--logfile', dest='logfile',
            help='path to the log file')
        if os.name in ('posix', ):
            argparser.add_argument(
                '-f', '--fullmode', dest='fullmode', action='store_true',
                help='use full terminal mode, exit with [Ctrl]+A')
        argparser.add_argument(
            '-d', '--debug', dest='debug', action='store_true',
            help='enable debug mode')
        args = argparser.parse_args()
        debug = args.debug

        miniterm = MiniTerm(device=args.device,
                            baudrate=to_int(args.baudrate),
                            logfile=args.logfile,
                            debug=args.debug)
        miniterm.run(os.name in ('posix', ) and args.fullmode or False,
                     args.reset, args.select)
    except Exception as e:
        print('\nError: %s' % e, file=stderr)
        if debug:
            print(format_exc(), file=stderr)
        exit(1)
    except KeyboardInterrupt:
        exit(2)


if __name__ == '__main__':
    main()
