#!/usr/bin/env python3

"""Simple Python serial terminal
"""

# Copyright (c) 2010-2020, Emmanuel Blot <emmanuel.blot@free.fr>
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
from os import environ, linesep, name as os_name, read as os_read, stat
from sys import modules, platform, stderr, stdin, stdout
from time import sleep
from threading import Event, Thread
from traceback import format_exc
from _thread import interrupt_main
MSWIN = platform == 'win32'
if not MSWIN:
    from termios import TCSANOW, tcgetattr, tcsetattr
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.misc import to_bps, add_custom_devices

#pylint: disable-msg=invalid-name
#pylint: disable-msg=import-error

if MSWIN:
    import msvcrt
    from subprocess import call
else:
    msvcrt = None
    call = None

#pylint: enable-msg=invalid-name
#pylint: enable-msg=import-error

"""
Windows function key scan codes to ANSI escape code dictionary:

Pause/Break, Ctrl+Alt+Del, Ctrl+Alt+arrows not mapable
Input: ordinal of char from msvcrt.getch()
Output: bytes string of ANSI escape sequence for linux/xterm
        numerical used over linux specifics for Home and End
        VT or CSI escape sequences used when linux has no sequence
        something unique for keys without an escape function
\x1b == Escape key
"""

if MSWIN:
    winfnkeys = {
        #Ctrl + Alt + Backspace
        14:     b'\x1b^H',
        #Ctrl + Alt + Enter
        28:     b'\x1b\r',
        # Pause/Break
        29:     b'\x1c',
        # Arrows
        72:     b'\x1b[A',
        80:     b'\x1b[B',
        77:     b'\x1b[C',
        75:     b'\x1b[D',
        # Arrows (Alt)
        152:    b'\x1b[1;3A',
        160:    b'\x1b[1;3B',
        157:    b'\x1b[1;3C',
        155:    b'\x1b[1;3D',
        # Arrows (Ctrl)
        141:    b'\x1b[1;5A',
        145:    b'\x1b[1;5B',
        116:    b'\x1b[1;5C',
        115:    b'\x1b[1;5D',
        #Ctrl + Tab
        148:    b'\x1b[2J',
        # Cursor (Home, Ins, Del...)
        71:     b'\x1b[1~',
        82:     b'\x1b[2~',
        83:     b'\x1b[3~',
        79:     b'\x1b[4~',
        73:     b'\x1b[5~',
        81:     b'\x1b[6~',
        # Cursor + Alt
        151:    b'\x1b[1;3~',
        162:    b'\x1b[2;3~',
        163:    b'\x1b[3;3~',
        159:    b'\x1b[4;3~',
        153:    b'\x1b[5;3~',
        161:    b'\x1b[6;3~',
        # Cursor + Ctrl (xterm)
        119:    b'\x1b[1;5H',
        146:    b'\x1b[2;5~',
        147:    b'\x1b[3;5~',
        117:    b'\x1b[1;5F',
        134:    b'\x1b[5;5~',
        118:    b'\x1b[6;5~',
        # Function Keys (F1 - F12)
        59:     b'\x1b[11~',
        60:     b'\x1b[12~',
        61:     b'\x1b[13~',
        62:     b'\x1b[14~',
        63:     b'\x1b[15~',
        64:     b'\x1b[17~',
        65:     b'\x1b[18~',
        66:     b'\x1b[19~',
        67:     b'\x1b[20~',
        68:     b'\x1b[21~',
        133:    b'\x1b[23~',
        134:    b'\x1b[24~',
        # Function Keys + Shift (F11 - F22)
        84:     b'\x1b[23;2~',
        85:     b'\x1b[24;2~',
        86:     b'\x1b[25~',
        87:     b'\x1b[26~',
        88:     b'\x1b[28~',
        89:     b'\x1b[29~',
        90:     b'\x1b[31~',
        91:     b'\x1b[32~',
        92:     b'\x1b[33~',
        93:     b'\x1b[34~',
        135:    b'\x1b[20;2~',
        136:    b'\x1b[21;2~',
        # Function Keys + Ctrl (xterm)
        94:     b'\x1bOP',
        95:     b'\x1bOQ',
        96:     b'\x1bOR',
        97:     b'\x1bOS',
        98:     b'\x1b[15;2~',
        99:     b'\x1b[17;2~',
        100:    b'\x1b[18;2~',
        101:    b'\x1b[19;2~',
        102:    b'\x1b[20;3~',
        103:    b'\x1b[21;3~',
        137:    b'\x1b[23;3~',
        138:    b'\x1b[24;3~',
        # Function Keys + Alt (xterm)
        104:    b'\x1b[11;5~',
        105:    b'\x1b[12;5~',
        106:    b'\x1b[13;5~',
        107:    b'\x1b[14;5~',
        108:    b'\x1b[15;5~',
        109:    b'\x1b[17;5~',
        110:    b'\x1b[18;5~',
        111:    b'\x1b[19;5~',
        112:    b'\x1b[20;5~',
        113:    b'\x1b[21;5~',
        139:    b'\x1b[23;5~',
        140:    b'\x1b[24;5~',
    }

class MiniTerm:
    """A mini serial terminal to demonstrate pyserial extensions"""

    DEFAULT_BAUDRATE = 115200

    def __init__(self, device, baudrate=None, parity=None, rtscts=False,
                 debug=False):
        self._termstates = []
        if not MSWIN and stdout.isatty():
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
        print('Entering minicom mode @ %d bps' % self._port.baudrate)
        stdout.flush()
        if MSWIN:
            call('', shell=True)
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
                char = getkey()
                if fullmode and ord(char) == 0x2:    # Ctrl+B
                    self._cleanup()
                    return
                if MSWIN:
                    if ord(char) in (0, 224):
                        char = getkey()
                        self._port.write(winfnkeys[ord(char)])
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
                    if MSWIN:
                        self._port.write(b'\3')
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
            for tfd, att in self._termstates:
                tcsetattr(tfd, TCSANOW, att)
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
            if version < (3, 0):
                raise ValueError
        except (ValueError, IndexError, ImportError):
            raise ImportError("pyserial 3.0+ is required")
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
                backend = port.BACKEND if hasattr(port, 'BACKEND') else '?'
                print("Using serial backend '%s'" % backend)
            return port
        except SerialException as exc:
            raise IOError(str(exc))


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


def init_term(fullterm: bool) -> None:
    """Internal terminal initialization function"""
    if os_name == 'nt':
        return True
    if os_name == 'posix':
        import termios
        tfd = stdin.fileno()
        old = termios.tcgetattr(tfd)
        new = termios.tcgetattr(tfd)
        new[3] = new[3] & ~termios.ICANON & ~termios.ECHO
        new[6][termios.VMIN] = 1
        new[6][termios.VTIME] = 0
        if fullterm:
            new[6][termios.VINTR] = 0
            new[6][termios.VSUSP] = 0
        termios.tcsetattr(tfd, termios.TCSANOW, new)
        def cleanup_console():
            termios.tcsetattr(tfd, termios.TCSAFLUSH, old)
            # terminal modes have to be restored on exit...
        register(cleanup_console)
        return True
    else:
        return True


def getkey() -> str:
    """Return a key from the current console, in a platform independent way"""
    # there's probably a better way to initialize the module without
    # relying onto a singleton pattern. To be fixed
    if MSWIN:
        # w/ py2exe, it seems the importation fails to define the global
        # symbol 'msvcrt', to be fixed
        while 1:
            char = msvcrt.getch()
            if char == '\r':
                return '\n'
            return char
    elif os_name == 'posix':
        char = os_read(stdin.fileno(), 1)
        return char
    else:
        import time
        time.sleep(1)
        return None


def is_term():
    """Tells whether the current stdout/stderr stream are connected to a
    terminal (vs. a regular file or pipe)"""
    return stdout.isatty()


def is_colorterm():
    """Tells whether the current terminal (if any) support colors escape
    sequences"""
    terms = ['xterm-color', 'ansi']
    return stdout.isatty() and environ.get('TERM') in terms


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

        init_term(args.fullmode)
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
        exit(1)
    except KeyboardInterrupt:
        exit(2)


if __name__ == '__main__':
    main()
