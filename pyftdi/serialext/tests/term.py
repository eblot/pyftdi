# Copyright (c) 2008-2016, Neotion
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

"""Console helper routines

"""

import os
import sys

_INIT = False


def _init_term(fullterm):
    """Internal terminal initialization function"""
    if os.name == 'nt':
        import msvcrt
        return True
    elif os.name == 'posix':
        import termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        new = termios.tcgetattr(fd)
        new[3] = new[3] & ~termios.ICANON & ~termios.ECHO
        new[6][termios.VMIN] = 1
        new[6][termios.VTIME] = 0
        if fullterm:
            new[6][termios.VINTR] = 0
            new[6][termios.VSUSP] = 0
        termios.tcsetattr(fd, termios.TCSANOW, new)
        def cleanup_console():
            termios.tcsetattr(fd, termios.TCSAFLUSH, old)
            # terminal modes have to be restored on exit...
        sys.exitfunc = cleanup_console
        return True
    else:
        return True


def getkey(fullterm=False):
    """Return a key from the current console, in a platform independent way"""
    # there's probably a better way to initialize the module without
    # relying onto a singleton pattern. To be fixed
    global _INIT
    if not _INIT:
        _INIT = _init_term(fullterm)
    if os.name == 'nt':
        # w/ py2exe, it seems the importation fails to define the global
        # symbol 'msvcrt', to be fixed
        import msvcrt
        while 1:
            z = msvcrt.getch()
            if z == '\3':
                raise KeyboardInterrupt('Ctrl-C break')
            if z == '\0':
                msvcrt.getch()
            else:
                if z == '\r':
                    return '\n'
                return z
    elif os.name == 'posix':
        c = os.read(sys.stdin.fileno(), 1)
        return c
    else:
        import time
        time.sleep(1)
        return None


def is_term():
    """Tells whether the current stdout/stderr stream are connected to a
    terminal (vs. a regular file or pipe)"""
    return sys.stdout.isatty()


def is_colorterm():
    """Tells whether the current terminal (if any) support colors escape
    sequences"""
    terms = ['xterm-color', 'ansi']
    return sys.stdout.isatty() and os.environ.get('TERM') in terms
