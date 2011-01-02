"""Console routines"""

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
    # there's probably a better way to initialize the module without going
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
