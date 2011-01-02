#!/usr/bin/env python
"Simple Python serial terminal"

import os
import sys
import time
import threading
from util.misc import to_int
from util.term import getkey

class MiniTerm(object):
    """A mini serial terminal to demonstrate pyserial extensions"""

    def __init__(self, device=None, baudrate=115200, logfile=None, 
                 debug=False):
        self._device = device
        self._baudrate = baudrate
        self._logfile = logfile
        self._debug = debug
        if not self._device:
            if os.name == 'nt':
                self._device = 'COM1'
            elif os.name == 'posix':
                (system, _, _, _, _) = os.uname()
                if system.lower() == 'darwin':
                    self._device = '/dev/cu.usbserial'
                else:
                    self._device = '/dev/ttyS0'
            else:
                raise AssertionError('Serial port unknown')
        self._port = self._open_port(self._device, self._baudrate,
                                     self._logfile, self._debug)
        self._resume = False

        # hwreset = False
        # self._port.setDTR(hwreset)
        # time.sleep(0.200)
        # self._port.setDTR(not hwreset)
        # time.sleep(0.100)
        

    def __del__(self):
        try:
            self._cleanup()
        except Exception:
            pass
 
    def run(self, fullmode):
        """Switch to a pure serial terminal application"""
        # wait forever, although Windows is stupid and does not signal Ctrl+C, 
        # so wait use a 1/2-second timeout that gives some time to check for a
        # Ctrl+C break then polls again...
        print 'Entering minicom mode'
        sys.stdout.flush()
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
                    sys.stdout.write(data)
                    sys.stdout.flush()
        except KeyboardInterrupt:
            return
        except Exception, e:
            print "Exception: %s" % e
            import thread
            thread.interrupt_main()
            
    def _writer(self, fullmode=False):
        """Loop and copy console->serial until EOF character is found"""
        while self._resume:
            try:
                c = getkey(fullmode)
                if fullmode and ord(c) == 0x1: # Ctrl+A
                    self._cleanup()
                    return
                else:
                    self._port.write(c)
            except KeyboardInterrupt:
                print '%sAborting...' % os.linesep
                self._cleanup()
                return

    def _cleanup(self):
        """Cleanup resource before exiting"""
        self._resume = False
        if self._port:
            # wait till the other thread completes
            time.sleep(0.5)
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
            print 'Bye.'

    @staticmethod
    def _open_port(device, baudrate, logfile=False, debug=False):
        """Open the serial communication port"""
        from serialext import SerialExpander
        serialclass, backend = \
            SerialExpander.serialclass(device, logfile and True)
        import serial
        try:
            port = serialclass(port=device, 
                               baudrate=baudrate,
                               timeout=0)
            if logfile:
                port.set_logger(logfile)
            if not port.isOpen(): 
                port.open()
            if not port.isOpen():
                raise AssertionError('Cannot open port "%s"' % device)
            if debug:
                print "Using serial backend '%s'" % backend
            return port
        except serial.serialutil.SerialException, e:
            raise AssertionError(str(e))


def get_options():
    """Parse and execute the command line and optionnally a config file"""
    from optparse import OptionParser
    usage = '%prog [options]\n' \
            'Configure a remote Neotion board over a serial line\n'
    optparser = OptionParser(usage=usage)
    optparser.add_option('-d', '--debug', dest='debug',
                         action='store_true',
                         help='Enable debug mode')
    if os.name in ('posix', ):
        optparser.add_option('-f', '--fullmode', dest='fullmode',
                             action='store_true',
                             help='Use full terminal mode, exit with [Ctrl]+A')
    optparser.add_option('-p', '--port', dest='device',
                         help='Serial port device name')
    optparser.add_option('-b', '--baudrate', dest='baudrate',
                         help='Serial port baudrate', default='115200')
    optparser.add_option('-o', '--logfile', dest='logfile',
                         help='Path to the log file')
    options, _ = optparser.parse_args(sys.argv[1:])
    return optparser, options

def main():
    """Main routine"""
    optparser, options = get_options()
    try:
        miniterm = MiniTerm(device=options.device,
                            baudrate=to_int(options.baudrate),
                            logfile=options.logfile,
                            debug=options.debug)
        miniterm.run(options.fullmode)
    except (AssertionError, IOError, ValueError), e:
        print >> sys.stderr, '\nError: %s' % e
        if options.debug:
            import traceback
            print >> sys.stderr, traceback.format_exc()
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(2)

# pip install virtualenv
# virtualenv ve
# ve/bin/python pip install pyserial
# cd .../pyusb-1.0.0-a0 && .../ve/bin/python setup.py install
# DYLD_LIBRARY_PATH=/usr/local/homebrew/lib PYTHONPATH=. 
#   ve/bin/python serialext/tests/pyterm.py -p ftdi://ftdi:ft4232/3 -d

if __name__ == '__main__':
    main()
