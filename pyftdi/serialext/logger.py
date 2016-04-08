# Copyright (c) 2008-2015, Neotion
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

import sys
import types
from pyftdi.misc import hexdump
from time import time

__all__ = ['SerialLogger']


class SerialLogger(object):
    """Serial port wrapper to log input/output data to a log file"""

    def __init__(self, logpath):
        try:
            self._logger = open(logpath, "wt")
        except IOError, e:
            print >> sys.stderr, \
                "Cannot log data to %s: %s" % (logpath, str(e))
        self._port = None
        self._methods = {}
        self._last = time()

    def spy(self, port):
        self._port = port
        methods = [m for m in self.__class__.__dict__
                   if not m.startswith('_') and
                   hasattr(getattr(self.__class__, m), '__call__') and
                   m != 'spy']
        # replace the spied instance method with our own methods
        for method_name in methods:
            try:
                old_method = getattr(port.__class__, method_name)
            except AttributeError:
                old_method = None
            if old_method:
                new_method = getattr(self.__class__, method_name)
                setattr(port, method_name, types.MethodType(new_method, self))
                self._methods[method_name] = old_method

    def _print(self, header, string):
        if self._logger:
            now = time()
            delta = (now-self._last)*1000
            self._last = now
            print >>self._logger, "%s (%3.3f ms):\n%s" % \
                (header, delta, string)
            self._logger.flush()

    def _log_read(self, data):
        try:
            self._print('READ', hexdump(data))
        except Exception, e:
            print >>sys.stderr, 'Cannot log input data (%s)' % e

    def _log_write(self, data):
        try:
            self._print('WRITE', hexdump(data))
        except Exception, e:
            print >>sys.stderr, 'Cannot log output data (%s)' % e, data

    def _log_flush(self, type_):
        try:
            self._print('FLUSH', type_)
        except Exception, e:
            print >>sys.stderr, 'Cannot log flush action (%s)' % e

    def _log_waiting(self, count):
        try:
            self._print('INWAITING', '%d' % count)
        except Exception, e:
            print >>sys.stderr, 'Cannot log inwaiting (%s)' % e

    def _log_setBaudrate(self, baudrate):
        try:
            self._print('SETBAUDRATE', '%d' % baudrate)
        except Exception, e:
            print >>sys.stderr, 'Cannot log setBaudrate (%s)' % e

    def _log_setDTR(self, hwreset):
        try:
            self._print('SETDTR', '%s' % hwreset)
        except Exception, e:
            print >>sys.stderr, 'Cannot log setDTR (%s)' % e

    def _log_setRTS(self, startmode):
        try:
            self._print('SETRTS', '%d' % startmode)
        except Exception, e:
            print >>sys.stderr, 'Cannot log setRTS (%s)' % e

    def close(self):
        self._logger.close()
        self._methods['close'](self._port)

    def read(self, size=1):
        data = self._methods['read'](self._port, size)
        self._log_read(data)
        return data

    def write(self, data):
        self._methods['write'](self._port, data)
        if len(data):
            self._log_write(data)

    def inWaiting(self):
        wait = self._methods['inWaiting'](self._port)
        self._log_waiting(wait)
        return wait

    def flush(self):
        self._log_flush('I+O')
        self._methods['flush'](self._port)

    def flushInput(self):
        self._log_flush('I')
        self._methods['flushInput'](self._port)

    def flushOutput(self):
        self._log_flush('O')
        self._methods['flushOutput'](self._port)

    def setBaudrate(self, baudrate):
        self._log_setBaudrate(baudrate)
        self._methods['setBaudrate'](self._port, baudrate)

    def setDTR(self, hwreset):
        self._log_setDTR(hwreset)
        self._methods['setDTR'](self._port, hwreset)

    def setRTS(self, startmode):
        self._log_setRTS(startmode)
        self._methods['setRTS'](self._port, startmode)
