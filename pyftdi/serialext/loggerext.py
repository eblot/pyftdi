# Copyright (c) 2008-2011, Neotion
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
from pyftdi.pyftdi.misc import hexdump

__all__ = ['SerialLoggerPort']


class SerialLoggerPort(object):
    """Serial port implementation to log input/output data to a log file"""

    def __init__(self, **kwargs):
        self._logger = None
        if 'logger' in kwargs:
            self.set_logger(kwargs['logger'])
            del kwargs['logger']
        super(SerialLoggerPort, self).__init__(**kwargs)

    def open(self, **kwargs):
        return super(SerialLoggerPort, self).open(**kwargs)

    def _log_read(self, data):
        if not self._logger:
            return
        try:
            print >>self._logger, "READ:\n%s" % hexdump(data)
        except:
            print >> sys.stderr, 'Cannot log read data'

    def _log_write(self, data):
        if not self._logger:
            return
        try:
            print >>self._logger, "WRITE:\n%s" % hexdump(data)
        except:
            print >>sys.stderr, 'Cannot log written data'

    def _log_flush(self, type_):
        if not self._logger:
            return
        try:
            print >>self._logger, "FLUSH:  %s\n" % type_
        except:
            print >>sys.stderr, 'Cannot log flush'

    def _log_waiting(self, count):
        if not self._logger:
            return
        try:
            print >>self._logger, "INWAITING: %d\n" % count
        except:
            print >>sys.stderr, 'Cannot log flush'

    def set_logger(self, logger):
        try:
            self._logger = open(logger, "wt")
        except IOError, e:
            print >> sys.stderr, \
                "Cannot log data to %s" % kwargs['logger']

    def read(self, size=1):
        data = super(SerialLoggerPort, self).read(size)
        self._log_read(data)
        return data

    def write(self, data):
        super(SerialLoggerPort, self).write(data)
        if len(data):
            self._log_write(data)

    def inWaiting(self):
        wait = super(SerialLoggerPort, self).inWaiting()
        self._log_waiting(wait)
        return wait

    def flush(self):
        self._log_flush('I+O')
        super(SerialLoggerPort, self).flush()

    def flushInput(self):
        self._log_flush('I')
        super(SerialLoggerPort, self).flushInput()

    def flushOutput(self):
        self._log_flush('O')
        super(SerialLoggerPort, self).flushOutput()
