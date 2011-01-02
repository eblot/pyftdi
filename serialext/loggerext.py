import sys
from util.misc import hexdump

__all__ = ['SerialLoggerPort']

class SerialLoggerPort(object):
    """Serial port implementation to log input/output data to a log file"""
    
    def __init__(self, **kwargs):
        self._logger = None
        if 'logger' in kwargs:
            self.set_logger(kwargs['logger'])
            del kwargs['logger']
        super(self.__class__, self).__init__(**kwargs)
        
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
        data = super(self.__class__, self).read(size)
        self._log_read(data)
        return data
        
    def write(self, data):
        super(self.__class__, self).write(data)
        if len(data):
            self._log_write(data)

    def inWaiting(self):
        wait = super(self.__class__, self).inWaiting()
        self._log_waiting(wait)
        return wait

    def flush(self):
        self._log_flush('I+O')
        super(self.__class__, self).flush()

    def flushInput(self):
        self._log_flush('I')
        super(self.__class__, self).flushInput()

    def flushOutput(self):
        self._log_flush('O')
        super(self.__class__, self).flushOutput()
