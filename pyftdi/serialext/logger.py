# Copyright (c) 2010-2016 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2008-2016, Neotion
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#pylint: disable-msg=no-member
#pylint: disable-msg=broad-except
#pylint: disable-msg=invalid-name
#pylint: disable-msg=super-with-arguments
#pylint: disable-msg=missing-function-docstring
#pylint: disable-msg=missing-module-docstring

from sys import stderr
from time import time
from ..misc import hexdump


__all__ = ['SerialLogger']


class SerialLogger:
    """Serial port wrapper to log input/output data to a log file.
    """

    def __init__(self, *args, **kwargs):
        logpath = kwargs.pop('logfile', None)
        if not logpath:
            raise ValueError('Missing logfile')
        try:
            self._logger = open(logpath, "wt")
        except IOError as e:
            print("Cannot log data to %s: %s" % (logpath, str(e)),
                  file=stderr)
        self._last = time()
        self._log_init(*args, **kwargs)
        super(SerialLogger, self).__init__(*args, **kwargs)

    def open(self,):
        self._log_open()
        super(SerialLogger, self).open()

    def close(self):
        self._log_close()
        self._logger.close()
        super(SerialLogger, self).close()

    def read(self, size=1):
        data = super(SerialLogger, self).read(size)
        self._log_read(data)
        return data

    def write(self, data):
        if data:
            self._log_write(data)
        super(SerialLogger, self).write(data)

    def flush(self):
        self._log_flush()
        super(SerialLogger, self).flush()

    def reset_input_buffer(self):
        self._log_reset('I')
        super(SerialLogger, self).reset_input_buffer()

    def reset_output_buffer(self):
        self._log_reset('O')
        super(SerialLogger, self).reset_output_buffer()

    def send_break(self, duration=0.25):
        self._log_signal('BREAK', 'for %.3f' % duration)
        super(SerialLogger, self).send_break()

    def _update_break_state(self):
        self._log_signal('BREAK', self._break_state)
        super(SerialLogger, self)._update_break_state()

    def _update_rts_state(self):
        self._log_signal('RTS', self._rts_state)
        super(SerialLogger, self)._update_rts_state()

    def _update_dtr_state(self):
        self._log_signal('DTR', self._dtr_state)
        super(SerialLogger, self)._update_dtr_state()

    @property
    def cts(self):
        level = super(SerialLogger, self).cts
        self._log_signal('CTS', level)
        return level

    @property
    def dsr(self):
        level = super(SerialLogger, self).dsr
        self._log_signal('DSR', level)
        return level

    @property
    def ri(self):
        level = super(SerialLogger, self).ri
        self._log_signal('RI', level)
        return level

    @property
    def cd(self):
        level = super(SerialLogger, self).cd
        self._log_signal('CD', level)
        return level

    def in_waiting(self):
        count = super(SerialLogger, self).in_waiting()
        self._log_waiting(count)
        return count

    def _print(self, header, string=None):
        if self._logger:
            now = time()
            delta = (now-self._last)*1000
            self._last = now
            print("%s (%3.3f ms):\n%s" % (header, delta, string or ''),
                  file=self._logger)
            self._logger.flush()

    def _log_init(self, *args, **kwargs):
        try:
            self._print(
                'NEW', '  args: %s %s' %
                (', '.join(args),
                 ', '.join({'%s=%s' % it for it in kwargs.items()})))
        except Exception as e:
            print('Cannot log init (%s)' % e, file=stderr)

    def _log_open(self):
        try:
            self._print('OPEN')
        except Exception as e:
            print('Cannot log open (%s)' % e, file=stderr)

    def _log_close(self):
        try:
            self._print('CLOSE')
        except Exception as e:
            print('Cannot log close (%s)' % e, file=stderr)

    def _log_read(self, data):
        try:
            self._print('READ', hexdump(data))
        except Exception as e:
            print('Cannot log input data (%s)' % e, file=stderr)

    def _log_write(self, data):
        try:
            self._print('WRITE', hexdump(data))
        except Exception as e:
            print('Cannot log output data (%s)' % e, data, file=stderr)

    def _log_flush(self):
        try:
            self._print('FLUSH')
        except Exception as e:
            print('Cannot log flush action (%s)' % e, file=stderr)

    def _log_reset(self, type_):
        try:
            self._print('RESET BUFFER', type_)
        except Exception as e:
            print('Cannot log reset buffer (%s)' % e, file=stderr)

    def _log_waiting(self, count):
        try:
            self._print('INWAITING', '%d' % count)
        except Exception as e:
            print('Cannot log inwaiting (%s)' % e, file=stderr)

    def _log_signal(self, name, value):
        try:
            self._print(name.upper(), '%s' % value)
        except Exception as e:
            print('Cannot log %s (%s)' % (name, e), file=stderr)
