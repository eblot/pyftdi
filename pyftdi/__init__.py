# Copyright (c) 2010-2020 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2010-2016, Neotion
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

__version__ = '0.43.0'
__title__ = 'PyFtdi'
__description__ = 'FTDI device driver (pure Python)'
__uri__ = 'http://github.com/eblot/pyftdi'
__doc__ = __description__ + ' <' + __uri__ + '>'
__author__ = 'Emmanuel Blot'
# For all support requests, please open a new issue on GitHub
__email__ = 'emmanuel.blot@free.fr'
__license__ = 'Modified BSD'
__copyright__ = 'Copyright (c) 2011-2019 Emmanuel Blot'


from logging import WARNING, NullHandler, getLogger


class FtdiLogger:

    log = getLogger('pyftdi')
    log.addHandler(NullHandler())
    log.setLevel(level=WARNING)

    @classmethod
    def set_formatter(cls, formatter):
        handlers = list(cls.log.handlers)
        for handler in handlers:
            handler.setFormatter(formatter)

    @classmethod
    def get_level(cls):
        return cls.log.getEffectiveLevel()

    @classmethod
    def set_level(cls, level):
        cls.log.setLevel(level=level)
