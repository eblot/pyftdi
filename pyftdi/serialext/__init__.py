# Copyright (c) 2010-2016 Emmanuel Blot <emmanuel.blot@free.fr>
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

"""Serial modules compliant with pyserial APIs
"""

try:
    from serial.serialutil import SerialException
except ImportError:
    raise ImportError("Python serial module not installed")
try:
    from serial import VERSION, serial_for_url as serial4url
    version = tuple([int(x) for x in VERSION.split('.')])
    if version < (3, 0):
        raise ValueError
except (ValueError, IndexError, ImportError):
    raise ImportError("pyserial 3.0+ is required")
try:
    from serial import protocol_handler_packages
    protocol_handler_packages.append('pyftdi.serialext')
except ImportError:
    raise SerialException('Cannot register pyftdi extensions')

serial_for_url = serial4url


def touch():
    """Do nothing, only for static checkers than do not like module import
       with no module references
    """
    pass
