# Copyright (c) 2010-2016 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2008-2015, Neotion
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

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
