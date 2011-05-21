# pyprolific - A pure Python PL2303 driver
# Copyright (C) 2011 Emmanuel Blot <emmanuel.blot@free.fr>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

"""pl2303 - A pure Python PL2303 driver on top of pyusb

Author:  Emmanuel Blot <emmanuel.blot@free.fr>
License: MIT
Require: pyusb
"""

import os
import struct
import threading
import usb.core
import usb.util
from array import array as Array

from pyftdi.misc import hexdump, hexline

__all__ = ['Prolific', 'ProlificError']

class ProlificError(IOError):
    """Communication error with the Prolific device"""
    pass


class Prolific(object):
    """PL2303 device driver"""
    pass
