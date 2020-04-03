"""PyUSB virtual FTDI device."""

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

from logging import getLogger
from pyftdi.tracer import FtdiMpsseTracer


class VirtMpsseTracer(FtdiMpsseTracer):
    """Reuse MPSSE tracer as a MPSSE command decoder engine.
    """

    def __init__(self, version: int):
        super().__init__(version)
        self.log = getLogger('pyftdi.virt.mpsse')

