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

import array, fcntl
import serial

__all__ = ['SerialDarwin']


class SerialDarwin(object):
    """Serial port implementation dedicated to the Darwin kernel (Mac OS X)
       It allows to circumvent the POSIX/termios current limitation of the
       baudrate to 230400bps.
       It is a true hack of the original pyserial implementation, and will
       hopefully be deprecated once pyserial implement a better serial port
       support for Apple computer.
       It supports custom baudrate with no defined upper bound, and has been
       tested with baudrates up to 3Mbps. It does not improve communication
       speed on Mac OS 10.3 and below.
    """
    IOSSIOSPEED = 0x80045402 #_IOW('T', 2, speed_t)

    def _reconfigurePort(port):
        """Augment serial.Serial._reconfigurePort w/ IOKit call"""
        try:
            serial.Serial._reconfigurePort(port)
        except AttributeError:
            # use IOKit-specific call to set up high speeds
            buf = array.array('i', [int(port._baudrate)])
            fcntl.ioctl(port.fd, SerialDarwin.IOSSIOSPEED, buf, 1)
        except:
            raise AssertionError('Cannot reconfigure Darwin serial port')
