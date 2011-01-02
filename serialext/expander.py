import os
import sys

__all__ = ['SerialExpander']


class SerialExpanderError(Exception):
    """Raised when the expander is in trouble"""
    pass


class SerialExpander(object):
    """Provides extension classes compatible with PySerial module"
       Create a class on request, to prevent useless dependency on
       non-standard PySerial module"""

    @staticmethod
    def serialclass(device, use_logger=False):
        """Provide a pyserial class that supports high-speed serial port on
        Apple computer (usually through a USB-RS232 adaptor key)"""
        try:
            import serial
        except ImportError:
            raise SerialExpanderError("Python serial module not installed")
        # default serial class
        type_ = serial.Serial
        backend = 'serial'
        if device.startswith('ftdi://'):
            # for now, assume the USB device is a FTDI device
            # a USB dispatcher should be implemented here
            from ftdiext import SerialFtdi
            type_ = type('SerialFtdi', (serial.SerialBase,),
                         dict(SerialFtdi.__dict__))
            backend = 'ftdi'
        elif os.path.exists(device):
            import stat
            from socketext import SerialSocket
            if stat.S_ISSOCK(os.stat(device)[0]):
                type_ = type('SerialSocket', (serial.SerialBase,),
                             dict(SerialSocket.__dict__))
                backend = 'socket'
        if backend == 'serial' and sys.platform.lower() in ('darwin'):
            # hack for Mac OS X hosts: the underlying termios system library
            # cannot cope with baudrates > 230kbps and pyserial << 9.7
            version = os.uname()[2].split('.')
            if 'version' not in serial.__dict__ or \
              serial.version[0] < 9 or serial.version[1] < 7:
                # Tiger or above can support arbitrary serial speeds
                if int(version[0]) >= 8:
                    # remove all speeds not supported with TERMIOS
                    # so that pyserial never attempts to use them directly
                    for b in serial.baudrate_constants.keys():
                        if b > 230400:
                            del serial.baudrate_constants[b]
                    from darwinext import SerialDarwin
                    type_ = type('SerialDarwin', (serial.Serial,),
                                 dict(SerialDarwin.__dict__))
        if use_logger:
            from loggerext import SerialLoggerPort
            type_ = type('SerialLoggerPort', (type_,),
                         dict(SerialLoggerPort.__dict__))
        return (type_, backend)
