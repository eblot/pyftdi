import array, fcntl
import serial

__all__ = ['SerialDarwin']

class SerialDarwin:
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
