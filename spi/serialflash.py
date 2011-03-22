# Copyright (c) 2010-2011, Emmanuel Blot <emmanuel.blot@free.fr>
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

from pyftdi import Ftdi
from pyftdi.spi import SpiController
from pyftdi.misc import hexdump
import array
import struct
import sys
import time


class SerialFlashNotSupported(Exception):
    """Exception thrown when a non-existing feature is invoked"""


class SerialFlashUnknownJedec(SerialFlashNotSupported):
    """Exception thrown when a JEDEC identifier is not recognized"""
    def __init__(self, jedec):
        from binascii import hexlify
        SerialFlashNotSupported.__init__(self,
            msg="Unknown flash device: %s" % hexlify(jedec))


class SerialFlash(object):
    """Interface of a generic SPI flash device"""

    def read(self, address, length):
        """Read a sequence of bytes from the specified address."""
        raise NotImplemented()

    def write(self, address, data):
        """Write a sequence of bytes, starting at the specified address."""
        raise NotImplemented()

    def erase(self, address, length):
        """Erase a block of bytes. Address and length depends upon device-
           specific constraints."""
        raise NotImplemented()

    def is_busy(self):
        """Reports whether the flash may receive commands or is actually
           being performing internal work"""
        raise NotImplemented()

    def get_capacity(self):
        """Get the flash device capacity in bytes"""
        raise NotImplemented()

    def get_capabilities(self):
        """List the flash device capabilities."""
        return ''

    def get_locks(self):
        """Report the currently write-protected areas of the device."""
        raise NotImplemented()

    def set_lock(self, address, length, otp=False):
        """Create a write-protected area. Device should have been unlocked
           first."""
        raise NotImplemented()

    def unlock(self):
        """Make the whole device read/write"""
        pass

    def get_unique_id(self):
        """Return the unique ID of the flash, if it exists"""
        raise NotImplemented()


class SerialFlashManager(object):
    """Serial flash manager.

       Automatically detects and instanciate the proper flash device class
       based on the JEDEC identifier which is read out from the device itself.
    """

    CMD_JEDEC_ID = 0x9F

    def __init__(self, vendor, product, interface=1):
        self._ctrl = SpiController()
        self._ctrl.configure(vendor, product, interface)

    def get_flash_device(self, cs=0):
        """Obtain an instance of the detected flash device"""
        spi = self._ctrl.get_port(cs)
        jedec = SerialFlashManager.read_jedec_id(spi)
        return SerialFlashManager._get_flash(spi, jedec)

    @staticmethod
    def read_jedec_id(spi):
        """Read flash device JEDEC identifier (3 bytes)"""
        jedec_cmd = struct.pack('<B', SerialFlashManager.CMD_JEDEC_ID)
        return spi.exchange(jedec_cmd, 3).tostring()

    @staticmethod
    def _get_flash(spi, jedec):
        devices = []
        contents = sys.modules[__name__].__dict__
        for name in contents:
            if name.endswith('FlashDevice') and not name.startswith('_'):
                devices.append(contents[name])
        for device in devices:
            if device.match(jedec):
                return device(spi, jedec)
        raise SerialFlashUnknownJedec(jedec)


class _Gen25FlashDevice(SerialFlash):
    """Generic flash device implementation.

       Most SPI flash devices share commands and parameters. Those devices
       generally contains '25' in their reference. However, there are virtually
       no '25' device that is fully compliant with any counterpart from
       a concurrent manufacturer. Most differences are focused on lock and
       security features. Here comes the mess... This class contains the most
       common implementation for the basic feature, and each physical device
       inherit from this class for feature specialization.
    """

    CMD_READ_LO_SPEED = 0x03 # Read @ low speed
    CMD_READ_HI_SPEED = 0x0B # Read @ high speed
    CMD_READ_STATUS = 0x05 # Read status register
    CMD_WRITE_ENABLE = 0x06 # Write enable
    CMD_WRITE_DISABLE = 0x04 # Write disable
    CMD_PROGRAM_PAGE = 0x02 # Write page
    CMD_ERASE_BLOCK = 0xD8 # Erase full block
    CMD_EWSR = 0x50 # Enable write status register
    CMD_WRSR = 0x01 # Write status register
    CMD_ERASE_SUBSECTOR = 0x20
    CMD_ERASE_HSECTOR = 0x52
    CMD_ERASE_SECTOR = 0xD8
    CMD_ERASE_CHIP = 0xC7
    CMD_PROGRAM_PAGE = 0x02

    PAGE_DIV = 8
    SUBSECTOR_DIV = 12
    HSECTOR_DIV = 15
    SECTOR_DIV = 16
    PAGE_SIZE = (1<<PAGE_DIV)
    SUBSECTOR_SIZE = (1<<SUBSECTOR_DIV)
    HSECTOR_SIZE = (1<<HSECTOR_DIV)
    SECTOR_SIZE = (1<<SECTOR_DIV)
    SPI_FREQUENCY_MAX = 50 # MHz
    ADDRESS_WIDTH = 3

    SR_WIP = 0b00000001 # Busy/Work-in-progress bit
    SR_WEL = 0b00000010 # Write enable bit
    SR_BP0 = 0b00000100 # bit protect #0
    SR_BP1 = 0b00001000 # bit protect #1
    SR_BP2 = 0b00010000 # bit protect #2
    SR_BP3 = 0b00100000 # bit protect #3
    SR_TBP = SR_BP3     # top-bottom protect bit
    SR_SP = 0b01000000
    SR_BPL = 0b10000000
    SR_PROTECT_NONE = 0 # BP[0..2] = 0
    SR_PROTECT_ALL = 0b00011100 # BP[0..2] = 1
    SR_LOCK_PROTECT = SR_BPL
    SR_UNLOCK_PROTECT = 0
    SR_BPL_SHIFT = 2

    def __init__(self, spiport):
        self._spi = spiport

    def get_capacity(self):
        """Get the flash device capacity in bytes"""
        return len(self)

    def is_busy(self):
        return _Gen25FlashDevice._is_busy(self._read_status())

    def unlock(self):
        ewsr_cmd = struct.pack('<B', _Gen25FlashDevice.CMD_EWSR)
        self._spi.exchange(ewsr_cmd)
        wrsr_cmd = struct.pack('<BB', _Gen25FlashDevice.CMD_WRSR,
                               (~_Gen25FlashDevice.STATUS_BP)&0xff)
        self._spi.exchange(wrsr_cmd)

    def read(self, address, length):
        buf = array.array('B')
        while length > 0:
            size = min(length, _Gen25FlashDevice.PAGE_SIZE)
            data = self._read_hi_speed(address, size)
            length -= len(data)
            address += len(data)
            buf.extend(data)
        return buf

    def write(self, address, data):
        """Write a sequence of bytes, starting at the specified address."""
        length = len(data)
        pos = 0
        while pos < length:
            size = min(length-pos, SpiController.PAYLOAD_MAX_LENGTH)
            self._write(address, data[pos:pos+size])
            address += size
            pos += size

    @staticmethod
    def _jedec2int(jedec, maxlength=3):
        return tuple([ord(x) for x in jedec[:maxlength]])

    def _read_lo_speed(self, address, length):
        read_cmd = struct.pack('<BBBB', _Gen25FlashDevice.CMD_READ_LO_SPEED,
                               (address>>16)&0xff, (address>>8)&0xff,
                               address&0xff)
        return self._spi.exchange(read_cmd, length)

    def _read_hi_speed(self, address, length):
        read_cmd = struct.pack('<BBBBB', _Gen25FlashDevice.CMD_READ_HI_SPEED,
                               (address>>16)&0xff, (address>>8)&0xff,
                               address&0xff, 0)
        return self._spi.exchange(read_cmd, length)

    def _write(self, address, data):
        # take care not to roll over the end of the flash page
        if address & (_Gen25FlashDevice.PAGE_SIZE-1):
            up = (address+(_Gen25FlashDevice.PAGE_SIZE-1)) & \
                    ~(_Gen25FlashDevice.PAGE_SIZE-1);
            count = min(len(data), up-address)
            sequences = [(address, data[:count]), (up, data[count:])]
        else:
            sequences = [(address, data)]
        for addr, buf in sequences:
            # print "write 0x%06x %d '%s'" % (addr, len(buf), buf)
            self._enable_write()
            wcmd = struct.pack('<BBBB', _Gen25FlashDevice.CMD_PROGRAM_PAGE,
                               (addr>>16)&0xff, (addr>>8)&0xff, addr&0xff)
            self._spi.exchange(wcmd + data)
            while self.is_busy():
                time.sleep(self.PAGE_WTIME)

    def _read_status(self):
        read_cmd = struct.pack('<B', _Gen25FlashDevice.CMD_READ_STATUS)
        return struct.unpack('<B', self._spi.exchange(read_cmd, 1))[0]

    def _enable_write(self):
        wren_cmd = struct.pack('<B', _Gen25FlashDevice.CMD_WRITE_ENABLE)
        self._spi.exchange(wren_cmd)

    def _disable_write(self):
        wrdi_cmd = struct.pack('<B', _Gen25FlashDevice.CMD_WRITE_DISABLE)
        self._spi.exchange(wrdi_cmd)

    def _erase_sector(self, address):
        self._enable_write()
        erblk_cmd = struct.pack('<BBBB', _Gen25FlashDevice.CMD_ERASE_BLOCK,
                                (address>>16)&0xff,
                                (address>>8)&0xff,
                                address&0xff)
        self._spi.exchange(erblk_cmd)
        while self.is_busy():
            time.sleep(self.SECTOR_ETIME_MAX)
        self._disable_write()

    @staticmethod
    def _is_busy(status):
        return (status & _Gen25FlashDevice.SR_WIP) and True or False

    @staticmethod
    def _is_wren(status):
        return (status & _Gen25FlashDevice.SR_WEL) and True or False


class Sst25FlashDevice(_Gen25FlashDevice):
    """SST25 flash device implementation"""

    JEDEC_ID = 0xBF
    SERIALFLASH_ID = 0x25
    CMD_PROGRAM_BYTE = 0x02
    CMD_PROGRAM_WORD = 0xAD # Auto address increment (for write command)
    SST25_AAI = 0b01000000 # AAI mode activation flag
    DEVICES = { 0x41 : 2<<20, 0x4A : 4<<20 }
    SUBSECTOR_ETIME_MAX = 0.025 # 25 ms
    HSECTOR_ETIME_MAX = 0.025 # 25 ms
    SECTOR_ETIME_MAX = 0.025 # 25 ms
    SPI_FREQ_MAX = 66 # MHz
    SPI_SETUP_TIME = 5E-09 # 5 ns
    SPI_HOLD_TIME = 5E-09 # 5 ns

    def __init__(self, spi, jedec):
        if not Sst25FlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        device = _Gen25FlashDevice._jedec2int(jedec)[-1]
        self._size = Sst25FlashDevice.DEVICES[device]
        self._spi = spi

    def __len__(self):
        return self._size

    def __str__(self):
        return 'SST SST25 %d MB' % (len(self)>>20, )

    @staticmethod
    def match(jedec):
        """Tells whether this class support this JEDEC identifier"""
        manufacturer, device, capacity = _Gen25FlashDevice._jedec2int(jedec)
        if manufacturer != Sst25FlashDevice.JEDEC_ID:
            return False
        if device != Sst25FlashDevice.SERIALFLASH_ID:
            return False
        if capacity not in Sst25FlashDevice.DEVICES:
            return False
        return True

    def write(self, address, data):
        """SST25 uses a very specific implementation to write data. It offers
           very poor performances, because the device lacks an internal buffer
           which translates into an ultra-heavy load on SPI bus. However, the
           device offers lightning-speed for flash data erasure"""
        if isinstance(data, str):
            data = [ord(x) for x in data]
        length = len(data)
        if (address&0x1) or (length&0x1) or (length==0):
            raise AssertionError("Alignement/size not supported")
        self._enable_write()
        aai_cmd = struct.pack('<BBBBBB', Sst25FlashDevice.CMD_PROGRAM_WORD,
                              (address>>16)&0xff,
                              (address>>8)&0xff,
                              address&0xff,
                              data.pop(0), data.pop(0))
        offset = 0
        percent = 0.0
        while True:
            percent = (1000.0*offset/length)
            #print "Address %06x (%2.1f%%)\r" % (address + offset, percent/10),
            offset += 2
            self._spi.exchange(aai_cmd)
            while self.is_busy():
                time.sleep(0.01) # 10 ms
            if not data:
                break
            aai_cmd = struct.pack('<BBB', Sst25FlashDevice.CMD_PROGRAM_WORD,
                                  data.pop(0), data.pop(0))
        #print ""
        self._disable_write()


class S25FlFlashDevice(_Gen25FlashDevice):
    """Spansion S25FL flash device implementation"""

    JEDEC_ID = 0x01
    SERIALFLASH_ID = 0x02
    CR_FREEZE = 0x01
    CR_QUAD = 0x02
    CR_TBPARM = 0x04
    CR_BPNV = 0x08
    CR_LOCK = 0x10
    CR_TBPROT = 0x20
    READ_CONFIG = 0x35
    SECTOR_ETIME_MAX = 3 # 3000 ms
    SUBSECTOR_ETIME_MAX = 0.8 # 800 ms
    SPI_FREQ_MAX = 104 # MHz (P series only)
    SPI_SETUP_TIME = 3E-09 # 3 ns
    SPI_HOLD_TIME = 3E-09 # 3 ns
    DEVICES = { 0x15 : 4<<20, 0x16 : 8<<20 }

    def __init__(self, spi, jedec):
        if not S25FlFlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        device = _Gen25FlashDevice._jedec2int(jedec)[-1]
        self._size = S25FlFlashDevice.DEVICES[device]
        self._spi = spi
        self._spi.set_frequency(S25FlFlashDevice.SPI_FREQ_MAX*1E06)

    def __len__(self):
        return self._size

    def __str__(self):
        return 'Spansion S25FL %d MB' % (len(self)>>20, )

    @staticmethod
    def match(jedec):
        """Tells whether this class support this JEDEC identifier"""
        manufacturer, device, capacity = _Gen25FlashDevice._jedec2int(jedec)
        if manufacturer != S25FlFlashDevice.JEDEC_ID:
            return False
        if device != S25FlFlashDevice.SERIALFLASH_ID:
            return False
        if capacity not in S25FlFlashDevice.DEVICES:
            return False
        return True


if __name__ == '__main__':
    import time
    mgr = SerialFlashManager(0x403, 0x6010, 1)
    flash = mgr.get_flash_device()
    print "Flash device: %s" % flash
    #delta = time.time()
    #data = flash.read(0, len(flash))
    #delta = time.time()-delta
    #length = len(data)
    #print "%d bytes in %d seconds @ %d KB/s" % \
    #    (length, delta, length/(1024*delta))
    flash.write(0x3c0020, 'This is a serial SPI flash test')
    data = flash.read(0x3c0020, 128).tostring()
    #data = flash.read(0xeff0, 128).tostring()
    from pyftdi.misc import hexdump
    print hexdump(data)
