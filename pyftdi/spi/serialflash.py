# Copyright (c) 2010-2012, Emmanuel Blot <emmanuel.blot@free.fr>
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

import sys
import time
from pyftdi.pyftdi.ftdi import Ftdi
from pyftdi.pyftdi.spi import SpiController
from pyftdi.pyftdi.misc import hexdump
from array import array as Array


class SerialFlashNotSupported(Exception):
    """Exception thrown when a non-existing feature is invoked"""

class SerialFlashUnknownJedec(SerialFlashNotSupported):
    """Exception thrown when a JEDEC identifier is not recognized"""
    def __init__(self, jedec):
        from binascii import hexlify
        SerialFlashNotSupported.__init__(self, "Unknown flash device: %s" % \
                                         hexlify(jedec))

class SerialFlashTimeout(Exception):
    """Exception thrown when a flash command cannot be completed in a timely
       manner"""

class SerialFlashValueError(ValueError):
    """Exception thrown when a parameter is out of range"""


class SerialFlash(object):
    """Interface of a generic SPI flash device"""

    FEAT_NONE = 0x000         # No special feature
    FEAT_LOCK = 0x001         # Basic, revertable locking
    FEAT_INVLOCK = 0x002      # Inverted (bottom/top) locking
    FEAT_SECTLOCK = 0x004     # Arbitrary sector locking
    FEAT_OTPLOCK = 0x008      # OTP locking available
    FEAT_UNIQUEID = 0x010     # Unique ID
    FEAT_SECTERASE = 0x100    # Can erase whole sectors
    FEAT_HSECTERASE = 0x200   # Can erase half sectors
    FEAT_SUBSECTERASE = 0x400 # Can erase sub sectors

    def read(self, address, length):
        """Read a sequence of bytes from the specified address."""
        raise NotImplementedError()

    def write(self, address, data):
        """Write a sequence of bytes, starting at the specified address."""
        raise NotImplementedError()

    def erase(self, address, length):
        """Erase a block of bytes. Address and length depends upon device-
           specific constraints."""
        raise NotImplementedError()

    def can_erase(self, address, length):
        """Tells whether a defined area can be erased on the Spansion flash
           device. It does not take into account any locking scheme."""
        raise NotImplementedError()

    def is_busy(self):
        """Reports whether the flash may receive commands or is actually
           being performing internal work"""
        raise NotImplementedError()

    def get_capacity(self):
        """Get the flash device capacity in bytes"""
        raise NotImplementedError()

    def get_capabilities(self):
        """Flash device capabilities."""
        return SerialFlash.FEAT_NONE

    def get_locks(self):
        """Report the currently write-protected areas of the device."""
        raise NotImplementedError()

    def set_lock(self, address, length, otp=False):
        """Create a write-protected area. Device should have been unlocked
           first."""
        raise NotImplementedError()

    def unlock(self):
        """Make the whole device read/write"""
        pass

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        raise NotImplemented()

    @property
    def features(self):
        """Flash device capabilities."""
        return SerialFlash.FEAT_NONE

    @property
    def unique_id(self):
        """Return the unique ID of the flash, if it exists"""
        raise NotImplementedError()


class SerialFlashManager(object):
    """Serial flash manager.

       Automatically detects and instanciate the proper flash device class
       based on the JEDEC identifier which is read out from the device itself.
    """

    CMD_JEDEC_ID = 0x9F

    def __init__(self, vendor, product, interface=1):
        self._ctrl = SpiController(silent_clock=False)
        self._ctrl.configure(vendor, product, interface)

    def get_flash_device(self, cs=0):
        """Obtain an instance of the detected flash device"""
        spi = self._ctrl.get_port(cs)
        jedec = SerialFlashManager.read_jedec_id(spi)
        if not jedec:
            # it is likely that the latency setting is too low if this
            # condition is encountered
            raise SerialFlashUnknownJedec("Unable to read JEDEC Id")
        return SerialFlashManager._get_flash(spi, jedec)

    @staticmethod
    def read_jedec_id(spi):
        """Read flash device JEDEC identifier (3 bytes)"""
        jedec_cmd = Array('B', [SerialFlashManager.CMD_JEDEC_ID])
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

    def is_busy(self):
        return _Gen25FlashDevice._is_busy(self._read_status())

    def unlock(self):
        ewsr_cmd = Array('<B', [_Gen25FlashDevice.CMD_EWSR])
        self._spi.exchange(ewsr_cmd)
        wrsr_cmd = Array('B', [_Gen25FlashDevice.CMD_WRSR,
                               (~_Gen25FlashDevice.STATUS_BP)&0xff])
        self._spi.exchange(wrsr_cmd)

    def read(self, address, length):
        if address+length > len(self):
            raise SerialFlashValueError('Out of range')
        buf = Array('B')
        while length > 0:
            size = min(length, SpiController.PAYLOAD_MAX_LENGTH)
            data = self._read_hi_speed(address, size)
            length -= len(data)
            address += len(data)
            buf.extend(data)
        return buf

    def write(self, address, data):
        """Write a sequence of bytes, starting at the specified address."""
        length = len(data)
        if address+length > len(self):
            raise SerialFlashValueError('Cannot fit in flash area')
        if not isinstance(data, Array):
            data = Array('B', data)
        pos = 0
        while pos < length:
            size = min(length-pos, _Gen25FlashDevice.PAGE_SIZE)
            self._write(address, data[pos:pos+size])
            address += size
            pos += size

    def erase(self, address, length):
        """Erase sectors/blocks/chip of a "generic" flash device.
           Erasure algorithm:
           The area to erase span across one or more sectors, which can be
           accounted as bigger blocks, depending on the start and end address
           of the location to be erased
           address ----------------- length ---------------------->
                 v                                                 v
              ...|LSS|LSS|LSS| LHS | LHS |  S  |  RHS  | RHS |RSS|RSS|RSS|....

             LSS: left subsector, RSS: right subsector
             LHS: left half-sector, RHS: right half-sector (32KB)
             S: (large) sector (64kB)
           Depending on the device capabilities, half-sector may or may not be
           used. This routine tries to find and erase the biggest flash page
           segments so that erasure time is decreased
           """
        # sanity check
        self.can_erase(address, length)
        # first page to erase on the left-hand size
        start = address
        # last page to erase on the left-hand size
        end = start + length
        # first page to erase on the right-hand size
        rstart = start
        # last page to erase on the right-hand size
        rend = end
        if self.features & SerialFlash.FEAT_SECTERASE:
            # Check whether one or more whole large sector can be erased
            s_start = (start+_Gen25FlashDevice.SECTOR_SIZE-1) & \
                        ~(_Gen25FlashDevice.SECTOR_SIZE-1)
            s_end = end & ~(_Gen25FlashDevice.SECTOR_SIZE-1)
            if s_start < s_end:
                self._erase_blocks(self.CMD_ERASE_SECTOR,
                                   self.get_timings('sector'),
                                   s_start, s_end,
                                   self.SECTOR_SIZE)
                # update the left-hand end marker
                end = s_start
                # update the right-hand start marker
                if s_end > rstart:
                    rstart = s_end
        if self.features & SerialFlash.FEAT_HSECTERASE:
            # Check whether one or more left halfsectors can be erased
            hsl_start = (start+_Gen25FlashDevice.HSECTOR_SIZE-1) & \
                           ~(_Gen25FlashDevice.HSECTOR_SIZE-1)
            hsl_end = end & ~(_Gen25FlashDevice.HSECTOR_SIZE-1)
            if hsl_start < hsl_end:
                self._erase_blocks(self.CMD_ERASE_HSECTOR,
                                   self.get_timings('hsector'),
                                   hsl_start, hsl_end,
                                   self.HSECTOR_SIZE)
                # update the left-hand end marker
                end = hsl_start
                # update the right-hand start marker
                if hsl_end > rstart:
                    rstart = hsl_end
        if self.features & SerialFlash.FEAT_SUBSECTERASE:
            # Check whether one or more left subsectors can be erased
            ssl_start = (start+_Gen25FlashDevice.SUBSECTOR_SIZE-1) & \
                           ~(_Gen25FlashDevice.SUBSECTOR_SIZE-1)
            ssl_end = end & ~(_Gen25FlashDevice.SUBSECTOR_SIZE-1)
            if ssl_start < ssl_end:
                self._erase_blocks(self.CMD_ERASE_SUBSECTOR,
                                   self.get_timings('subsector'),
                                   ssl_start, ssl_end,
                                   self.SUBSECTOR_SIZE)
                # update the right-hand start marker
                if ssl_end > rstart:
                    rstart = ssl_end
        if self.features & SerialFlash.FEAT_HSECTERASE:
            # Check whether one or more whole left halfsectors can be erased
            hsr_start = (rstart+_Gen25FlashDevice.HSECTOR_SIZE-1) & \
                           ~(_Gen25FlashDevice.HSECTOR_SIZE-1)
            hsr_end = rend & ~(_Gen25FlashDevice.HSECTOR_SIZE-1)
            if hsr_start < hsr_end:
                self._erase_blocks(self.CMD_ERASE_HSECTOR,
                                   self.get_timings('hsector'),
                                   hsr_start, hsr_end,
                                   self.HSECTOR_SIZE)
                # update the right-hand start marker
                if hsr_end > rstart:
                    rstart = hsr_end
        if self.features & SerialFlash.FEAT_SUBSECTERASE:
            # Check whether one or more whole right subsectors can be erased
            ssr_start = (rstart+_Gen25FlashDevice.SUBSECTOR_SIZE-1) & \
                           ~(_Gen25FlashDevice.SUBSECTOR_SIZE-1)
            ssr_end = rend & ~(_Gen25FlashDevice.SUBSECTOR_SIZE-1)
            if ssr_start < ssr_end:
                self._erase_blocks(self.CMD_ERASE_SUBSECTOR,
                                   self.get_timings('subsector'),
                                   ssr_start, ssr_end,
                                   self.SUBSECTOR_SIZE)

    def _can_erase(self, address, length):
        """Tells whether a defined area can be erased on the Spansion flash
           device. It does not take into account any locking scheme."""
        if address & (self.erase_size-1):
            # start address should be aligned on a subsector boundary
            raise SerialFlashValueError('Start address not aligned on a '
                                        'erase sector boundary')
        if ((length-1) & (self.erase_size-1)) != (self.erase_size-1):
            # length should be a multiple of a subsector
            raise SerialFlashValueError('End address not aligned on a '
                                        'erase sector boundary')

    @property
    def erase_size(self):
        """Return the erase size in bytes"""
        features = self.features
        if features & SerialFlash.FEAT_SUBSECTERASE:
            return self.SUBSECTOR_SIZE
        if features & SerialFlash.FEAT_HSECTERASE:
            return self.HSECTOR_SIZE
        if features & SerialFlash.FEAT_SECTERASE:
            return self.SECTOR_SIZE
        raise SerialFlashNotSupported("Unknown erase size")

    @staticmethod
    def _jedec2int(jedec, maxlength=3):
        return tuple([ord(x) for x in jedec[:maxlength]])

    def _read_lo_speed(self, address, length):
        read_cmd = Array('B', [_Gen25FlashDevice.CMD_READ_LO_SPEED,
                               (address>>16)&0xff, (address>>8)&0xff,
                               address&0xff])
        return self._spi.exchange(read_cmd, length)

    def _read_hi_speed(self, address, length):
        read_cmd = Array('B', [_Gen25FlashDevice.CMD_READ_HI_SPEED,
                               (address>>16)&0xff, (address>>8)&0xff,
                               address&0xff, 0])
        return self._spi.exchange(read_cmd, length)

    def _write(self, address, data):
        # take care not to roll over the end of the flash page
        if address & (_Gen25FlashDevice.PAGE_SIZE-1):
            up = (address+(_Gen25FlashDevice.PAGE_SIZE-1)) & \
                    ~(_Gen25FlashDevice.PAGE_SIZE-1)
            count = min(len(data), up-address)
            sequences = [(address, data[:count]), (up, data[count:])]
        else:
            sequences = [(address, data)]
        for addr, buf in sequences:
            self._enable_write()
            wcmd = Array('B', [_Gen25FlashDevice.CMD_PROGRAM_PAGE,
                               (addr>>16)&0xff, (addr>>8)&0xff,
                               addr&0xff])
            wcmd.extend(data)
            self._spi.exchange(wcmd)
            self._wait_for_completion(self.get_timings('page'))

    def _read_status(self):
        read_cmd = Array('B', [_Gen25FlashDevice.CMD_READ_STATUS])
        #self._spi.flush()
        data = self._spi.exchange(read_cmd, 1)
        if len(data) != 1:
            raise SerialFlashTimeout("Unable to retrieve flash status")
        return data[0]

    def _enable_write(self):
        wren_cmd = Array('B', [_Gen25FlashDevice.CMD_WRITE_ENABLE])
        self._spi.exchange(wren_cmd)

    def _disable_write(self):
        wrdi_cmd = Array('B', [_Gen25FlashDevice.CMD_WRITE_DISABLE])
        self._spi.exchange(wrdi_cmd)

    def _erase_blocks(self, command, times, start, end, size):
        """Erase one or more blocks"""
        while start < end:
            #print "ERASE BLOCK 0x%06x..0x%06x" % (start, start+size-1)
            self._enable_write()
            cmd = Array('B', [command, (start>>16)&0xff,
                              (start>>8)&0xff, start&0xff])
            self._spi.exchange(cmd)
            self._wait_for_completion(times)
            start += size

    def _wait_for_completion(self, times):
        typical_time, max_time = times
        timeout = time.time()
        timeout += typical_time+max_time
        cycle = 0
        while self.is_busy():
            # need to wait at least once
            if cycle and time.time() > timeout:
                raise SerialFlashTimeout('Command timeout (%d cycles)' % cycle)
            time.sleep(typical_time)
            cycle +=1

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
    CMD_WRITE_STATUS_REGISTER = 0x01
    SST25_AAI = 0b01000000 # AAI mode activation flag
    DEVICES = { 0x41 : 2<<20, 0x4A : 4<<20 }
    SPI_FREQ_MAX = 66 # MHz
    TIMINGS = { 'subsector' : (0.025, 0.025) # 25 ms
                'hsector' : (0.025, 0.025) # 25 ms
                'sector' : (0.025, 0.025) # 25 ms
              }

    def __init__(self, spi, jedec):
        if not Sst25FlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        capacity = _Gen25FlashDevice._jedec2int(jedec)[-1]
        self._size = Sst25FlashDevice.DEVICES[capacity]
        self._spi = spi

    def __len__(self):
        return self._size

    def __str__(self):
        return 'SST SST25 %d MB' % (len(self)>>20, )

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        return Sst25FlashDevice.TIMINGS[time]

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
        if address+len(data) > len(self):
            raise SerialFlashValueError('Cannot fit in flash area')
        if not isinstance(data, Array):
            data = Array('B', data)
        length = len(data)
        if (address&0x1) or (length&0x1) or (length==0):
            raise AssertionError("Alignement/size not supported")
        self._unprotect()
        self._enable_write()
        aai_cmd = Array('B', [Sst25FlashDevice.CMD_PROGRAM_WORD,
                              (address>>16)&0xff,
                              (address>>8)&0xff,
                              address&0xff,
                              data.pop(0), data.pop(0)])
        offset = 0
        percent = 0.0
        while True:
            percent = (1000.0*offset/length)
            offset += 2
            self._spi.exchange(aai_cmd)
            while self.is_busy():
                time.sleep(0.01) # 10 ms
            if not data:
                break
            aai_cmd = Array('B', [Sst25FlashDevice.CMD_PROGRAM_WORD,
                                  data.pop(0), data.pop(0)])
        self._disable_write()

    def can_erase(self, address, length):
        """Verifies that a defined area can be erased on the SST flash device.
        """
        # SST25 does not support OTP locking, for now only test the erase
        # range validity
        if address & (self.SUBSECTOR_SIZE-1):
            raise SerialFlashValueError('Start address should be aligned on a '
                                        'subsector boundary')
        if length & (self.SUBSECTOR_SIZE-1):
            raise SerialFlashValueError('End address should be aligned on a '
                                        'subsector boundary')
        if (address + length) > len(self):
            raise SerialFlashValueError('Would erase over the flash capacity')

    def _unprotect(self):
        """Disable default protection for all sectors"""
        unprotect = Array('B',
                          [Sst25FlashDevice.CMD_WRITE_STATUS_REGISTER, 0x00])
        self._enable_write()
        self._spi.exchange(unprotect)
        while self.is_busy():
            time.sleep(0.01) # 10 ms


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
    CMD_READ_CONFIG = 0x35
    SPI_FREQ_MAX = 104 # MHz (P series only)
    DEVICES = { 0x15 : 4<<20, 0x16 : 8<<20 }
    TIMINGS = { 'page' : (0.0015, 0.003) # 1.5/3 ms
                'subsector' : (0.2, 0.8) # 200/800 ms
                'sector' : (0.5, 2.0) # 0.5/2 s
                'bulk' : (32, 64) # seconds
              }

    def __init__(self, spi, jedec):
        if not S25FlFlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        capacity = _Gen25FlashDevice._jedec2int(jedec)[-1]
        self._size = S25FlFlashDevice.DEVICES[capacity]
        self._spi = spi
        self._spi.set_frequency(S25FlFlashDevice.SPI_FREQ_MAX*1E06)

    def __len__(self):
        return self._size

    def __str__(self):
        return 'Spansion S25FL %d MB' % (len(self)>>20, )

    @property
    def features(self):
        """Flash device features"""
        # note that subsector erasure is only supported for a 2 * sectorsize
        # -long area at start OR end of the flash. can_erase asserts this
        # condition
        return SerialFlash.FEAT_SECTERASE|SerialFlash.FEAT_SUBSECTERASE

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        return S25FlFlashDevice.TIMINGS[time]

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

    def _can_erase(self, address, length):
        """Verifies that a defined area can be erased on the Spansion flash
           device. It does not take into account any locking scheme.
        """
        # we first need to check the current configuration register, as a
        # previous configuration may prevent from altering some of the bits
        readcfg_cmd = Array('B', [S25FlFlashDevice.CMD_READ_CONFIG])
        config = self._spi.exchange(readcfg_cmd, 1)[0]
        if config & S25FlFlashDevice.CR_TBPARM:
            # "parameter zone" is defined in the high sectors
            border = len(self)-2*S25FlFlashDevice.SECTOR_SIZE
            ls_size = S25FlFlashDevice.SECTOR_SIZE
            rs_size = S25FlFlashDevice.SUBSECTOR_SIZE
        else:
            # "parameter zone" is defined in the low sectors
            border = 2*S25FlFlashDevice.SECTOR_SIZE
            ls_size = S25FlFlashDevice.SUBSECTOR_SIZE
            rs_size = S25FlFlashDevice.SECTOR_SIZE
        start = address;
        fend = address+length;
        # sanity check
        if (start > fend) or (fend > len(self)):
            raise SerialFlashValueError('Out of flash storage range')
        if (fend > border) and (start < border):
            end = border
        else:
            end = fend
        if start >= border:
            size = rs_size
        else:
            size = ls_size
        while True: # expect 1 (no border cross) or 2 loops (border cross)
            # sanity check
            if start & (size-1):
                # start address should be aligned on a (sub)sector boundary
                raise SerialFlashValueError('Start address not aligned on a '
                                            'sector boundary')
            # sanity check
            if (((end-start)-1) & (size-1)) != (size-1):
                # length should be a multiple of a (sub)sector
                raise SerialFlashValueError('End address not aligned on a '
                                            'sector boundary')
            # stop condition
            if (start >= border) or (end >= fend):
                break
            start = end
            end = fend
            size = rs_size


class M25PxFlashDevice(_Gen25FlashDevice):
    """Numonix M25P/M25PX flash device implementation"""

    JEDEC_ID = 0x20
    P_SERIALFLASH_ID = 0x71
    PX_SERIALFLASH_ID = 0x20
    SPI_FREQ_MAX = 75 # MHz (P series only)
    DEVICES = { 0x15 : 2<<20, 0x16 : 4<<20 }
    TIMINGS = { #'page' : (0.0015, 0.003) # 1.5/3 ms
                'subsector' : (0.150, 0.150) # 150/150 ms
                'sector' : (3.0, 3.0) # 3/3 s
                #'bulk' : (32, 64) # seconds
              }

    def __init__(self, spi, jedec):
        if not M25PFlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        device, capacity = _Gen25FlashDevice._jedec2int(jedec)[1:]
        self._size = M25PFlashDevice.DEVICES[device]
        self._spi = spi
        self._spi.set_frequency(M25PFlashDevice.SPI_FREQ_MAX*1E06)
        self._var = (device == M25PFlashDevice.PX_SERIALFLASH_ID) \
                    and 'PX' or 'X'

    def __len__(self):
        return self._size

    def __str__(self):
        return 'Numonix M25%s%02d %d MB' % \
            (self._var, len(self)>>17, len(self)>>20)

    @property
    def features(self):
        """Flash device features"""
        return SerialFlash.FEAT_SECTERASE|SerialFlash.FEAT_SUBSECTERASE
               # SerialFlash.FEAT_LOCK|SerialFlash.FEAT_INVLOCK

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        return S25FlFlashDevice.TIMINGS[time]

    @staticmethod
    def match(jedec):
        """Tells whether this class support this JEDEC identifier"""
        manufacturer, device, capacity = _Gen25FlashDevice._jedec2int(jedec)
        if manufacturer != M25PFlashDevice.JEDEC_ID:
            return False
        if not device in (M25PFlashDevice.P_SERIALFLASH_ID,
                          M25PFlashDevice.PX_SERIALFLASH_ID):
            return False
        if capacity not in M25PFlashDevice.DEVICES:
            return False
        return True


class W25xFlashDevice(_Gen25FlashDevice):
    """Winbond W25Q/W25X flash device implementation"""

    JEDEC_ID = 0xEF
    X_SERIALFLASH_ID = 0x30
    Q_SERIALFLASH_ID = 0x40
    SPI_FREQ_MAX = 104 # MHz
    CMD_READ_UID = 0x4B
    UID_LEN = 0x8 # 64 bits
    READ_UID_WIDTH = 4 # 4 dummy bytes
    DEVICES = { 0x15 : 2<<20, 0x16 : 4<<20, 0x17 : 8<<20 }
    TIMINGS = { #'page' : (0.0015, 0.003) # 1.5/3 ms
                'subsector' : (0.200, 0.200) # 200/200 ms
                'sector' : (1.0, 1.0) # 1/1 s
                #'bulk' : (32, 64) # seconds
              }

    def __init__(self, spi, jedec):
        if not W25xFlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        device, capacity = _Gen25FlashDevice._jedec2int(jedec)[1:]
        self._size = M25PFlashDevice.DEVICES[device]
        self._spi = spi
        self._spi.set_frequency(W25xFlashDevice.SPI_FREQ_MAX*1E06)
        self._var = (device == W25xFlashDevice.X_SERIALFLASH_ID) \
                    and 'X' or 'Q'

    def __len__(self):
        return self._size

    def __str__(self):
        return 'Winbond W25%s%02d %d MB' % \
            (self._var, len(self)>>17, len(self)>>20)

    @property
    def features(self):
        """Flash device features"""
        return SerialFlash.FEAT_SECTERASE|SerialFlash.FEAT_SUBSECTERASE
               # SerialFlash.FEAT_LOCK|SerialFlash.FEAT_INVLOCK

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        return W25xFlashDevice.TIMINGS[time]

    @staticmethod
    def match(jedec):
        """Tells whether this class support this JEDEC identifier"""
        manufacturer, device, capacity = _Gen25FlashDevice._jedec2int(jedec)
        if manufacturer != W25xFlashDevice.JEDEC_ID:
            return False
        if not device in (W25xFlashDevice.X_SERIALFLASH_ID,
                          W25xFlashDevice.Q_SERIALFLASH_ID):
            return False
        if capacity not in W25xFlashDevice.DEVICES:
            return False
        return True


class Mx25lFlashDevice(_Gen25FlashDevice):
    """Macronix MX25L flash device implementation"""

    JEDEC_ID = 0xC2
    D_SERIALFLASH_ID = 0x9E
    E_SERIALFLASH_ID = 0x26
    SPI_FREQ_MAX = 104 # MHz
    DEVICES = { 0x15 : 2<<20, 0x16 : 4<<20, 0x17 : 8<<20, 0x18 : 16<<20 }
    TIMINGS = { #'page' : (0.0015, 0.003) # 1.5/3 ms
                'subsector' : (0.300, 0.300) # 300/300 ms
                'hsector' : (2.0, 2.0) # 2/2 s
                'sector' : (2.0, 2.0) # 2/2 s
                #'bulk' : (32, 64) # seconds
              }

    def __init__(self, spi, jedec):
        if not Mx25lFlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        device, capacity = _Gen25FlashDevice._jedec2int(jedec)[1:]
        self._size = M25PFlashDevice.DEVICES[device]
        self._spi = spi
        self._spi.set_frequency(Mx25lFlashDevice.SPI_FREQ_MAX*1E06)
        self._var = (device == Mx25lFlashDevice.D_SERIALFLASH_ID) \
                    and 'D' or 'E'

    def __len__(self):
        return self._size

    def __str__(self):
        return 'Macronix M25L55%s%02d %d MB' % \
            (self._var, len(self)>>17, len(self)>>20)

    @property
    def features(self):
        """Flash device features"""
        return SerialFlash.FEAT_SECTERASE|SerialFlash.FEAT_HSECTERASE|\
               SerialFlash.FEAT_SUBSECTERASE
               # SerialFlash.FEAT_LOCK|SerialFlash.FEAT_INVLOCK

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        return Mx25lFlashDevice.TIMINGS[time]

    @staticmethod
    def match(jedec):
        """Tells whether this class support this JEDEC identifier"""
        manufacturer, device, capacity = _Gen25FlashDevice._jedec2int(jedec)
        if manufacturer != Mx25lFlashDevice.JEDEC_ID:
            return False
        if not device in (Mx25lFlashDevice.D_SERIALFLASH_ID,
                          Mx25lFlashDevice.E_SERIALFLASH_ID):
            return False
        if capacity not in Mx25lFlashDevice.DEVICES:
            return False
        return True


class En25qFlashDevice(_Gen25FlashDevice):
    """EON EN25Q flash device implementation"""

    JEDEC_ID = 0x1C
    SERIALFLASH_ID = 0x30
    SPI_FREQ_MAX = 100 # MHz
    DEVICES = { 0x15 : 2<<20, 0x16 : 4<<20, 0x17 : 8<<20 }
    TIMINGS = { #'page' : (0.0015, 0.003) # 1.5/3 ms
                'subsector' : (0.300, 0.300) # 300/300 ms
                'sector' : (2.0, 2.0) # 2/2 s
                #'bulk' : (32, 64) # seconds
              }

    def __init__(self, spi, jedec):
        if not En25qFlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        capacity = _Gen25FlashDevice._jedec2int(jedec)[-1]
        self._size = En25qFlashDevice.DEVICES[capacity]
        self._spi = spi
        self._spi.set_frequency(En25qFlashDevice.SPI_FREQ_MAX*1E06)

    def __len__(self):
        return self._size

    def __str__(self):
        return 'Eon EN25Q%02d %d MB' % (len(self)>>17, len(self)>>20)

    @property
    def features(self):
        """Flash device features"""
        return SerialFlash.FEAT_SECTERASE|SerialFlash.FEAT_SUBSECTERASE
               # SerialFlash.FEAT_LOCK|SerialFlash.FEAT_INVLOCK

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        return En25qFlashDevice.TIMINGS[time]

    @staticmethod
    def match(jedec):
        """Tells whether this class support this JEDEC identifier"""
        manufacturer, device, capacity = _Gen25FlashDevice._jedec2int(jedec)
        if manufacturer != En25qFlashDevice.JEDEC_ID:
            return False
        if not device != En25qFlashDevice.SERIALFLASH_ID:
            return False
        if capacity not in En25qFlashDevice.DEVICES:
            return False
        return True


class At25FlashDevice(_Gen25FlashDevice):
    """Atmel AT25 flash device implementation"""

    JEDEC_ID = 0x1F
    SERIALFLASH_ID = 0x30
    SPI_FREQ_MAX = 85 # MHz
    DEVICES = { 0x46 : 2<<20, 0x47 : 4<<20, 0x48 : 8<<20 }
    TIMINGS = { #'page' : (0.0015, 0.003) # 1.5/3 ms
                'subsector' : (0.200, 0.200) # 200/200 ms
                'sector' : (0.950, 0.950) # 950/950 ms
                #'bulk' : (32, 64) # seconds
              }

    def __init__(self, spi, jedec):
        if not At25FlashDevice.match(jedec):
            raise SerialFlashUnknownJedec(jedec)
        capacity = _Gen25FlashDevice._jedec2int(jedec)[-1]
        self._size = At25FlashDevice.DEVICES[capacity]
        self._spi = spi
        self._spi.set_frequency(At25FlashDevice.SPI_FREQ_MAX*1E06)

    def __len__(self):
        return self._size

    def __str__(self):
        return 'Atmel AT25%02d %d MB' % (len(self)>>17, len(self)>>20)

    @property
    def features(self):
        """Flash device features"""
        return SerialFlash.FEAT_SECTERASE|SerialFlash.FEAT_SUBSECTERASE
               # SerialFlash.FEAT_LOCK|SerialFlash.FEAT_INVLOCK

    def get_timings(self, time):
        """Get a time tuple (typical, max)"""
        return At25FlashDevice.TIMINGS[time]

    @staticmethod
    def match(jedec):
        """Tells whether this class support this JEDEC identifier"""
        manufacturer, device, capacity = _Gen25FlashDevice._jedec2int(jedec)
        if manufacturer != At25FlashDevice.JEDEC_ID:
            return False
        if not device != At25FlashDevice.SERIALFLASH_ID:
            return False
        if capacity not in At25FlashDevice.DEVICES:
            return False
        return True
