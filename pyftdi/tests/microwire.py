#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2018, Stephen Goadhouse <sgoadhouse@virginia.edu>
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

from binascii import hexlify
from doctest import testmod
from os import environ
from pyftdi import FtdiLogger
from pyftdi.ftdi import FtdiError
from pyftdi.spi import SpiController, SpiIOError
from pyftdi.misc import hexdump
from array import array
from sys import modules, stderr, stdout
from time import sleep
import logging
import unittest


class MicrowireData93LC56BTest(object):
    """Basic class for a Microchip MicroWire 93LC56B data flash device
       selected as CS0, SPI mode 0, Active High polarity for CS and
       Bi-directional data. MicroWire is very similar to SPI so the
       basic SpiController can be used.

       Test setup: UM232H connected to the EEPROM on a FT4232H-56Q
       Mini Module. The FT4232H is forced into reset by connecting the
       RT# pin (CN2-8) to GND (CN2-6). Then make the following
       connections between the boards:

       UM232H       FT4232H-56Q
       ======       ===========
       GND        - GND
       D0 (CN2-1) - ECL (CN3-6)
       D1 (CN2-2) - EDA (CN3-7)
       D2 (CN2-3) - EDA (CN3-7)
       D3 (CN2-4) - ECS (CN3-5)

       NOTE: D1 & D2 are indeed both tied to the same EDA pin.

    """

    def __init__(self):
        self._mw = SpiController(cs_count=1,cs_act_hi=True)
        self._freq = 1E6
        self._mode = 0
        self._bidir = True
        
        # Maximum number of read cycles to wait while looking for the
        # Ready status after each write
        self._write_timeout_cnt = 25

        # According to the datasheet, the maximum write time is 6 ms
        self._Twc = 0.006
        
        # The opcodes are a full byte to make it easy to use with the
        # byte interface of SpiController. These opcodes also include
        # the start bit (SB), which is simply the left-most '1'
        # bit. The actual 2-bit opcode (OC) follows this start bit.
        #
        # The instructions ERAL, EWDS, EWEN and WRAL require a special
        # address byte to complete the opcode. So they are 2 element
        # lists whereas the others are single element lists.
        self._SBOC_erase = [0x07]
        self._SBOC_eral  = [0x04, 0x80] # requires EEPROM Vcc >= 4.5V
        self._SBOC_ewds  = [0x04, 0x00]
        self._SBOC_ewen  = [0x04, 0xc0]
        self._SBOC_read  = [0x06]
        self._SBOC_write = [0x05]
        self._SBOC_wral  = [0x04, 0x40] # requires EEPROM Vcc >= 4.5V
        
    def open(self):
        """Open an MicroWire connection to a slave"""
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:232h/1')
        self._mw.configure(url)

    def read_word(self, addr):
        """Read a single 16-bit word from the EEPROM starting at word address, addr"""
        
        # NOTE: Using SPI Mode 0. This really should have the FTDI
        # clock the read bits in on the rising edge, at least based on
        # my understanding of SPI. However, spi.py reads the bits on
        # the falling edge of the clock. For the 93LC56B, this is
        # exactly what we want. However, if spi.py ever gets changed,
        # will need to do writes and reads seperately with reads in
        # SPI Mode 1.
        port = self._mw.get_port(0, freq=self._freq,
                                  mode=self._mode, bidir=self._bidir)

        # byteswap() is to handle little endian data
        word = port.exchange(self._SBOC_read+[(addr&0xFF)],2)
        word[0],word[1] = word[1],word[0]
        return word.tobytes()

    def read_all(self, readlen):
        # NOTE: Using SPI Mode 0. This really should have the FTDI
        # clock the read bits in on the rising edge, at least base don
        # my understanding of SPI. However, spi.py reads the bits on
        # the falling edge of the clock. For the 93LC56B, this is
        # exactly what we want. However, if spi.py ever gets changed,
        # will need to do writes and reads seperately with reads in
        # SPI Mode 1.
        port = self._mw.get_port(0, freq=self._freq,
                                 mode=self._mode, bidir=self._bidir)
        
        # readlen is byte len so make sure it is an even number since
        # EEPROM is 16-bit device.
        if (readlen & 0x01):
            err = "readlen must be even - the EEPROM is a 16-bit device"
            raise SpiIOError(err)

        data = port.exchange(self._SBOC_read+[0x00], readlen)

        # byte swap to handle data in little endian (from:
        # https://stackoverflow.com/questions/36096292/
        #         efficient-way-to-swap-bytes-in-python)
        data[0::2], data[1::2] = data[1::2], data[0::2]

        #print('DATA: ', data)
        #words = spack('H'*(len(data)//2), data)
        
        return data.tobytes()

    def calc_eeprom_checksum(self, data):
        """Calculate EEPROM checksum over the data

           :param bytes data: data to compute checksum over. Must be
                              an even number of bytes to properly
                              compute checksum.
        """

        if isinstance(data, bytes):
            pass
        elif isinstance(data, array):
            data = data.tobytes()
        elif isinstance(data, list):
            data = bytes(data)
        else:
            raise SpiIOError("incompatible type for data")

        if (len(data) & 0x01):
            err = "data length must be even - the EEPROM is a 16-bit device"
            raise SpiIOError(err)

        # NOTE: checksum is computed using 16-bit values in little
        # endian ordering
        checksum = 0xAAAA
        for idx in range(0, len(data), 2):
            val = ((data[idx+1] << 8) + data[idx]) & 0x0ffff
            checksum = val^checksum
            checksum = (((checksum << 1) & 0x0ffff) |
                        ((checksum >> 15) & 0x0ffff))

        return checksum

    def write_word(self, addr, word):
        port = self._mw.get_port(0, freq=self._freq,
                                 mode=self._mode, bidir=self._bidir)

        # Must first enable Erase/Write
        port.exchange(self._SBOC_ewen)

        # Send the word, LSB first (little endian)
        port.exchange(self._SBOC_write+[(addr&0xFF),
                                        word & 0x0000ff,
                                        (word & 0x00ff00) >> 8])
            
        # Wait the write time
        sleep(self._Twc)
        
        # send a stop condition if sent at least 1 read with stop
        # False. Data is thrown away.
        status = port.read(1)
        print('Status: {}'.format(status))

        # Check the last bit of the last byte to make sure it is high
        # for Ready
        if ((status[-1] & 0x01) == 0x00):
            raise SpiIOError('ERROR: SPI Write never completed!')

        # Now disable Erase/Write since done with this write
        port.exchange(self._SBOC_ewds)
        

    # Write multiple bytes starting at byte address, addr. Length of
    # data must be a multiple of 2 since the EEPROM is 16-bits. So
    # extend data by 1 byte if this is not the case.
    def write(self, addr, data):

        if isinstance(data, bytes):
            pass
        elif isinstance(data, array):
            data = data.tobytes()
        elif isinstance(data, list):
            data = bytes(data)
        else:
            raise SpiIOError("incompatible type for data")

        # If addr is odd, raise an exception since it must be even
        if (addr & 0x01):
            err = "write addr must be even - the EEPROM is a 16-bit device"
            raise SpiIOError(err)
            
        wd_addr = (addr >> 1) # convert to word address

        # if the byte data is an odd number of bytes, force it to be
        # on 16-bit divisions
        if (len(data) & 0x01):
            err = "data length must be even - the EEPROM is a 16-bit device"
            raise SpiIOError(err)
        
        port = self._mw.get_port(0, freq=self._freq,
                                 mode=self._mode, bidir=self._bidir)

        # Must first enable Erase/Write
        port.exchange(self._SBOC_ewen)

        for idx in range(0, len(data), 2):
            # Send the word, MSB first
            port.exchange(self._SBOC_write+[(wd_addr&0xFF),
                                            data[idx+1],
                                            data[idx]])
            
            # Wait the write time
            sleep(self._Twc)
        
            # send a stop condition if sent at least 1 read with stop
            # False. Data is thrown away.
            status = port.read(1)

            # Check the last bit of the last byte to make sure it is
            # high for Ready
            if ((status[-1] & 0x01) == 0x00):
                print('ERROR: Last write never completed! Aborting!')
                break

            # increment to the next word address
            wd_addr += 1

        # Now disable Erase/Write since done with this write
        port.exchange(self._SBOC_ewds)
        

    def close(self):
        """Close the MicroWire connection"""
        self._mw.terminate()


class MicrowireTestCase(unittest.TestCase):
    """FTDI MicroWire driver test case

       Simple test to demonstrate MicroWire feature.

       Please ensure that the HW you connect to the FTDI port A does match
       the encoded configuration. GPIOs can be driven high or low, so check
       your HW setup before running this test as it might damage your HW.

       Do NOT run this test if you use FTDI port A as an UART or I2C
       bridge -or any unsupported setup!! You've been warned.
    """

    def test_microwire1(self):
        """Test reading EEPROM with read_word() and read_all()"""
        chksumAct = 0
        chksumExp = 1
        chksumSav = 2
        
        print()
        mw = MicrowireData93LC56BTest()
        mw.open()

        try:
            # Test read_word()
            data = b''
            for addr in range(0,256//2):
                data += mw.read_word(addr)                
            print(hexdump(data))

            # check that the right number of bytes were read
            self.assertTrue(len(data) == 256)

            # Pull out actual checksum from EEPROM data
            chksumAct = (data[-1] << 8) | data[-2]

            # compute expected checksum value over the EEPROM
            # contents, except the EEPROM word
            chksumExp = mw.calc_eeprom_checksum(data[:-2])

            print('Checksum Actual: 0x{:04x} Expected: 0x{:04x}'
                  .format(chksumAct,chksumExp))

        except FtdiError:
            self.assertTrue(chksumAct == chksumExp)
        except SpiIOError:
            self.assertTrue(chksumAct == chksumExp)
        else:        
            self.assertTrue(chksumAct == chksumExp)

        mw.close()

        # save checksum so can be compared when data is read with
        # read_all()
        chksumSav = chksumAct

        # reset chksumAct & chksumExp
        chksumAct = 0
        chksumExp = 1
        
        print()
        mw = MicrowireData93LC56BTest()
        mw.open()

        try:
            # Test read_all()
            data = mw.read_all(256)
            print(hexdump(data))

            # check that the right number of bytes were read
            self.assertTrue(len(data) == 256)

            # Pull out actual checksum from EEPROM data
            chksumAct = (data[-1] << 8) | data[-2]

            # compute expected checksum value over the EEPROM
            # contents, except the EEPROM word
            chksumExp = mw.calc_eeprom_checksum(data[:-2])

            print('Checksum Actual: 0x{:04x} Expected: 0x{:04x}'
                  .format(chksumAct,chksumExp))

        except FtdiError:
            self.assertTrue(chksumAct == chksumExp)
        except SpiIOError:
            self.assertTrue(chksumAct == chksumExp)
        else:        
            self.assertTrue(chksumAct == chksumExp)
            self.assertTrue(chksumAct == chksumSav)

        mw.close()
            
def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(MicrowireTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    testmod(modules[__name__])
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'info').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError('Invalid log level: %s', level)
    FtdiLogger.set_level(loglevel)
    unittest.main(defaultTest='suite')
