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

import unittest
import sys
from os import environ
from pyftdi.ftdi import Ftdi, FtdiError
from pyftdi.misc import hexdump
from time import sleep


class EepromTest(object):
    """
    """

    def __init__(self):
        self._ftdi = None

    def open(self):
        """Open a connection to the FTDI, defining which pins are configured as
           output and input"""

        # out_pins value of 0x00 means all inputs
        out_pins = 0x00
        
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:2232h/1')

        try:
            ftdi = Ftdi()
            ## If you REALLY muck things up, need to use this open_bitbang() function directly and enter vendor and product ID:
            # ftdi.open_bitbang(vendor=0x0403, product=0x6011, direction=out_pins)
            ftdi.open_bitbang_from_url(url, direction=out_pins)
            self._ftdi = ftdi
        except IOError as e:
            raise IOError ('Unable to open USB port: %s' % str(e))


    def close(self):
        """Close the FTDI connection"""
        if self._ftdi:
            self._ftdi.close()
            self._ftdi = None

    def read_eeprom(self, addr=0, length=Ftdi.EEPROM_MAX_SIZE):
        """Pass through function for Ftdi Class read_eeprom:

           Read the EEPROM starting at byte address, addr, and returning 
           length bytes. Here, addr and length are in bytes but we access 
           a 16-bit word at a time so update addr and length to work with 
           word accesses.

           :param int addr: byte address that desire to read - nearest word will be read
           :param int length: byte length to read - nearest word boundary will be read
           :return: eeprom bytes, as an array of bytes
           :rtype: array
        """
        return self._ftdi.read_eeprom(addr,length)

    def calc_eeprom_checksum(self, data):
        """Pass through function for Ftdi Class calc_eeprom_checksum:

           Calculate EEPROM checksum over the data

           :param bytes data: data to compute checksum over. Must be
                              an even number of bytes to properly
                              compute checksum.
        """
        return self._ftdi.calc_eeprom_checksum(data)

    def write_eeprom(self,addr,data,eeprom_sz=Ftdi.EEPROM_MAX_SIZE):
        """Pass through function for Ftdi Class write_eeprom:

           Write multiple bytes starting at byte address, addr. Length of
           data must be a multiple of 2 since the EEPROM is 16-bits. So
           extend data by 1 byte if this is not the case.

           WARNING: Writing to the EEPROM can cause very UNDESIRED
           effects if the wrong value is written in the wrong
           place. You can even essentially BRICK your FTDI device. Use
           this function only with EXTREME caution.

           If using a Hi-Speed Mini Module and you brick for FTDI
           device, see
           http://www.ftdichip.com/Support/Documents/AppNotes/AN_136%20Hi%20Speed%20Mini%20Module%20EEPROM%20Disaster%20Recovery.pdf

           :param int addr: starting byte address to start writing
           :param bytes data: data to be written
        """
        self._ftdi.write_eeprom(addr, data, eeprom_sz)


class EepromTestCase(unittest.TestCase):
    """FTDI EEPROM access method test case"""

    def test_eeprom_read(self):
        """Simple test to demonstrate can read EEPROM by checking its checksum.
        """

        eeprom = EepromTest()
        eeprom.open()

        try:
            data = eeprom.read_eeprom()
            print(hexdump(data))

            # check that the right number of bytes were read
            self.assertTrue(len(data) == Ftdi.EEPROM_MAX_SIZE)
            
            # Pull out actual checksum from EEPROM data
            chksumAct = (data[-1] << 8) | data[-2]

            # compute expected checksum value over the EEPROM contents, except the EEPROM word
            chksumExp = eeprom.calc_eeprom_checksum(data[:-2])

            print('Checksum Actual: 0x{:04x} Expected: 0x{:04x}'.format(chksumAct,chksumExp))

        except FtdiError:
            self.assertFalse(chksumAct == chksumExp)
        else:
            self.assertTrue(chksumAct == chksumExp)
        
        eeprom.close()

def suite():
    suite_ = unittest.TestSuite()
    suite_.addTest(unittest.makeSuite(EepromTestCase, 'test'))
    return suite_


if __name__ == '__main__':
    import doctest
    doctest.testmod(sys.modules[__name__])
    unittest.main(defaultTest='suite')
