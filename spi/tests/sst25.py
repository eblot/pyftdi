#!/usr/bin/env python

from pyftdi import Ftdi
from pyftdi.spi import SpiController
from pyftdi.misc import hexdump
import struct
import time
import sys


if __name__ == '__main__':
    print "Test disabled, being rewritten with a more generic implementation"
    #spi_ctrl = SpiController()
    #spi_ctrl.configure(vendor=0x403, product=0x6011, interface=0)
    #spi = spi_ctrl.get_port(0)
    #
    #print "Identify"
    #start_count = 0
    #while True:
    #    jedec_id = read_jedec_id(spi)
    #    if jedec_id == '\xbf\x25\x4a':
    #        print "Found STT25 4MB flash"
    #        break
    #    print "%d: DATA %s" % (start_count, hexdump(jedec_id))
    #    time.sleep(0.5)
    #    start_count += 1
    #    if start_count >= 3:
    #        raise AssertionError("Unable to identify flash device")
    #
    #print "Unlock"
    #unlock(spi)
    #
    #ref = ''.join([chr(x&0xff) for x in xrange(1<<16)])
    #
    #for sector in [x<<16 for x in xrange(0,(4<<20)>>16)]:
    ##for sector in (0,):
    #    print "Write sector %d" % sector
    #    sys.stdout.flush()
    #    write_data(spi, sector, ref)
    #    print "Read back"
    #    stored = read_data(spi, sector, len(ref))
    #    if ref != stored:
    #        print "Comparison mismatch"
    #        print hexdump(ref)
    #        print hexdump(stored)
    #    else:
    #        print "Sector Ok"
    #    print "Erase sector"
    #    erase_sector(spi, sector)
    #
    #
    ##CMD_READ_SECURITY = 0x77
    ##sec_cmd = struct.pack('<BBBB', CMD_READ_SECURITY, 0, 0 ,0)
    ##while True:
    ##    data = spi.command( sec_cmd, 128)
    ##    print "%d DATA %s" % hexdump(data)
    ##    time.sleep(0.5)
