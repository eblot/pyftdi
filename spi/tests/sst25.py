#!/usr/bin/env python

from pyftdi import Ftdi
from pyftdi.spi import SpiController
from pyftdi.misc import hexdump
import struct
import time
import sys


def read_jedec_id(spi):
    CMD_JEDEC_ID = 0x9F
    jedec_cmd = struct.pack('<B', CMD_JEDEC_ID)
    return spi.command(jedec_cmd, 3)

def read_data(spi, address, length):
    CMD_READ = 0x03
    read_cmd = struct.pack('<BBBB', CMD_READ,
                           (address>>16)&0xff, (address>>8)&0xff, address&0xff)
    return spi.command(read_cmd, length)

def read_status(spi):
    CMD_RDSR = 0x05
    read_cmd = struct.pack('<B', CMD_RDSR)
    return struct.unpack('<B', spi.command(read_cmd, 1))[0]

ST25_BUSY = 0b00000001
ST25_WEL = 0b00000010
ST25_BP = 0b00111100
ST25_AAI = 0b01000000
ST25_BPL = 0b10000000

def is_busy(status):
    return (status & ST25_BUSY) and True or False

def is_wren(status):
    return (status & ST25_WEL) and True or False

def enable_write(spi):
    CMD_WREN = 0x06
    wren_cmd = struct.pack('<B', CMD_WREN)
    spi.command(wren_cmd)

def disable_write(spi):
    CMD_WRDI = 0x04
    wrdi_cmd = struct.pack('<B', CMD_WRDI)
    spi.command(wrdi_cmd)

def unlock(spi):
    CMD_EWSR = 0x50
    ewsr_cmd = struct.pack('<B', CMD_EWSR)
    spi.command(ewsr_cmd)
    CMD_WRSR = 0x01
    wrsr_cmd = struct.pack('<BB', CMD_WRSR, (~ST25_BP)&0xff)
    spi.command(wrsr_cmd)

def write_data(spi, address, data):
    if isinstance(data, str):
        data = [ord(x) for x in data]
    length = len(data)
    if (address&0x1) or (length&0x1) or (length==0):
        raise AssertionError("Alignement/size not supported")
    enable_write(spi)
    CMD_AAI = 0xAD
    aai_cmd = struct.pack('<BBBBBB', CMD_AAI,
                          (address>>16)&0xff,
                          (address>>8)&0xff,
                          address&0xff,
                          data.pop(0), data.pop(0))
    offset = 0
    percent = 0.0
    while True:
        percent = (1000.0*offset/length)
        print "Address %06x (%2.1f%%)\r" % (address + offset, percent/10),
        offset += 2
        spi.command(aai_cmd)
        while is_busy(read_status(spi)):
            time.sleep(0.01)
        if not data:
            break
        aai_cmd = struct.pack('<BBB', CMD_AAI, data.pop(0), data.pop(0))
    print ""
    disable_write(spi)

def erase_sector(spi, address):
    enable_write(spi)
    CMD_ERBLK = 0xD8
    erblk_cmd = struct.pack('<BBBB', CMD_ERBLK,
                            (address>>16)&0xff,
                            (address>>8)&0xff,
                            address&0xff)
    spi.command(erblk_cmd)
    while is_busy(read_status(spi)):
        time.sleep(0.030)
    disable_write(spi)

if __name__ == '__main__':
    spi_ctrl = SpiController()
    spi_ctrl.configure(vendor=0x403, product=0x6011, interface=0)
    spi = spi_ctrl.get_port(0)

    print "Identify"
    start_count = 0
    while True:
        jedec_id = read_jedec_id(spi)
        if jedec_id == '\xbf\x25\x4a':
            print "Found STT25 4MB flash"
            break
        print "%d: DATA %s" % (start_count, hexdump(jedec_id))
        time.sleep(0.5)
        start_count += 1
        if start_count >= 3:
            raise AssertionError("Unable to identify flash device")

    print "Unlock"
    unlock(spi)

    ref = ''.join([chr(x&0xff) for x in xrange(1<<16)])

    for sector in [x<<16 for x in xrange(0,(4<<20)>>16)]:
    #for sector in (0,):
        print "Write sector %d" % sector
        sys.stdout.flush()
        write_data(spi, sector, ref)
        print "Read back"
        stored = read_data(spi, sector, len(ref))
        if ref != stored:
            print "Comparison mismatch"
            print hexdump(ref)
            print hexdump(stored)
        else:
            print "Sector Ok"
        print "Erase sector"
        erase_sector(spi, sector)


    #CMD_READ_SECURITY = 0x77
    #sec_cmd = struct.pack('<BBBB', CMD_READ_SECURITY, 0, 0 ,0)
    #while True:
    #    data = spi.command( sec_cmd, 128)
    #    print "%d DATA %s" % hexdump(data)
    #    time.sleep(0.5)
