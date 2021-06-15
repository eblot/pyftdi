#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020-2021, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=no-self-use
#pylint: disable-msg=invalid-name
#pylint: disable-msg=global-statement
#pylint: disable-msg=too-many-locals


import logging
from collections import defaultdict
from contextlib import redirect_stdout
from doctest import testmod
from io import StringIO
from os import environ
from string import ascii_letters
from sys import modules, stdout, version_info
from unittest import TestCase, TestSuite, makeSuite, main as ut_main
from urllib.parse import urlsplit
from pyftdi import FtdiLogger
from pyftdi.eeprom import FtdiEeprom
from pyftdi.ftdi import Ftdi, FtdiMpsseError
from pyftdi.gpio import GpioController
from pyftdi.misc import to_bool
from pyftdi.serialext import serial_for_url
from pyftdi.usbtools import UsbTools

# MockLoader is assigned in ut_main
MockLoader = None


class FtdiTestCase(TestCase):
    """Common features for all tests.
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        cls.debug = to_bool(environ.get('FTDI_DEBUG', 'off'), permissive=False)

    @classmethod
    def tearDownClass(cls):
        if cls.loader:
            cls.loader.unload()

    def setUp(self):
        if self.debug:
            print('.'.join(self.id().split('.')[-2:]))


class MockUsbToolsTestCase(FtdiTestCase):
    """Test UsbTools APIs.
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ftmany.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_enumerate(self):
        """Enumerate FTDI devices."""
        ftdis = [(0x403, pid)
                 for pid in (0x6001, 0x6010, 0x6011, 0x6014, 0x6015)]
        count = len(UsbTools.find_all(ftdis))
        self.assertEqual(count, 6)

    def test_device(self):
        """Access and release FTDI device."""
        ftdis = [(0x403, 0x6001)]
        ft232rs = UsbTools.find_all(ftdis)
        self.assertEqual(len(ft232rs), 1)
        devdesc, ifcount = ft232rs[0]
        self.assertEqual(ifcount, 1)
        dev = UsbTools.get_device(devdesc)
        self.assertIsNotNone(dev)
        UsbTools.release_device(dev)

    def test_string(self):
        """Retrieve a string from its identifier."""
        ftdis = [(0x403, 0x6010)]
        ft2232h = UsbTools.find_all(ftdis)[0]
        devdesc, _ = ft2232h
        dev = UsbTools.get_device(devdesc)
        serialn = UsbTools.get_string(dev, dev.iSerialNumber)
        self.assertEqual(serialn, 'FT2DEF')

    def test_list_devices(self):
        """List FTDI devices."""
        vid = 0x403
        vids = {'ftdi': vid}
        pids = {
            vid: {
                '230x': 0x6015,
                '232r': 0x6001,
                '232h': 0x6014,
                '2232h': 0x6010,
                '4232h': 0x6011,
            }
        }
        devs = UsbTools.list_devices('ftdi:///?', vids, pids, vid)
        self.assertEqual(len(devs), 6)
        ifmap = {
            0x6001: 1,
            0x6010: 2,
            0x6011: 4,
            0x6014: 1,
            0x6015: 1
        }
        for dev, desc in devs:
            strings = UsbTools.build_dev_strings('ftdi', vids, pids,
                                                 [(dev, desc)])
            self.assertEqual(len(strings), ifmap[dev.pid])
            for url, _ in strings:
                parts, _ = UsbTools.parse_url(url, 'ftdi', vids, pids, vid)
                self.assertEqual(parts.vid, dev.vid)
                self.assertEqual(parts.pid, dev.pid)
                self.assertEqual(parts.bus, dev.bus)
                self.assertEqual(parts.address, dev.address)
                self.assertEqual(parts.sn, dev.sn)
        devs = UsbTools.list_devices('ftdi://:232h/?', vids, pids, vid)
        self.assertEqual(len(devs), 2)
        devs = UsbTools.list_devices('ftdi://:2232h/?', vids, pids, vid)
        self.assertEqual(len(devs), 1)


class MockFtdiDiscoveryTestCase(FtdiTestCase):
    """Test FTDI device discovery APIs.
       These APIs are FTDI wrappers for UsbTools APIs.
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ftmany.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_list_devices(self):
        """List FTDI devices."""
        devs = Ftdi.list_devices('ftdi:///?')
        self.assertEqual(len(devs), 6)
        devs = Ftdi.list_devices('ftdi://:232h/?')
        self.assertEqual(len(devs), 2)
        devs = Ftdi.list_devices('ftdi://:2232h/?')
        self.assertEqual(len(devs), 1)
        devs = Ftdi.list_devices('ftdi://:4232h/?')
        self.assertEqual(len(devs), 1)
        out = StringIO()
        Ftdi.show_devices('ftdi:///?', out)
        lines = [l.strip() for l in out.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 10)
        portmap = defaultdict(int)
        reference = {'232': 1, '2232': 2, '4232': 4, '232h': 2, 'ft-x': 1}
        for line in lines:
            url = line.split(' ')[0].strip()
            parts = urlsplit(url)
            self.assertEqual(parts.scheme, 'ftdi')
            self.assertRegex(parts.path, r'^/[1-4]$')
            product = parts.netloc.split(':')[1]
            portmap[product] += 1
        self.assertEqual(portmap, reference)


class MockSimpleDeviceTestCase(FtdiTestCase):
    """Test FTDI APIs with a single-port FTDI device (FT232H)
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_enumerate(self):
        """Check simple enumeration of a single FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith('ftdi://'))
        # skip description, i.e. consider URL only
        self.assertTrue(lines[0].split(' ')[0].endswith('/1'))


class MockDualDeviceTestCase(FtdiTestCase):
    """Test FTDI APIs with two similar single-port FTDI devices (FT232H)
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft232h_x2.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_enumerate(self):
        """Check simple enumeration of a 2-port FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 2)
        for line in lines:
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            self.assertTrue(line.split(' ')[0].endswith('/1'))


class MockTwoPortDeviceTestCase(FtdiTestCase):
    """Test FTDI APIs with a dual-port FTDI device (FT2232H)
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft2232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_enumerate(self):
        """Check simple enumeration of a 4-port FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 2)
        for pos, line in enumerate(lines, start=1):
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            self.assertTrue(line.split(' ')[0].endswith(f'/{pos}'))


class MockFourPortDeviceTestCase(FtdiTestCase):
    """Test FTDI APIs with a quad-port FTDI device (FT4232H)
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft4232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_enumerate(self):
        """Check simple enumeration of two similar FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 4)
        for pos, line in enumerate(lines, start=1):
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            self.assertTrue(line.split(' ')[0].endswith(f'/{pos}'))


class MockManyDevicesTestCase(FtdiTestCase):
    """Test FTDI APIs with several, mixed type FTDI devices
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ftmany.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_enumerate(self):
        """Check simple enumeration of two similar FTDI device."""
        ftdi = Ftdi()
        temp_stdout = StringIO()
        with redirect_stdout(temp_stdout):
            self.assertRaises(SystemExit, ftdi.open_from_url, 'ftdi:///?')
        lines = [l.strip() for l in temp_stdout.getvalue().split('\n')]
        lines.pop(0)  # "Available interfaces"
        while lines and not lines[-1]:
            lines.pop()
        self.assertEqual(len(lines), 10)
        for line in lines:
            self.assertTrue(line.startswith('ftdi://'))
            # skip description, i.e. consider URL only
            url = line.split(' ')[0]
            urlparts = urlsplit(url)
            self.assertEqual(urlparts.scheme, 'ftdi')
            parts = urlparts.netloc.split(':')
            if parts[1] == '4232':
                # def file contains no serial number, so expect bus:addr syntax
                self.assertEqual(len(parts), 4)
                self.assertRegex(parts[2], r'^\d$')
                self.assertRegex(parts[3], r'^\d$')
            else:
                # other devices are assigned a serial number
                self.assertEqual(len(parts), 3)
                self.assertTrue(parts[2].startswith('FT'))
            self.assertRegex(urlparts.path, r'^/\d$')


class MockSimpleDirectTestCase(FtdiTestCase):
    """Test FTDI open/close APIs with a basic featured FTDI device (FT230H)
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_open_close(self):
        """Check simple open/close sequence."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self.assertEqual(ftdi.usb_path, (1, 1, 0))
        ftdi.close()

    def test_open_bitbang(self):
        """Check simple open/close BitBang sequence."""
        ftdi = Ftdi()
        ftdi.open_bitbang_from_url('ftdi:///1')
        ftdi.close()

    def test_open_mpsse(self):
        """Check simple MPSSE access."""
        ftdi = Ftdi()
        # FT230X is a pure UART bridge, MPSSE should not be available
        self.assertRaises(FtdiMpsseError,
                          ftdi.open_mpsse_from_url, 'ftdi:///1')


class MockSimpleMpsseTestCase(FtdiTestCase):
    """Test FTDI open/close APIs with a MPSSE featured FTDI device (FT232H)
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_open_close(self):
        """Check simple open/close sequence."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self.assertEqual(ftdi.usb_path, (4, 5, 0))
        ftdi.close()

    def test_open_bitbang(self):
        """Check simple open/close BitBang sequence."""
        ftdi = Ftdi()
        ftdi.open_bitbang_from_url('ftdi:///1')
        ftdi.close()

    def test_open_mpsse(self):
        """Check simple MPSSE access."""
        ftdi = Ftdi()
        ftdi.open_mpsse_from_url('ftdi:///1')
        ftdi.close()


class MockSimpleGpioTestCase(FtdiTestCase):
    """Test FTDI GPIO APIs
    """

    def tearDown(self):
        self.loader.unload()

    def _test_gpio(self):
        """Check simple GPIO write and read sequence."""
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        gpio = GpioController()
        # access to the virtual GPIO port
        out_pins = 0xAA
        gpio.configure('ftdi://:232h/1', direction=out_pins)
        bus, address, iface = gpio.ftdi.usb_path
        self.assertEqual((bus, address, iface), (4, 5, 0))
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        vport = vftdi.get_port(1)
        gpio.write_port(0xF3)
        self.assertEqual(vport.gpio, 0xAA & 0xF3)
        vport.gpio = 0x0c
        vio = gpio.read_port()
        self.assertEqual(vio, (0xAA & 0xF3) | (~0xAA & 0x0c))
        gpio.close()

    def test_baudrate(self):
        """Check simple GPIO write and read sequence."""
        # load custom CBUS config, with:
            # CBUS0: GPIO (gpio)
            # CBUS1: GPIO (gpio)
            # CBUS0: DRIVE1 (forced to high level)
            # CBUS0: TXLED  (eq. to highz for tests)
        with open('pyftdi/tests/resources/ft230x_io.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        gpio = GpioController()
        # access to the virtual GPIO port
        out_pins = 0xAA
        gpio.configure('ftdi://:230x/1', direction=out_pins)
        vftdi = self.loader.get_virtual_ftdi(1, 1)
        vftdi.get_port(1)
        baudrate = 1000000
        gpio.set_frequency(baudrate)
        gpio.close()


class MockSimpleUartTestCase(FtdiTestCase):
    """Test FTDI UART APIs
    """

    def tearDown(self):
        self.loader.unload()

    def test_uart(self):
        """Check simple TX/RX sequence."""
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        port = serial_for_url('ftdi:///1')
        bus, address, _ = port.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        vport = vftdi.get_port(1)
        msg = ascii_letters
        port.write(msg.encode())
        txd = vport[vport.UART_PINS.TXD]
        buf = txd.read(len(ascii_letters)+10).decode()
        self.assertEqual(msg, buf)
        msg = ''.join(reversed(msg))
        rxd = vport[vport.UART_PINS.TXD]
        rxd.write(msg.encode())
        buf = port.read(len(ascii_letters)).decode()
        self.assertEqual(msg, buf)
        port.close()

    def test_uart_loopback(self):
        """Check TXD/RXD loopback."""
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        port = serial_for_url('ftdi:///1')
        bus, address, _ = port.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        vport = vftdi.get_port(1)
        txd = vport[vport.UART_PINS.TXD]
        rxd = vport[vport.UART_PINS.RXD]
        txd.connect_to(rxd)
        msg = ascii_letters
        port.write(msg.encode())
        buf = port.read(len(ascii_letters)).decode()
        self.assertEqual(msg, buf)
        port.close()

    def test_baudrate_fs_dev(self):
        """Check baudrate settings for full speed devices."""
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        port = serial_for_url('ftdi:///1', baudrate=1200)
        bus, address, _ = port.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        vport = vftdi.get_port(1)
        self.assertRaises(ValueError, setattr, port, 'baudrate', 100)
        self.assertRaises(ValueError, setattr, port, 'baudrate', 3100000)
        for baudrate in (200, 600, 1200, 2400, 4800, 9600, 115200, 230400,
                         460800, 490000, 921600, 1000000, 1200000, 1500000,
                         2000000, 3000000):
            port.baudrate = baudrate
            #print(f'{baudrate} -> {port.ftdi.baudrate} -> {vport.baudrate}')
            self.assertEqual(port.ftdi.baudrate, vport.baudrate)
        port.close()

    def test_baudrate_hs_dev(self):
        """Check baudrate settings for high speed devices."""
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        port = serial_for_url('ftdi:///1', baudrate=1200)
        bus, address, _ = port.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        vport = vftdi.get_port(1)
        self.assertRaises(ValueError, setattr, port, 'baudrate', 100)
        self.assertRaises(ValueError, setattr, port, 'baudrate', 12100000)
        for baudrate in (200, 600, 1200, 2400, 4800, 9600, 115200, 230400,
                         460800, 490000, 921600, 1000000, 1200000, 1500000,
                         2000000, 3000000, 4000000, 6000000):
            port.baudrate = baudrate
            #print(f'{baudrate} -> {port.ftdi.baudrate} -> {vport.baudrate}')
            self.assertEqual(port.ftdi.baudrate, vport.baudrate)
        port.close()


class MockRawExtEepromTestCase(FtdiTestCase):
    """Test FTDI EEPROM low-level APIs with external EEPROM device
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def _restore_eeprom(self, ftdi):
        bus, address, _ = ftdi.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        data = self.loader.eeprom_backup
        size = len(vftdi.eeprom)
        if len(data) < size:
            data = bytearray(data) + bytes(size-len(data))
        vftdi.eeprom = bytes(data)

    def test_dump(self):
        """Check EEPROM full content."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self._restore_eeprom(ftdi)
        ref_data = bytes(list(range(256)))
        size = len(ref_data)
        data = ftdi.read_eeprom()
        self.assertEqual(len(data), size)
        self.assertEqual(ref_data, data)
        ftdi.close()

    def test_random_access_read(self):
        """Check EEPROM random read access."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self._restore_eeprom(ftdi)
        ref_data = bytes(list(range(256)))
        size = len(ref_data)
        # out of bound
        self.assertRaises(ValueError, ftdi.read_eeprom, size, 2)
        # last bytes
        buf = ftdi.read_eeprom(size-2, 2)
        self.assertEqual(buf[0:2], ref_data[-2:])
        self.assertEqual(buf[0:2], b'\xfe\xff')
        # out of bound
        self.assertRaises(ValueError, ftdi.read_eeprom, size-2, 4)
        # unaligned access
        buf = ftdi.read_eeprom(1, 2)
        self.assertEqual(buf[0:2], ref_data[1:3])
        self.assertEqual(buf[0:2], b'\x01\x02')
        # long read, unaligned access, unaligned size
        buf = ftdi.read_eeprom(43, 43)
        self.assertEqual(len(buf), 43)
        self.assertEqual(buf, ref_data[43:86])
        ftdi.close()

    def test_randow_access_write(self):
        """Check EEPROM random write access."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        bus, address, _ = ftdi.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        self._restore_eeprom(ftdi)
        checksum1 = vftdi.eeprom[-2:]
        orig_data = vftdi.eeprom[:8]
        ref_data = b'ABCD'
        ftdi.write_eeprom(0, ref_data, dry_run=False)
        checksum2 = vftdi.eeprom[-2:]
        # verify the data have been written
        self.assertEqual(vftdi.eeprom[:4], ref_data)
        # verify the data have not been overwritten
        self.assertEqual(vftdi.eeprom[4:8], orig_data[4:])
        # verify the checksum has been updated
        # TODO compute the expected checksum
        self.assertNotEqual(checksum1, checksum2)
        checksum1 = vftdi.eeprom[-2:]
        orig_data = vftdi.eeprom[:24]
        ftdi.write_eeprom(9, ref_data, dry_run=False)
        checksum2 = vftdi.eeprom[-2:]
        # verify the unaligned data have been written
        self.assertEqual(vftdi.eeprom[9:13], ref_data)
        # verify the data have not been overwritten
        self.assertEqual(vftdi.eeprom[:9], orig_data[:9])
        self.assertEqual(vftdi.eeprom[13:24], orig_data[13:])
        # verify the checksum has been updated
        self.assertNotEqual(checksum1, checksum2)
        checksum1 = vftdi.eeprom[-2:]
        orig_data = vftdi.eeprom[:48]
        ftdi.write_eeprom(33, ref_data[:3], dry_run=False)
        checksum2 = vftdi.eeprom[-2:]
        # verify the unaligned data have been written
        self.assertEqual(vftdi.eeprom[33:36], ref_data[:3])
        # verify the data have not been overwritten
        self.assertEqual(vftdi.eeprom[:33], orig_data[:33])
        self.assertEqual(vftdi.eeprom[36:48], orig_data[36:])
        # verify the checksum has been updated
        self.assertNotEqual(checksum1, checksum2)


class MockRawIntEepromTestCase(FtdiTestCase):
    """Test FTDI EEPROM low-level APIs with internal EEPROM device
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            cls.loader.load(yfp)

    def test_descriptor_update(self):
        """Check EEPROM content overrides YaML configuration."""
        # this test is more about testing the virtual FTDI infrastructure
        # than a pure PyFtdi test
        devs = Ftdi.list_devices('ftdi:///?')
        self.assertEqual(len(devs), 1)
        desc = devs[0][0]
        # these values are not the ones defined in YaML, but stored in EEPROM
        self.assertEqual(desc.sn, 'FT3KMGTL')
        self.assertEqual(desc.description, 'LC231X')

    def test_eeprom_read(self):
        """Check full read sequence."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        data = ftdi.read_eeprom()
        self.assertEqual(len(data), 0x400)
        ftdi.close()


class MockCBusEepromTestCase(FtdiTestCase):
    """Test FTDI EEPROM APIs that manage CBUS feature
    """

    def tearDown(self):
        self.loader.unload()

    def test_ft230x(self):
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        eeprom = FtdiEeprom()
        eeprom.open('ftdi:///1')
        # default EEPROM config does not have any CBUS configured as GPIO
        self.assertEqual(eeprom.cbus_pins, [])
        self.assertEqual(eeprom.cbus_mask, 0)
        # enable CBUS1 and CBUS3 as GPIO
        eeprom.set_property('cbus_func_1', 'gpio')
        eeprom.set_property('cbus_func_3', 'gpio')
        eeprom.sync()
        self.assertEqual(eeprom.cbus_pins, [1, 3])
        self.assertEqual(eeprom.cbus_mask, 0xA)
        # enable CBUS0 and CBUS2 as GPIO
        eeprom.set_property('cbus_func_0', 'gpio')
        eeprom.set_property('cbus_func_2', 'gpio')
        # not yet committed
        self.assertEqual(eeprom.cbus_pins, [1, 3])
        self.assertEqual(eeprom.cbus_mask, 0xA)
        eeprom.sync()
        # committed
        self.assertEqual(eeprom.cbus_pins, [0, 1, 2, 3])
        self.assertEqual(eeprom.cbus_mask, 0xF)
        # invalid CBUS pin
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func_4', 'gpio')
        # invalid pin function
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func_0', 'gpio_')
        # invalid pin
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func', 'gpio')
        # valid alternative mode
        eeprom.set_property('cbus_func_0', 'txled')
        eeprom.set_property('cbus_func_1', 'rxled')
        eeprom.sync()
        self.assertEqual(eeprom.cbus_pins, [2, 3])
        self.assertEqual(eeprom.cbus_mask, 0xC)
        eeprom.close()

    def test_ft232h(self):
        with open('pyftdi/tests/resources/ft232h_x2.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        eeprom = FtdiEeprom()
        eeprom.open('ftdi://::FT1ABC1/1', ignore=True)
        eeprom.erase()
        eeprom.initialize()
        # default EEPROM config does not have any CBUS configured as GPIO
        self.assertEqual(eeprom.cbus_pins, [])
        self.assertEqual(eeprom.cbus_mask, 0)
        eeprom.set_property('cbus_func_6', 'gpio')
        eeprom.set_property('cbus_func_9', 'gpio')
        # not yet committed
        self.assertEqual(eeprom.cbus_pins, [])
        self.assertEqual(eeprom.cbus_mask, 0)
        eeprom.sync()
        # committed
        self.assertEqual(eeprom.cbus_pins, [6, 9])
        self.assertEqual(eeprom.cbus_mask, 0xA)
        eeprom.set_property('cbus_func_5', 'gpio')
        eeprom.set_property('cbus_func_8', 'gpio')
        eeprom.sync()
        self.assertEqual(eeprom.cbus_pins, [5, 6, 8, 9])
        self.assertEqual(eeprom.cbus_mask, 0xF)
        # pin1 and pin3 is not configurable as GPIO
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func_1', 'gpio')
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func_3', 'gpio')
        eeprom.close()


class MockCbusGpioTestCase(FtdiTestCase):
    """Test FTDI CBUS GPIO APIs
    """

    def tearDown(self):
        self.loader.unload()

    def test_230x(self):
        """Check simple GPIO write and read sequence."""
        # load custom CBUS config, with:
            # CBUS0: GPIO (gpio)
            # CBUS1: GPIO (gpio)
            # CBUS0: DRIVE1 (forced to high level)
            # CBUS0: TXLED  (eq. to highz for tests)
        with open('pyftdi/tests/resources/ft230x_io.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self.assertEqual(ftdi.has_cbus, True)
        vftdi = self.loader.get_virtual_ftdi(1, 1)
        vport = vftdi.get_port(1)
        # CBUS0: in, CBUS1: out, CBUS2: in, CBUS3: out
        #   however, only CBUS0 and CBUS1 are mapped as GPIO,
        #   CBUS2 forced to 1 and CBUS3 not usable as IO
        #   even if use mask is 1111
        eeprom_mask = 0b0011
        eeprom_force = 0b0100
        cbus_mask = 0b1111
        cbus_dir = 0b1010
        ftdi.set_cbus_direction(cbus_mask, cbus_dir)
        cbus_out = 0b0011
        # CBUS0: 1, CBUS1: 1
        #   however, only CBUS1 is out, so CBUS0 output value should be ignored
        ftdi.set_cbus_gpio(cbus_out)
        exp_out = cbus_dir & cbus_out
        exp_out &= eeprom_mask
        exp_out |= eeprom_force
        vcbus, vactive = vport.cbus
        self.assertEqual(vcbus, exp_out)
        self.assertEqual(vactive, eeprom_mask | eeprom_force)
        cbus_out = 0b0000
        ftdi.set_cbus_gpio(cbus_out)
        exp_out = cbus_dir & cbus_out
        exp_out &= eeprom_mask
        exp_out |= eeprom_force
        vcbus, vactive = vport.cbus
        self.assertEqual(vcbus, exp_out)
        cbus_in = 0b0101
        vport.cbus = cbus_in
        cbus = ftdi.get_cbus_gpio()
        exp_in = cbus_in & eeprom_mask
        self.assertEqual(cbus, exp_in)
        ftdi.close()

    def test_lc231x(self):
        """Check simple GPIO write and read sequence."""
         # load custom CBUS config, with:
            # CBUS0: GPIO (gpio)
            # CBUS1: TXLED
            # CBUS2: DRIVE0 (to light up RX green led)
            # CBUS3: GPIO (gpio)
            # only CBUS0 and CBUS3 are available on LC231X
            # CBUS1 is connected to TX led, CBUS2 to RX led
        with open('pyftdi/tests/resources/ft231x_cbus.yaml', 'rb') as yfp:
            self.loader.load(yfp)
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self.assertEqual(ftdi.has_cbus, True)
        vftdi = self.loader.get_virtual_ftdi(1, 1)
        vport = vftdi.get_port(1)
        # CBUS0: in, CBUS1: out, CBUS2: in, CBUS3: out
        #   however, only CBUS0 and CBUS3 are mapped as GPIO,
        #   CBUS1 not usable as IO, CBUS2 is fixed to low
        #   even if use mask is 1111
        eeprom_mask = 0b1001
        eeprom_force_low = 0b0100
        cbus_mask = 0b1111
        cbus_dir = 0b1010
        ftdi.set_cbus_direction(cbus_mask, cbus_dir)
        cbus_out = 0b1111
        # however, only CBUS0 & 3 are out, so CBUS1/CBUS2 should be ignored
        ftdi.set_cbus_gpio(cbus_out)
        exp_out = cbus_dir & cbus_out
        exp_out &= eeprom_mask
        vcbus, vactive = vport.cbus
        self.assertEqual(vcbus, exp_out)
        self.assertEqual(vactive, eeprom_mask | eeprom_force_low)
        cbus_out = 0b0000
        ftdi.set_cbus_gpio(cbus_out)
        exp_out = cbus_dir & cbus_out
        exp_out &= eeprom_mask
        vcbus, vactive = vport.cbus
        self.assertEqual(vcbus, exp_out)
        cbus_in = 0b0101
        vport.cbus = cbus_in
        cbus = ftdi.get_cbus_gpio()
        exp_in = cbus_in & eeprom_mask
        self.assertEqual(cbus, exp_in)
        ftdi.close()


def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(MockUsbToolsTestCase, 'test'))
    suite_.addTest(makeSuite(MockFtdiDiscoveryTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockDualDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockTwoPortDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockFourPortDeviceTestCase, 'test'))
    suite_.addTest(makeSuite(MockManyDevicesTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleDirectTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleMpsseTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleGpioTestCase, 'test'))
    suite_.addTest(makeSuite(MockSimpleUartTestCase, 'test'))
    suite_.addTest(makeSuite(MockRawExtEepromTestCase, 'test'))
    suite_.addTest(makeSuite(MockRawIntEepromTestCase, 'test'))
    suite_.addTest(makeSuite(MockCBusEepromTestCase, 'test'))
    suite_.addTest(makeSuite(MockCbusGpioTestCase, 'test'))
    return suite_


def main():
    testmod(modules[__name__])
    debug = to_bool(environ.get('FTDI_DEBUG', 'off'))
    if debug:
        formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-7s'
                                      ' %(name)-18s [%(lineno)4d] %(message)s',
                                      '%H:%M:%S')
    else:
        formatter = logging.Formatter('%(message)s')
    level = environ.get('FTDI_LOGLEVEL', 'warning').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError as exc:
        raise ValueError(f'Invalid log level: {level}') from exc
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    FtdiLogger.set_level(loglevel)
    FtdiLogger.set_formatter(formatter)
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('backend.usbvirt', )
    # Ensure the virtual backend can be found and is loaded
    backend = UsbTools.find_backend()
    try:
        # obtain the loader class associated with the virtual backend
        global MockLoader
        MockLoader = backend.create_loader()
    except AttributeError as exc:
        raise AssertionError('Cannot load virtual USB backend') from exc
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
