#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=no-self-use
#pylint: disable-msg=invalid-name
#pylint: disable-msg=global-statement

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
from pyftdi.misc import hexdump
from pyftdi.serialext import serial_for_url
from pyftdi.usbtools import UsbTools

# MockLoader is assigned in ut_main
MockLoader = None


# need support for f-string syntax
if version_info[:2] < (3, 6):
    raise AssertionError('Python 3.6 is required for this module')


class MockUsbToolsTestCase(TestCase):
    """Test UsbTools APIs.
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ftmany.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockFtdiDiscoveryTestCase(TestCase):
    """Test FTDI device discovery APIs.
       These APIs are FTDI wrappers for UsbTools APIs.
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ftmany.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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
        reference = {'232': 1, '2232': 2, '4232': 4, '232h': 2, '230x': 1}
        for line in lines:
            url = line.split(' ')[0].strip()
            parts = urlsplit(url)
            self.assertEqual(parts.scheme, 'ftdi')
            self.assertRegex(parts.path, r'^/[1-4]$')
            product = parts.netloc.split(':')[1]
            portmap[product] += 1
        self.assertEqual(portmap, reference)


class MockSimpleDeviceTestCase(TestCase):
    """Test FTDI APIs with a single-port FTDI device (FT232H)
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockDualDeviceTestCase(TestCase):
    """Test FTDI APIs with two similar single-port FTDI devices (FT232H)
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h_x2.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockTwoPortDeviceTestCase(TestCase):
    """Test FTDI APIs with a dual-port FTDI device (FT2232H)
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft2232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockFourPortDeviceTestCase(TestCase):
    """Test FTDI APIs with a quad-port FTDI device (FT4232H)
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft4232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockManyDevicesTestCase(TestCase):
    """Test FTDI APIs with several, mixed type FTDI devices
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ftmany.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockSimpleDirectTestCase(TestCase):
    """Test FTDI open/close APIs with a basic featured FTDI device (FT230H)
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockSimpleMpsseTestCase(TestCase):
    """Test FTDI open/close APIs with a MPSSE featured FTDI device (FT232H)
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockSimpleGpioTestCase(TestCase):
    """Test FTDI GPIO APIs
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test(self):
        """Check simple GPIO write and read sequence."""
        gpio = GpioController()
        # access to the virtual GPIO port
        out_pins = 0xAA
        gpio.configure('ftdi://:232h/1', direction=out_pins)
        bus, address, iface = gpio.ftdi.usb_path
        self.assertEqual((bus, address, iface), (4, 5, 0))
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        gpio.write_port(0xF3)
        self.assertEqual(vftdi.gpio, 0xAA & 0xF3)
        vftdi.gpio = 0x0c
        vio = gpio.read_port()
        self.assertEqual(vio, (0xAA & 0xF3) | (~0xAA & 0x0c))
        gpio.close()


class MockSimpleUartTestCase(TestCase):
    """Test FTDI UART APIs
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test(self):
        """Check simple TX/RX sequence."""
        port = serial_for_url('ftdi:///1')
        bus, address, _ = port.usb_path
        vftdi = self.loader.get_virtual_ftdi(bus, address)
        msg = ascii_letters
        port.write(msg.encode())
        buf = vftdi.uart_read(len(ascii_letters)+10).decode()
        self.assertEqual(msg, buf)
        msg = ''.join(reversed(msg))
        vftdi.uart_write(msg.encode())
        buf = port.read(len(ascii_letters)).decode()
        self.assertEqual(msg, buf)
        port.close()


class MockRawExtEepromTestCase(TestCase):
    """Test FTDI EEPROM low-level APIs with external EEPROM device
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockRawIntEepromTestCase(TestCase):
    """Test FTDI EEPROM low-level APIs with internal EEPROM device
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

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


class MockCBusEepromTestCase(TestCase):
    """Test FTDI EEPROM APIs that manage CBUS feature
    """

    def test_ft230x(self):
        loader = MockLoader()
        with open('pyftdi/tests/resources/ft230x.yaml', 'rb') as yfp:
            loader.load(yfp)
        UsbTools.flush_cache()
        eeprom = FtdiEeprom()
        eeprom.open('ftdi:///1')
        # default EEPROM config does not have any CBUS configured as GPIO
        self.assertEqual(eeprom.cbus_pins, [])
        self.assertEqual(eeprom.cbus_mask, 0)
        # enable CBUS1 and CBUS3 as GPIO
        eeprom.set_property('cbus_func_1', 'iomode')
        eeprom.set_property('cbus_func_3', 'iomode')
        eeprom.sync()
        self.assertEqual(eeprom.cbus_pins, [1, 3])
        self.assertEqual(eeprom.cbus_mask, 0xA)
        # enable CBUS0 and CBUS2 as GPIO
        eeprom.set_property('cbus_func_0', 'iomode')
        eeprom.set_property('cbus_func_2', 'iomode')
        # not yet committed
        self.assertEqual(eeprom.cbus_pins, [1, 3])
        self.assertEqual(eeprom.cbus_mask, 0xA)
        eeprom.sync()
        # committed
        self.assertEqual(eeprom.cbus_pins, [0, 1, 2, 3])
        self.assertEqual(eeprom.cbus_mask, 0xF)
        # invalid CBUS pin
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func_4', 'iomode')
        # invalid pin function
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func_0', 'iomode_')
        # invalid pin
        self.assertRaises(ValueError, eeprom.set_property,
                          'cbus_func', 'iomode')
        # valid alternative mode
        eeprom.set_property('cbus_func_0', 'txled')
        eeprom.set_property('cbus_func_1', 'rxled')
        eeprom.sync()
        self.assertEqual(eeprom.cbus_pins, [2, 3])
        self.assertEqual(eeprom.cbus_mask, 0xC)
        eeprom.close()
        loader.unload()

    def test_ft232h(self):
        loader = MockLoader()
        with open('pyftdi/tests/resources/ft232h_x2.yaml', 'rb') as yfp:
            loader.load(yfp)
        UsbTools.flush_cache()
        eeprom = FtdiEeprom()
        eeprom.open('ftdi://::FT1ABC1/1', ignore=True)
        eeprom.erase()
        eeprom.initialize()
        # default EEPROM config does not have any CBUS configured as GPIO
        self.assertEqual(eeprom.cbus_pins, [])
        self.assertEqual(eeprom.cbus_mask, 0)
        eeprom.set_property('cbus_func_1', 'iomode')
        eeprom.set_property('cbus_func_3', 'iomode')
        eeprom.sync()
        # CBUS1 and CBUS3 are not addressable as GPIOs
        # they should appear in cbus_pins, but not in cbus_mask
        self.assertEqual(eeprom.cbus_pins, [1, 3])
        self.assertEqual(eeprom.cbus_mask, 0)
        eeprom.set_property('cbus_func_6', 'iomode')
        eeprom.set_property('cbus_func_9', 'iomode')
        # not yet committed
        self.assertEqual(eeprom.cbus_pins, [1, 3])
        self.assertEqual(eeprom.cbus_mask, 0)
        eeprom.sync()
        # committed
        self.assertEqual(eeprom.cbus_pins, [1, 3, 6, 9])
        self.assertEqual(eeprom.cbus_mask, 0xA)
        eeprom.set_property('cbus_func_5', 'iomode')
        eeprom.set_property('cbus_func_8', 'iomode')
        eeprom.sync()
        self.assertEqual(eeprom.cbus_pins, [1, 3, 5, 6, 8, 9])
        self.assertEqual(eeprom.cbus_mask, 0xF)
        eeprom.close()
        loader.unload()


class MockCbusGpioTestCase(TestCase):
    """Test FTDI CBUS GPIO APIs
    """

    @classmethod
    def setUpClass(cls):
        cls.loader = MockLoader()
        with open('pyftdi/tests/resources/ft230x_io.yaml', 'rb') as yfp:
            cls.loader.load(yfp)
        UsbTools.flush_cache()

    @classmethod
    def tearDownClass(cls):
        cls.loader.unload()

    def test_simple(self):
        """Check simple GPIO write and read sequence."""
        ftdi = Ftdi()
        ftdi.open_from_url('ftdi:///1')
        self.assertEqual(ftdi.has_cbus, True)
        vftdi = self.loader.get_virtual_ftdi(1, 1)
        ftdi.set_cbus_direction(0xf, 0xa)
        ftdi.set_cbus_gpio(0x3)
        self.assertEqual(vftdi.cbus, 0xa & 0x3)
        vftdi.cbus = 0x5
        cbus = ftdi.get_cbus_gpio()
        self.assertEqual(cbus, 0x5)
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
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    level = environ.get('FTDI_LOGLEVEL', 'warning').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError(f'Invalid log level: {level}')
    FtdiLogger.set_level(loglevel)
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('backend.usbvirt', )
    # Ensure the virtual backend can be found and is loaded
    backend = UsbTools.find_backend()
    try:
        # obtain the loader class associated with the virtual backend
        global MockLoader
        MockLoader = backend.create_loader()
    except AttributeError:
        raise AssertionError('Cannot load virtual USB backend')
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
