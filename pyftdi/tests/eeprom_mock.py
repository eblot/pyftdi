#!/usr/bin/env python3
# Copyright (c) 2019-2021, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# mock eeprom tests that can be run in CI without a device connected
import logging
from os import environ
from sys import modules, stdout
from unittest import TestCase, TestSuite, SkipTest, makeSuite, main as ut_main
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.eeprom import FtdiEeprom
from pyftdi.misc import to_bool, hexdump
from pyftdi.ftdi import FtdiError

VirtLoader = None


class FtdiTestCase:
    """Common features for all tests.
    """

    # manufacturer/product/serial number strings to use in tests
    TEST_MANU_NAME = "MNAME"
    TEST_PROD_NAME = "PNAME"
    TEST_SN = "SN123"
    TEST_CONFIG_FILENAME = ''

    @classmethod
    def setUpClass(cls):
        cls.debug = to_bool(environ.get('FTDI_DEBUG', 'off'), permissive=False)
        cls.url = environ.get('FTDI_DEVICE', 'ftdi:///1')
        cls.loader = None

    @classmethod
    def tearDownClass(cls):
        if cls.loader:
            cls.loader.unload()

    def setUp(self):
        pass


class EepromMirrorTestCase(FtdiTestCase):
    """Test FTDI EEPROM mirror feature (duplicate eeprom data over 2 eeprom
    sectors). Generally this is tested with a virtual eeprom (by setting
    environment variable FTDI_VIRTUAL=on), however you may also test with an
    actual device at your own risk. Note that none of the tests should
    commit any of their eeprom changes
    """

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        if VirtLoader:
            cls.loader = VirtLoader()
            with open(cls.TEST_CONFIG_FILENAME, 'rb') as yfp:
                cls.loader.load(yfp)
        if cls.url == 'ftdi:///1':
            ftdi = Ftdi()
            ftdi.open_from_url(cls.url)
            count = ftdi.device_port_count
            ftdi.close()

    def test_mirror_properties(self):
        """Check FtdiEeprom properties are accurate for a device that can
            mirror
        """
        # properties should work regardless of if the mirror option is set
        # or not
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.open(self.url, ignore=True)
        self.assertTrue(eeprom.has_mirroring)
        self.assertFalse(eeprom.is_mirroring_enabled)
        self.assertEqual(eeprom.size // 2, eeprom.mirror_sector)
        eeprom.close()

        mirrored_eeprom = FtdiEeprom()
        mirrored_eeprom.enable_mirroring(True)
        mirrored_eeprom.open(self.url, ignore=True)
        self.assertTrue(mirrored_eeprom.has_mirroring)
        self.assertTrue(mirrored_eeprom.is_mirroring_enabled)
        self.assertEqual(mirrored_eeprom.size // 2,
            mirrored_eeprom.mirror_sector)
        mirrored_eeprom.close()

    def test_mirror_manufacturer(self):
        """Verify manufacturer string is properly duplicated across the 2
            eeprom sectors
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(True)
        eeprom.open(self.url, ignore=True)
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        self._check_for_mirrored_eeprom_contents(eeprom)

    def test_mirror_product(self):
        """Verify product string is properly duplicated across the 2 eeprom
            sectors
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(True)
        eeprom.open(self.url, ignore=True)
        eeprom.erase()
        eeprom.set_product_name(self.TEST_PROD_NAME)
        self._check_for_mirrored_eeprom_contents(eeprom)

    def test_mirror_serial(self):
        """Verify serial string is properly duplicated across the 2 eeprom
            sectors
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(True)
        eeprom.open(self.url, ignore=True)
        eeprom.erase()
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_mirrored_eeprom_contents(eeprom)

    def test_varstr_combinations(self):
        """Verify various combinations of var strings are properly duplicated
            across the 2 eeprom sectors
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(True)
        eeprom.open(self.url, ignore=True)

        # manu + prod str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        eeprom.set_product_name(self.TEST_PROD_NAME)
        self._check_for_mirrored_eeprom_contents(eeprom)

        # manu + sn str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_mirrored_eeprom_contents(eeprom)

        # prod + sn str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_PROD_NAME)
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_mirrored_eeprom_contents(eeprom)

        # manu + prod + sn str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        eeprom.set_manufacturer_name(self.TEST_PROD_NAME)
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_mirrored_eeprom_contents(eeprom)

    def test_compute_size_detects_mirror(self):
        """Verify the eeproms internal _compute_size method
            returns the correct bool value when it detects an eeprom mirror
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.open(self.url, ignore=True)
        _, mirrored = eeprom._compute_size([])
        self.assertFalse(mirrored)
        test_buf = bytearray(eeprom.size)
        sector_mid = eeprom.size // 2
        for ii in range(sector_mid):
            test_buf[ii] = ii % 255
            test_buf[sector_mid+ii] = test_buf[ii]
        _, mirrored = eeprom._compute_size(bytes(test_buf))
        self.assertTrue(mirrored)

        # change one byte and confirm failure
        test_buf[eeprom.size - 2] = test_buf[eeprom.size - 2] - 1
        _, mirrored = eeprom._compute_size(bytes(test_buf))
        self.assertFalse(mirrored)

    def _check_for_mirrored_eeprom_contents(self, eeprom: FtdiEeprom):
        """Check that contents of the eeprom is identical over the two
            sectors
        """
        sector_size = eeprom.size // 2
        for ii in range(0, sector_size):
            self.assertEqual(eeprom.data[ii],
                eeprom.data[ii + eeprom.mirror_sector],
                f'Mismatch mirror data @ 0x{ii:02x}: 0x{eeprom.data[ii]:02x} '
                f'!= 0x{eeprom.data[ii + eeprom.mirror_sector]:02x}')

class NonMirroredEepromTestCase(FtdiTestCase):
    """Test FTDI EEPROM mirror features do not break FTDI devices that do
    not use mirroring
    """
    TEST_MANU_NAME = "MNAME"
    TEST_PROD_NAME = "PNAME"
    TEST_SN = "SN123"

    @classmethod
    def setUpClass(cls):
        FtdiTestCase.setUpClass()
        if VirtLoader:
            cls.loader = VirtLoader()
            with open(cls.TEST_CONFIG_FILENAME, 'rb') as yfp:
                cls.loader.load(yfp)
        if cls.url == 'ftdi:///1':
            ftdi = Ftdi()
            ftdi.open_from_url(cls.url)
            count = ftdi.device_port_count
            ftdi.close()

    def test_mirror_properties(self):
        """Check FtdiEeprom properties are accurate for a device that can not
           mirror.
           Only run this test if the device under test is incapable of
           mirroring
        """
        if bool(getattr(self, 'DEVICE_CAN_MIRROR', None)):
            self.skipTest("Mirror properties for devices capable of mirroring"
                + " are tested in EepromMirrorTestCase")
        # properties should work regardless of if the mirror option is set
        # or not
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.open(self.url, ignore=True)
        self.assertFalse(eeprom.has_mirroring)
        self.assertFalse(eeprom.is_mirroring_enabled)
        with self.assertRaises(FtdiError):
            eeprom.mirror_sector
        eeprom.close()
        # even if mirroring is enabled, should still stay false
        mirrored_eeprom = FtdiEeprom()
        mirrored_eeprom.enable_mirroring(True)
        mirrored_eeprom.open(self.url, ignore=True)
        self.assertFalse(mirrored_eeprom.has_mirroring)
        self.assertFalse(mirrored_eeprom.is_mirroring_enabled)
        with self.assertRaises(FtdiError):
            mirrored_eeprom.mirror_sector
        mirrored_eeprom.close()

    def test_no_mirror_manufacturer(self):
        """Verify manufacturer string is NOT duplicated/mirrored
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(False)
        eeprom.open(self.url, ignore=True)
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        self._check_for_non_mirrored_eeprom_contents(eeprom)

    def test_no_mirror_product(self):
        """Verify product string is NOT duplicated/mirrored
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(False)
        eeprom.open(self.url, ignore=True)
        eeprom.erase()
        eeprom.set_product_name(self.TEST_PROD_NAME)
        self._check_for_non_mirrored_eeprom_contents(eeprom)

    def test_mirror_serial(self):
        """Verify serial string is NOT duplicated/mirrored
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(False)
        eeprom.open(self.url, ignore=True)
        eeprom.erase()
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_non_mirrored_eeprom_contents(eeprom)

    def test_varstr_combinations(self):
        """Verify various combinations of var strings are NOT
        duplicated/mirrored
        """
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.enable_mirroring(False)
        eeprom.open(self.url, ignore=True)

        # manu + prod str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        eeprom.set_product_name(self.TEST_PROD_NAME)
        self._check_for_non_mirrored_eeprom_contents(eeprom)

        # manu + sn str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_non_mirrored_eeprom_contents(eeprom)

        # prod + sn str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_PROD_NAME)
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_non_mirrored_eeprom_contents(eeprom)

        # manu + prod + sn str
        eeprom.erase()
        eeprom.set_manufacturer_name(self.TEST_MANU_NAME)
        eeprom.set_manufacturer_name(self.TEST_PROD_NAME)
        eeprom.set_serial_number(self.TEST_SN)
        self._check_for_non_mirrored_eeprom_contents(eeprom)

    def test_compute_size_does_not_mirror(self):
        """Verify the eeproms internal _compute_size method returns the correct
           bool value when it detects no mirroring.
        """
        if self.DEVICE_CAN_MIRROR:
            self.skipTest("Mirror properties for devices capable of mirroring"
                + " are tested in EepromMirrorTestCase")
        eeprom = FtdiEeprom()
        eeprom.set_test_mode(True)
        eeprom.open(self.url, ignore=True)
        _, mirrored = eeprom._compute_size([])
        self.assertFalse(mirrored)
        eeprom.close()

        eeprom = FtdiEeprom()
        eeprom.open(self.url, ignore=False)
        _, mirrored = eeprom._compute_size([])
        self.assertFalse(mirrored)
        eeprom.close()

    def _check_for_non_mirrored_eeprom_contents(self, eeprom: FtdiEeprom):
        """Check that contents of the eeprom is not mirrored
        """
        mirror_sector_start = eeprom.size // 2
        # split eeprom into 2 sectors as would be done if mirroring was enabled
        # and verify the device is not mirrored
        normal_mirror_s1 = eeprom.data[:mirror_sector_start]
        normal_mirror_s2 = eeprom.data[mirror_sector_start:]
        self.assertNotEqual(normal_mirror_s1, normal_mirror_s2)


class EepromMirrorFt232hTestCase(EepromMirrorTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft232h.yaml'


class EepromMirrorFt2232hTestCase(EepromMirrorTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft2232h.yaml'


class EepromMirrorFt4232hTestCase(EepromMirrorTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft4232h.yaml'


class EepromMirrorFt232rTestCase(NonMirroredEepromTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft232r.yaml'
    DEVICE_CAN_MIRROR = False


class EepromMirrorFt230xTestCase(NonMirroredEepromTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft230x.yaml'
    DEVICE_CAN_MIRROR = False


class EepromNonMirroredFt232hTestCase(NonMirroredEepromTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft232h.yaml'
    DEVICE_CAN_MIRROR = True


class EepromNonMirroredFt2232hTestCase(NonMirroredEepromTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft2232h.yaml'
    DEVICE_CAN_MIRROR = True


class EepromNonMirroredFt4232hTestCase(NonMirroredEepromTestCase, TestCase):
    TEST_CONFIG_FILENAME = 'pyftdi/tests/resources/ft4232h.yaml'
    DEVICE_CAN_MIRROR = True


def suite():
    suite_ = TestSuite()
    # Test devices that support the mirroring capability
    suite_.addTest(makeSuite(EepromMirrorFt232hTestCase, 'test'))
    suite_.addTest(makeSuite(EepromMirrorFt2232hTestCase, 'test'))
    suite_.addTest(makeSuite(EepromMirrorFt4232hTestCase, 'test'))
    # Test devices that do not support the mirror capability
    suite_.addTest(makeSuite(EepromMirrorFt232rTestCase, 'test'))
    suite_.addTest(makeSuite(EepromMirrorFt230xTestCase, 'test'))
    # test devices that support the mirroring capability, but have it disabled
    suite_.addTest(makeSuite(EepromNonMirroredFt232hTestCase, 'test'))
    suite_.addTest(makeSuite(EepromNonMirroredFt2232hTestCase, 'test'))
    suite_.addTest(makeSuite(EepromNonMirroredFt4232hTestCase, 'test'))
    return suite_


def virtualize():
    if not to_bool(environ.get('FTDI_VIRTUAL', 'off')):
        return
    from pyftdi.usbtools import UsbTools
    # Force PyUSB to use PyFtdi test framework for USB backends
    UsbTools.BACKENDS = ('backend.usbvirt', )
    # Ensure the virtual backend can be found and is loaded
    backend = UsbTools.find_backend()
    try:
        # obtain the loader class associated with the virtual backend
        global VirtLoader
        VirtLoader = backend.create_loader()
    except AttributeError:
        raise AssertionError('Cannot load virtual USB backend')


def setup_module():
    import doctest
    doctest.testmod(modules[__name__])
    debug = to_bool(environ.get('FTDI_DEBUG', 'off'))
    if debug:
        formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-7s'
                                      ' %(name)-20s [%(lineno)4d] %(message)s',
                                      '%H:%M:%S')
    else:
        formatter = logging.Formatter('%(message)s')
    level = environ.get('FTDI_LOGLEVEL', 'warning').upper()
    try:
        loglevel = getattr(logging, level)
    except AttributeError:
        raise ValueError(f'Invalid log level: {level}')
    FtdiLogger.log.addHandler(logging.StreamHandler(stdout))
    FtdiLogger.set_level(loglevel)
    FtdiLogger.set_formatter(formatter)
    virtualize()


def main():
    setup_module()
    try:
        ut_main(defaultTest='suite')
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    # Useful environment variables:
    #  FTDI_DEVICE: a specific FTDI URL, default to ftdi:///1
    #  FTDI_LOGLEVEL: a Logger debug level, to define log verbosity
    #  FTDI_DEBUG: to enable/disable debug mode
    #  FTDI_VIRTUAL: to use a virtual device rather than a physical device
    main()
