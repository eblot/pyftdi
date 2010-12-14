# -*- coding: utf-8 -*-

import unittest
from pyftdi.ftdi import Ftdi

class FtdiTestCase(unittest.TestCase):
    """FTDI driver test case"""

    def setUp(self):
        import sys
        if sys.platform in ('darwin', ):
            import os.path
            if os.path.isdir('/usr/local/homebrew/lib'):
                os.environ['DYLD_LIBRARY_PATH'] = '/usr/local/homebrew/lib'
    
    def test_multiple_interface(self):
        # the following calls used to create issues (several interfaces from
        # the same device)
        ftdi1 = Ftdi()
        ftdi1.open(interface=1)
        ftdi2 = Ftdi()
        ftdi2.open(interface=2)
        ftdi1.close()
        ftdi2.close()
    
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(FtdiTestCase, 'test'))
    return suite
