# -*- coding: utf-8 -*-

import unittest

def suite():
    import pyftdi.tests.ftdi
    suite = unittest.TestSuite()
    suite.addTest(pyftdi.tests.ftdi.suite())
    return suite

if __name__ == '__main__':
    unittest.main(defaultTest='suite')
