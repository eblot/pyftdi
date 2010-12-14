#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import unittest

def suite():
    import pyftdi.tests
    suite = unittest.TestSuite()
    suite.addTest(pyftdi.tests.suite())
    return suite

if __name__ == '__main__':
    import doctest
    doctest.testmod(sys.modules[__name__])
    unittest.main(defaultTest='suite')
