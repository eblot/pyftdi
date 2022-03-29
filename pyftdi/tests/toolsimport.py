#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2020, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.

#pylint: disable-msg=empty-docstring
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=no-self-use
#pylint: disable-msg=invalid-name
#pylint: disable-msg=global-statement

from doctest import testmod
from importlib import import_module
from os.path import dirname, join as joinpath
from sys import modules, path as syspath
from unittest import TestCase, TestSuite, makeSuite, main as ut_main


class ToolsTestCase(TestCase):
    """Test tool suite can be loaded.

       This is especially useful to find Python syntax version mismatch
       and other not-yet-supported modules/features.

       PyFtdi and tools should support Python 3.7 onwards.
    """

    @classmethod
    def setUpClass(cls):
        tools_path = joinpath(dirname(dirname(__file__)), 'bin')
        syspath.append(tools_path)

    def test_ftconf(self):
        """Test ftconf.py tool"""
        mod = import_module('ftconf')
        self.assertIsNot(getattr(mod, 'main', None), mod)
        self.assertIsNot(mod.__doc__, None)

    def test_i2cscan(self):
        """Test ftconf.py tool"""
        mod = import_module('i2cscan')
        self.assertIsNot(getattr(mod, 'main', None), mod)
        self.assertIsNot(mod.__doc__, None)

    def test_pyterm(self):
        """Test ftconf.py tool"""
        mod = import_module('pyterm')
        self.assertIsNot(getattr(mod, 'main', None), mod)
        self.assertIsNot(mod.__doc__, None)

    def test_ftdi_urls(self):
        """Test ftconf.py tool"""
        mod = import_module('ftdi_urls')
        self.assertIsNot(getattr(mod, 'main', None), mod)
        self.assertIsNot(mod.__doc__, None)


def suite():
    suite_ = TestSuite()
    suite_.addTest(makeSuite(ToolsTestCase, 'test'))
    return suite_


def main():
    testmod(modules[__name__])
    ut_main(defaultTest='suite')


if __name__ == '__main__':
    main()
