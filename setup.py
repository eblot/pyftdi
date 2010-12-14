#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2010 Emmanuel Blot
# All rights reserved.

from setuptools import setup, find_packages

setup (
    name = 'pyftdi',
    version = '0.1.0',
    description = 'Pure Python FTDI device driver',
    author = 'Emmanuel Blot',
    author_email = 'emmanuel.blot@free.fr',
    license = 'LGPL v2',

    packages = find_packages('pyftdi',
                             exclude=['*.tests',
                                      '*.tests.*',
                                      'tests.*',
                                      'tests',
                                      'test']),
    package_dir = {'':'pyftdi'},
    zip_safe = True,
    install_requires = [ 'setuptools>=0.6c11', 
                       #'pyusb >= 1.0.0a1' 
                       ],
)
