#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2010-2011 Emmanuel Blot
# All rights reserved.

from distutils.core import setup

def _read(fname):
    import os
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name='pyftdi',
    version='0.1.1',
    description='FTDI device driver',
    author='Emmanuel Blot',
    author_email='emmanuel.blot@free.fr',
    license='LGPL v2',
    keywords = 'driver ftdi usb serial spi jtag',
    url='http://github.com/eblot/pyftdi',
    download_url='https://github.com/eblot/pyftdi/tarball/master',
    packages=['pyftdi'],
    requires=[ 'pyusb (>= 1.0.0a0)' ],
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Environment :: Other Environment',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: GNU Library or '
            'Lesser General Public License (LGPL)',
        'Operating System :: MacOS :: MacOS X',
        'Operating System :: POSIX',
        'Operating System :: Microsoft :: Windows :: Windows 95/98/2000',
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware :: Hardware Drivers',
    ],
    long_description=_read('README.rst'),
)
