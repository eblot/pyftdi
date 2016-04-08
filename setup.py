#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2010-2016 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2010-2016 Neotion
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

try:
    # try to use setuptools first, so extended command set such as
    # python setup.py develop is available
    from setuptools import setup
except ImportError:
    # if setuptools package is not available, fall back to the default
    # distribution package.
    from distutils.core import setup
from pyftdi import __version__ as VERSION


def _read(fname):
    import os
    return open(os.path.join(os.path.dirname(__file__),
                'pyftdi', fname)).read()

setup(
    name='pyftdi',
    version=VERSION,
    description='FTDI device driver (pure Python)',
    author='Emmanuel Blot',
    author_email='emmanuel.blot@free.fr',
    license='LGPL v2',
    keywords='driver ftdi usb serial spi rs232 gpio bit-bang',
    url='http://github.com/eblot/pyftdi',
    download_url='https://github.com/eblot/pyftdi/archive/v%s.tar.gz' %
                 VERSION,
    packages=['pyftdi', 'pyftdi.serialext'],
    extras_require={'spiflash': []},
    package_data={'pyftdi': ['*.rst'],
                  'pyftdi.serialext': ['*.rst']},
    requires=['pyusb (>= 1.0.0b1)',
              'pyserial (>= 2.6)',
              'six'],
    install_requires=['pyusb>=1.0.0b1',
                      'pyserial>=2.6',
                      'six'],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Other Environment',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: GNU Library or '
            'Lesser General Public License (LGPL)',
        'Operating System :: MacOS :: MacOS X',
        'Operating System :: POSIX',
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware :: Hardware Drivers',
    ],
    long_description=_read('README.rst'),
)
