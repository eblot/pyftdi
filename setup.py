#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2010-2011 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2010-2011 Neotion
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

from distutils.core import setup


def _read(fname):
    import os
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name='pyftdi',
    version='0.2.2',
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
