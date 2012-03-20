#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2010-2012 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2010-2012 Neotion
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

VERSION='0.5.2'

def _read(fname):
    import os
    return open(os.path.join(os.path.dirname(__file__),
                'pyftdi', fname)).read()

setup(
    name='pyftdi',
    version=VERSION,
    description='FTDI device driver',
    author='Emmanuel Blot',
    author_email='emmanuel.blot@free.fr',
    license='LGPL v2',
    keywords = 'driver ftdi usb serial spi jtag prolific rs232',
    url='http://github.com/eblot/pyftdi',
    download_url=''.join(('https://github.com/eblot/pyftdi/tarball/v',
                          VERSION)),
    packages=['pyftdi','pyftdi.pyftdi','pyftdi.serialext'],
    package_data={'pyftdi': ['*.rst'],
                  'pyftdi.serialext' : ['*.rst']},
    requires=['pyusb (>= 1.0.0a2)'],
    classifiers=[
        'Development Status :: 4 - Beta',
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
