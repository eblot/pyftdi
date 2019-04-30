#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2010-2019 Emmanuel Blot <emmanuel.blot@free.fr>
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

import codecs
import os
import re

from setuptools import find_packages, setup

NAME = 'pyftdi'
PACKAGES = find_packages(where='.')
META_PATH = os.path.join('pyftdi', '__init__.py')
KEYWORDS = ['driver', 'ftdi', 'usb', 'serial', 'spi', 'i2c', 'twi', 'rs232',
            'gpio', 'bit-bang']
CLASSIFIERS = [
    'Development Status :: 4 - Beta',
    'Environment :: Other Environment',
    'Natural Language :: English',
    'Intended Audience :: Developers',
    'License :: OSI Approved :: GNU Library or '
    'Lesser General Public License (LGPL)',
    'Operating System :: MacOS :: MacOS X',
    'Operating System :: POSIX',
    'Programming Language :: Python :: 3.5',
    'Topic :: Software Development :: Libraries :: Python Modules',
    'Topic :: System :: Hardware :: Hardware Drivers',
]
INSTALL_REQUIRES = [
    'pyusb >= 1.0.0',
    'pyserial >= 3.0',
]


HERE = os.path.abspath(os.path.dirname(__file__))


def read(*parts):
    """
    Build an absolute path from *parts* and and return the contents of the
    resulting file.  Assume UTF-8 encoding.
    """
    with codecs.open(os.path.join(HERE, *parts), 'rb', 'utf-8') as f:
        return f.read()


META_FILE = read(META_PATH)


def find_meta(meta):
    """
    Extract __*meta*__ from META_FILE.
    """
    meta_match = re.search(
        r"^__{meta}__ = ['\"]([^'\"]*)['\"]".format(meta=meta),
        META_FILE, re.M
    )
    if meta_match:
        return meta_match.group(1)
    raise RuntimeError("Unable to find __{meta}__ string.".format(meta=meta))


if __name__ == '__main__':
    setup(
        name=NAME,
        description=find_meta('description'),
        license=find_meta('license'),
        url=find_meta('uri'),
        version=find_meta('version'),
        author=find_meta('author'),
        author_email=find_meta('email'),
        maintainer=find_meta('author'),
        maintainer_email=find_meta('email'),
        keywords=KEYWORDS,
        long_description=read('README.rst'),
        packages=PACKAGES,
        scripts=['pyftdi/bin/i2cscan.py'],
        package_dir={'': '.'},
        package_data={'pyftdi': ['*.rst', 'doc/*.rst', 'doc/api/*.rst',
                                 'INSTALL'],
                      'pyftdi.serialext': ['*.rst', 'doc/api/uart.rst']},
        classifiers=CLASSIFIERS,
        install_requires=INSTALL_REQUIRES,
        python_requires='>=3.5',
    )
