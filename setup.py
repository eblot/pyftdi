#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2010-2021 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2010-2016 Neotion
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#pylint: disable-msg=unused-variable
#pylint: disable-msg=missing-docstring
#pylint: disable-msg=broad-except
#pylint: disable-msg=no-self-use

from codecs import open as codec_open
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from distutils.cmd import Command
from distutils.log import DEBUG, INFO
from os import close, getcwd, unlink, walk
from os.path import abspath, dirname, join as joinpath, relpath
from py_compile import compile as pycompile, PyCompileError
from re import split as resplit, search as research
from sys import stderr, exit as sysexit
from tempfile import mkstemp


NAME = 'pyftdi'
PACKAGES = find_packages(where='.')
META_PATH = joinpath('pyftdi', '__init__.py')
KEYWORDS = ['driver', 'ftdi', 'usb', 'serial', 'spi', 'i2c', 'twi', 'rs232',
            'gpio', 'bit-bang']
CLASSIFIERS = [
    'Development Status :: 4 - Beta',
    'Environment :: Other Environment',
    'Natural Language :: English',
    'Intended Audience :: Developers',
    'License :: OSI Approved :: BSD License',
    'Operating System :: MacOS :: MacOS X',
    'Operating System :: POSIX',
    'Programming Language :: Python :: 3.6',
    'Programming Language :: Python :: 3.7',
    'Programming Language :: Python :: 3.8',
    'Programming Language :: Python :: 3.9',
    'Topic :: Software Development :: Libraries :: Python Modules',
    'Topic :: System :: Hardware :: Hardware Drivers',
]
INSTALL_REQUIRES = [
    'pyusb >= 1.0.0, != 1.2.0',
    'pyserial >= 3.0',
]
TEST_REQUIRES = [
    'ruamel.yaml >= 0.16',
]

HERE = abspath(dirname(__file__))


def read(*parts):
    """
    Build an absolute path from *parts* and and return the contents of the
    resulting file.  Assume UTF-8 encoding.
    """
    with codec_open(joinpath(HERE, *parts), 'rb', 'utf-8') as dfp:
        return dfp.read()


def read_desc(*parts):
    """Read and filter long description
    """
    text = read(*parts)
    text = resplit(r'\.\.\sEOT', text)[0]
    return text


META_FILE = read(META_PATH)


def find_meta(meta):
    """
    Extract __*meta*__ from META_FILE.
    """
    meta_match = research(
        r"(?m)^__{meta}__ = ['\"]([^'\"]*)['\"]".format(meta=meta),
        META_FILE
    )
    if meta_match:
        return meta_match.group(1)
    raise RuntimeError("Unable to find __{meta}__ string.".format(meta=meta))


class BuildPy(build_py):
    """Override byte-compile sequence to catch any syntax error issue.

       For some reason, distutils' byte-compile when it forks a sub-process
       to byte-compile a .py file into a .pyc does NOT check the success of
       the compilation. Therefore, any syntax error is explictly ignored,
       and no output file is generated. This ends up generating an incomplete
       package w/ a nevertheless successfull setup.py execution.

       Here, each Python file is build before invoking distutils, so that any
       syntax error is catched, raised and setup.py actually fails should this
       event arise.

       This step is critical to check that an unsupported syntax does not end
       up as a 'valid' package from setuptools perspective...
    """

    def byte_compile(self, files):
        for file in files:
            if not file.endswith('.py'):
                continue
            pfd, pyc = mkstemp('.pyc')
            close(pfd)
            try:
                pycompile(file, pyc, doraise=True)
                continue
            except PyCompileError as exc:
                # avoid chaining exceptions
                print(str(exc), file=stderr)
                raise SyntaxError("Cannot byte-compile '%s'" % file)
            finally:
                unlink(pyc)
        super().byte_compile(files)


class CheckStyle(Command):
    """A custom command to check Python coding style."""

    description = 'check coding style'
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        self.announce('checking coding style', level=INFO)
        filecount = 0
        topdir = dirname(__file__) or getcwd()
        for dpath, dnames, fnames in walk(topdir):
            dnames[:] = [d for d in dnames
                         if not d.startswith('.') and d != 'doc']
            for filename in (joinpath(dpath, f)
                             for f in fnames if f.endswith('.py')):
                self.announce('checking %s' % relpath(filename, topdir),
                              level=INFO)
                with open(filename, 'rt') as pfp:
                    for lpos, line in enumerate(pfp, start=1):
                        if len(line) > 80:
                            print('\n  %d: %s' % (lpos, line.rstrip()))
                            raise RuntimeError("Invalid line width '%s'" %
                                               relpath(filename, topdir))
                filecount += 1
        if not filecount:
            raise RuntimeError('No Python file found from "%s"' %
                               topdir)


def main():
    setup(
        cmdclass={
            'build_py': BuildPy,
            'check_style': CheckStyle
        },
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
        long_description=read_desc('pyftdi/doc/index.rst'),
        packages=PACKAGES,
        scripts=['pyftdi/bin/i2cscan.py',
                 'pyftdi/bin/ftdi_urls.py',
                 'pyftdi/bin/ftconf.py',
                 'pyftdi/bin/pyterm.py'],
        package_dir={'': '.'},
        package_data={'pyftdi': ['*.rst', 'doc/*.rst', 'doc/api/*.rst',
                                 'INSTALL'],
                      'pyftdi.serialext': ['*.rst', 'doc/api/uart.rst']},
        classifiers=CLASSIFIERS,
        install_requires=INSTALL_REQUIRES,
        test_requires=TEST_REQUIRES,
        python_requires='>=3.6',
    )


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(exc, file=stderr)
        sysexit(1)
