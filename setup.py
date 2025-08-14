#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2010-2025 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2010-2016 Neotion
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# pylint: disable=unused-variable
# pylint: disable=missing-docstring
# pylint: disable=broad-except

from codecs import open as codec_open
from os import close, getcwd, unlink, walk
from os.path import abspath, dirname, join as joinpath, relpath
from py_compile import compile as pycompile, PyCompileError
from re import split as resplit, search as research
from sys import stderr, exit as sysexit
from tempfile import mkstemp
from setuptools import Command, find_packages, setup
from setuptools.command.build_py import build_py


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
    'Operating System :: MacOS :: MacOS X',
    'Operating System :: POSIX',
    'Programming Language :: Python :: 3.9',
    'Programming Language :: Python :: 3.10',
    'Programming Language :: Python :: 3.11',
    'Programming Language :: Python :: 3.12',
    'Programming Language :: Python :: 3.13',
    'Topic :: Software Development :: Libraries :: Python Modules',
    'Topic :: System :: Hardware :: Hardware Drivers',
]
INSTALL_REQUIRES = [
    'pyusb >= 1.0.0, != 1.2.0',
    'pyserial >= 3.0',
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
    meta_match = research(rf"(?m)^__{meta}__ = ['\"]([^'\"]*)['\"]", META_FILE)
    if meta_match:
        return meta_match.group(1)
    raise RuntimeError(f'Unable to find __{meta}__ string.')


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
                raise SyntaxError(f"Cannot byte-compile '{file}'") from exc
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
        self.announce('checking coding style', level=2)
        filecount = 0
        topdir = dirname(__file__) or getcwd()
        error_count = 0
        for dpath, dnames, fnames in walk(topdir):
            dnames[:] = [d for d in dnames
                         if not d.startswith('.') and d != 'doc']
            for filename in (joinpath(dpath, f)
                             for f in fnames if f.endswith('.py')):
                self.announce(f'checking {relpath(filename, topdir)}', level=2)
                with open(filename, 'rt', encoding='utf-8') as pfp:
                    for lpos, line in enumerate(pfp, start=1):
                        if len(line) > 80:
                            toppath = relpath(filename, topdir)
                            print(f'  invalid line width in {toppath}:{lpos}',
                                  file=stderr)
                            print(f'    {line.strip()}', file=stderr)
                            error_count += 1
                filecount += 1
        if error_count:
            raise RuntimeError(f'{error_count} errors')
        if not filecount:
            raise RuntimeError(f'No Python file found from "{topdir}"')


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
        long_description_content_type='text/x-rst',
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
        python_requires='>=3.9',
    )


if __name__ == '__main__':
    try:
        main()
    except Exception as exc_:
        print(exc_, file=stderr)
        sysexit(1)
