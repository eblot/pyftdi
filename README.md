# PyFtdi

![Python package](https://github.com/eblot/pyftdi/workflows/Python%20package/badge.svg)
![Mock tests](https://github.com/eblot/pyftdi/workflows/Python%20mock%20tests/badge.svg)
![Syntax tests](https://github.com/eblot/pyftdi/workflows/Python%20syntax%20tests/badge.svg)

[![PyPI](https://img.shields.io/pypi/v/pyftdi.svg?maxAge=2592000)](https://pypi.org/project/pyftdi/)
[![Python Versions](https://img.shields.io/pypi/pyversions/pyftdi.svg)](https://pypi.org/project/pyftdi/)
[![Downloads](https://img.shields.io/pypi/dm/pyftdi.svg)](https://pypi.org/project/pyftdi/)

## Documentation

PyFtdi documentation is available from https://eblot.github.io/pyftdi/

## Overview

PyFtdi aims at providing a user-space driver for popular FTDI devices,
implemented in pure Python language.

Suported FTDI devices include:

* UART and GPIO bridges

  * FT232R (single port, 3Mbps)
  * FT230X/FT231X/FT234X/ (single port, 3Mbps)

* UART, GPIO and multi-serial protocols (SPI, I2C, JTAG) bridges

  * FT2232C/D (dual port, clock up to 6 MHz)
  * FT232H (single port, clock up to 30 MHz)
  * FT2232H (dual port, clock up to 30 MHz)
  * FT4232H (quad port, clock up to 30 MHz)

## Features

PyFtdi currently supports the following features:

* UART/Serial USB converter, up to 12Mbps (depending on the FTDI device
  capability)
* GPIO/Bitbang support, with 8-bit asynchronous, 8-bit synchronous and
  8-/16-bit MPSSE variants
* SPI master, with simultanous GPIO support, up to 12 pins per port,
  with support for non-byte sized transfer
* I2C master, with simultanous GPIO support, up to 14 pins per port
* Basic JTAG master capabilities
* EEPROM support (some parameters cannot yet be modified, only retrieved)
* Experimental CBUS support on selected devices, 4 pins per port

## Supported host OSes

* macOS
* Linux
* FreeBSD
* Windows, although not officially supported

## License

`SPDX-License-Identifier: BSD-3-Clause`

## Warnings

### Python support

PyFtdi requires Python 3.7+.

### API break

PyFtdi *v0.52* was the last PyFtdi version to support Python 3.5.
Python 3.5 has reached end-of-life on September 5th, 2020.

Starting with version *v0.40.0*, several API changes are being introduced.
While PyFtdi tries to maintain backward compatibility with previous versions,
some of these changes may require existing clients to update calls to PyFtdi.

Do not upgrade to *v0.40.0* or above without testing your client against the
new PyFtdi releases. PyFtdi versions up to *v0.39.9* keep a stable API
with *v0.22+* series.

See the *Major Changes* section on the online documentation for details about
potential API breaks.
