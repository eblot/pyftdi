#!/usr/bin/env python3

"""Check readline feature."""

from os.path import dirname
from sys import path
from serial import serial_for_url

path.append(dirname(dirname(dirname(dirname(__file__)))))

# pylint: disable=wrong-import-position
from pyftdi import serialext


def main():
    """Verify readline() works with PyFTDI extension."""
    serialext.touch()
    with serial_for_url('ftdi:///1', baudrate=115200) as ser:
        line = ser.readline()
        print(line)


if __name__ == '__main__':
    main()
