This module contains a basic driver for Prolific PL2303 chip written in
pure Python. PL2303 is not an FTDI device, but it may serve the same purpose:
a USB-to-serial adapter.

As such, a Python driver for this device has been added to this project sarting
at version 0.4.0, so that a PL2303 serial adaptor can be used as an FTDI
alternative to drive a serial port from a USB bus.

For now, the current status is highly experimental and should not be used but
for experimentations with the Prolific devices.
