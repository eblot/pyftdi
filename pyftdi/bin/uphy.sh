#!/bin/sh

# Load/unload kernel extension helper for macOS

system=$(uname -s)
if [ "${system}" != "Darwin" ]; then
    echo "This script is dedicated to macOS" >&2
    exit 1
fi
version=$(sw_vers -productVersion | cut -d. -f2)
if [ ${version} -lt 9 ]; then
    echo "This version of OS X does not use an Apple FTDI driver"
    exit 0
fi
if [ ${version} -gt 13 ]; then
    echo "Apple FTDI driver on this macOS version should not be unloaded" >&2
    exit 1
fi
kextstat 2>/dev/null | grep com.apple.driver.AppleUSBFTDI > /dev/null
if [ $? -eq 0 ]; then
    echo "Admin priviledge required to unload Apple FTDI driver"
    sudo kextunload -v -bundle com.apple.driver.AppleUSBFTDI
else
   if [ "$1" = "0" ]; then
      echo "Admin priviledge required to load Apple FTDI driver"
      sudo kextload -v -bundle com.apple.driver.AppleUSBFTDI
   fi
fi
