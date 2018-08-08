#!/bin/sh

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
