# Copyright (C) 2010-2012 Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
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

import threading
import usb.core
import usb.util

__all__ = ['UsbTools']

class UsbTools(object):
    """Helpers to obtain information about connected USB devices."""

    # Need to maintain a list of reference USB devices, to circumvent a
    # limitation in pyusb that prevents from opening several times the same
    # USB device. The following dictionary used bus/address/vendor/product keys
    # to track (device, refcount) pairs
    DEVICES = {}
    LOCK = threading.RLock()
    USBDEVICES = []

    @staticmethod
    def find_all(vps):
        """Find all devices that match the vendor/product pairs of the vps
           list."""
        devices = []
        devs = UsbTools._find_devices(vps)
        for dev in devs:
            ifcount = max([cfg.bNumInterfaces for cfg in dev])
            sernum = usb.util.get_string(dev, 64, dev.iSerialNumber)
            description = usb.util.get_string(dev, 64, dev.iProduct)
            devices.append((dev.idVendor, dev.idProduct, sernum, ifcount,
                            description))
        return devices

    @classmethod
    def get_device(cls, vendor, product, index, serial, description):
        """Find a previously open device with the same vendor/product
           or initialize a new one, and return it"""
        cls.LOCK.acquire()
        try:
            vps = [(vendor, product)]
            if index or serial or description:
                dev = None
                if not vendor:
                    raise AssertionError('Vendor identifier is required')
                devs = cls._find_devices(vps)
                if description:
                    devs = [dev for dev in devs if \
                              usb.util.get_string(dev, 64, dev.iProduct) \
                                == description]
                if serial:
                    devs = [dev for dev in devs if \
                              usb.util.get_string(dev, 64, dev.iSerialNumber) \
                                == serial]
                try:
                    dev = devs[index]
                except IndexError:
                    raise IOError("No such device")
            else:
                devs = cls._find_devices(vps)
                dev = devs and devs[0] or None
            if not dev:
                raise IOError('Device not found')
            try:
                devkey = (dev.bus, dev.address, vendor, product)
                if None in devkey[0:2]:
                    raise AttributeError('USB back does not support bus '
                                         'enumeration')
            except AttributeError:
                devkey = (vendor, product)
            if devkey not in cls.DEVICES:
                for configuration in dev:
                    # we need to detach any kernel driver from the device
                    # be greedy: reclaim all device interfaces from the kernel
                    for interface in configuration:
                        ifnum = interface.bInterfaceNumber
                        try:
                            if not dev.is_kernel_driver_active(ifnum):
                                continue
                            dev.detach_kernel_driver(ifnum)
                        except NotImplementedError, e:
                            # only libusb 1.x backend implements this method
                            break
                        except usb.core.USBError, e:
                            pass
                dev.set_configuration()
                cls.DEVICES[devkey] = [dev, 1]
            else:
                cls.DEVICES[devkey][1] += 1
            return cls.DEVICES[devkey][0]
        finally:
            cls.LOCK.release()

    @classmethod
    def release_device(cls, usb_dev):
        """Release a previously open device, if it not used anymore"""
        # Lookup for ourselves in the class dictionary
        cls.LOCK.acquire()
        try:
            for devkey in cls.DEVICES:
                dev, refcount = cls.DEVICES[devkey]
                if dev == usb_dev:
                    # found
                    if refcount > 1:
                        # another interface is open, decrement
                        cls.DEVICES[devkey][1] -= 1
                    else:
                        # last interface in use, release
                        usb.util.dispose_resources(cls.DEVICES[devkey][0])
                        del cls.DEVICES[devkey]
                    break
        finally:
            cls.LOCK.release()

    @classmethod
    def _find_devices(cls, vps):
        """Find an USB device and return it.
           This code re-implements the usb.core.find() method using a local
           cache to avoid calling several times the underlying LibUSB and the
           system USB calls to enumerate the available USB devices. As these
           calls are time-hungry (about 1 second/call), the enumerated devices
           are cached. It consumes a bit more memory but dramatically improves
           start-up time.
           Hopefully, this kludge is temporary and replaced with a better
           implementation from PyUSB at some point.
        """
        cls.LOCK.acquire()
        try:
            backend = None
            candidates = ('libusb1', 'libusb10', 'libusb0', 'libusb01',
                          'openusb')
            um = __import__('usb.backend', globals(), locals(),
                            candidates, -1)
            for c in candidates:
                try:
                    m = getattr(um, c)
                except AttributeError:
                    continue
                backend = m.get_backend()
                if backend is not None:
                    break
            else:
                raise ValueError('No backend available')
            if not cls.USBDEVICES:
                # not freed until Python runtime completion
                # enumerate_devices returns a generator, so back up the
                # generated device into a list. To save memory, we only
                # back up the supported devices
                devlist = []
                vpdict = {}
                for v, p in vps:
                    vpdict.setdefault(v, [])
                    vpdict[v].append(p)
                for dev in backend.enumerate_devices():
                    device = usb.core.Device(dev, backend)
                    vendor = device.idVendor
                    product = device.idProduct
                    if vendor in vpdict:
                        products = vpdict[vendor]
                        if products and (product not in products):
                            continue
                        devlist.append(device)
                cls.USBDEVICES = devlist
            return cls.USBDEVICES
        finally:
            cls.LOCK.release()
