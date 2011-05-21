# Copyright (C) 2010-2011 Emmanuel Blot <emmanuel.blot@free.fr>
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


class UsbError(IOError):
    """Base class for error with a USB device"""


class UsbTools(object):
    """
    """
    
    # Need to maintain a list of reference USB devices, to circumvent a
    # limitation in pyusb that prevents from opening several times the same
    # USB device. The following dictionary used bus/address/vendor/product keys
    # to track (device, refcount) pairs if USBSCAN is available or simply
    # vendor/product
    DEVICES = {}
    LOCK = threading.RLock()
    USBDEVICES = []

    @staticmethod
    def find_all(vps):
        """Find a previously open device with the same vendor/product
           or initialize a new one, and return it"""
        devices = []
        devs = UsbTools._find_devices(vps)
        for dev in devs:
            sernum = usb.util.get_string(dev, 64, dev.iSerialNumber)
            devices.append((dev.idVendor, dev.idProduct, sernum))
        return devices

    @staticmethod
    def is_usbscan_available():
        """Check if the required extension to manage USB buses and addresses
        """
        try:
            from pkg_resources import get_distribution, parse_version
        except ImportError:
            # if pkg_resources is not available, do not try to test PyUSB
            # version
            return False
        try:
            pyusb_version = parse_version(get_distribution("pyusb").version)
        except Exception:
            # if pyusb is not installed as a distribution, cannot check
            # version
            return False
        # check if the required version is installed
        req_version = ('00000001', '*a', '00000001', '*final')
        if pyusb_version < req_version:
            return False
        # check if the required patches are installed
        from usb.core import Device as UsbDevice
        for extension in ['get_bus_number', 'get_device_address']:
            try:
                getattr(UsbDevice, extension)
            except AttributeError:
                return False
        return True

    @classmethod
    def get_device(cls, vendor, product, index, serial):
        """Find a previously open device with the same vendor/product
           or initialize a new one, and return it"""
        usbscan = cls.is_usbscan_available()
        cls.LOCK.acquire()
        try:
            vps = [(vendor, product)]
            if index or serial:
                dev = None
                if not vendor:
                    raise AssertionError('Vendor identifier is required')
                devs = Ftdi._find_devices(vps)
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
            if usbscan:
                bus = dev.get_bus_number()
                address = dev.get_device_address()
                devkey = (bus, address, vendor, product)
            else:
                devkey = (vendor, product)
            if devkey not in cls.DEVICES:
                for configuration in dev:
                    # we need to detach any kernel driver from the device
                    # be greedy: reclaim all device interfaces from the kernel
                    for interface in configuration:
                        ifnum = interface.bInterfaceNumber
                        if not dev.is_kernel_driver_active(ifnum):
                            continue
                        try:
                            dev.detach_kernel_driver(ifnum)
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
                        dispose_resources(cls.DEVICES[devkey][0])
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
            import usb.backend.libusb10 as libusb10
            import usb.backend.libusb01 as libusb01
            import usb.backend.openusb as openusb
            for m in (libusb10, openusb, libusb01):
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

    @staticmethod
    def show_devices(scheme, vdict, pdict, idict, candidates,
                     out=None):
        from string import printable as printablechars
        if not out:
            import sys
            out = sys.stdout
        print >> out, "Available interfaces:"
        indices = {}
        for (v, p, s) in candidates:
            try:
                ifcount = idict[v][p]
            except KeyError, e:
                continue
            ikey = (v, p)
            indices[ikey] = indices.get(ikey, 0) + 1
            # try to find a matching string for the current vendor
            vendors = []
            # fallback if no matching string for the current vendor is found
            vendor = '%04x' % v
            for vc in vdict:
                if vdict[vc] == v:
                    vendors.append(vc)
            if vendors:
                vendors.sort(key=len)
                vendor = vendors[0]
            # try to find a matching string for the current vendor
            # fallback if no matching string for the current product is found
            product = '%04x' % p
            try:
                products = []
                productids = pdict[v]
                for pc in productids:
                    if productids[pc] == p:
                        products.append(pc)
                if products:
                    products.sort(key=len)
                    product = products[0]
            except KeyError:
                pass
            # if the serial number is an ASCII char, use it, or use the index
            # value
            if [c for c in s if c not in printablechars or c == '?']:
                serial = '%d' % indices[ikey]
            else:
                serial = s
            # Now print out the prettiest URL syntax
            for i in range(1, ifcount+1):
                # On most configurations, low interfaces are used for MPSSE,
                # high interfaces are dedicated to UARTs
                print >> out, '  %s%s:%s:%s/%d' % \
                    (scheme, vendor, product, serial, i)
        print >> out, ''
