# Copyright (C) 2010-2016 Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (C) 2016 Emmanuel Bouaziz <ebouaziz@free.fr>
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
from pyftdi.misc import to_int
from six import print_
from six.moves.urllib.parse import urlsplit

__all__ = ['UsbTools']


class UsbToolsError(Exception):
    """UsbTools error"""


class UsbTools(object):
    """Helpers to obtain information about connected USB devices."""

    # Need to maintain a list of reference USB devices, to circumvent a
    # limitation in pyusb that prevents from opening several times the same
    # USB device. The following dictionary used bus/address/vendor/product keys
    # to track (device, refcount) pairs
    Devices = {}
    Lock = threading.RLock()
    UsbDevices = {}
    UsbApi = None

    @staticmethod
    def find_all(vps, nocache=False):
        """Find all devices that match the vendor/product pairs of the vps
           list."""
        devs = set()
        for v, p in vps:
            devs.update(UsbTools._find_devices(v, p, nocache))
        devices = set()
        for dev in devs:
            ifcount = max([cfg.bNumInterfaces for cfg in dev])
            sernum = UsbTools.get_string(dev, dev.iSerialNumber)
            description = UsbTools.get_string(dev, dev.iProduct)
            devices.add((dev.idVendor, dev.idProduct, sernum, ifcount,
                         description))
        return list(devices)

    @classmethod
    def flush_cache(cls):
        cls.Lock.acquire()
        cls.UsbDevices = {}
        cls.Lock.release()

    @classmethod
    def get_device(cls, vendor, product, index, serial, description):
        """Find a previously open device with the same vendor/product
           or initialize a new one, and return it"""
        cls.Lock.acquire()
        try:
            if index or serial or description:
                dev = None
                if not vendor:
                    raise ValueError('Vendor identifier is required')
                devs = cls._find_devices(vendor, product)
                if description:
                    devs = [dev for dev in devs if
                            UsbTools.get_string(dev, dev.iProduct) ==
                            description]
                if serial:
                    devs = [dev for dev in devs if
                            UsbTools.get_string(dev, dev.iSerialNumber) ==
                            serial]
                try:
                    dev = devs[index]
                except IndexError:
                    raise IOError("No such device")
            else:
                devs = cls._find_devices(vendor, product)
                dev = devs and list(devs)[0] or None
            if not dev:
                raise IOError('Device not found')
            try:
                devkey = (dev.bus, dev.address, vendor, product)
                if None in devkey[0:2]:
                    raise AttributeError('USB backend does not support bus '
                                         'enumeration')
            except AttributeError:
                devkey = (vendor, product)
            if devkey not in cls.Devices:
                for configuration in dev:
                    # we need to detach any kernel driver from the device
                    # be greedy: reclaim all device interfaces from the kernel
                    for interface in configuration:
                        ifnum = interface.bInterfaceNumber
                        try:
                            if not dev.is_kernel_driver_active(ifnum):
                                continue
                            dev.detach_kernel_driver(ifnum)
                        except NotImplementedError:
                            # only libusb 1.x backend implements this method
                            break
                        except usb.core.USBError:
                            pass
                # only change the active configuration if the active one is
                # not the first. This allows other libusb sessions running
                # with the same device to run seamlessly.
                if dev.get_active_configuration().bConfigurationValue != 1:
                    try:
                        dev.set_configuration()
                    except usb.core.USBError:
                        pass
                cls.Devices[devkey] = [dev, 1]
            else:
                cls.Devices[devkey][1] += 1
            return cls.Devices[devkey][0]
        finally:
            cls.Lock.release()

    @classmethod
    def release_device(cls, usb_dev):
        """Release a previously open device, if it not used anymore"""
        # Lookup for ourselves in the class dictionary
        cls.Lock.acquire()
        try:
            for devkey in cls.Devices:
                dev, refcount = cls.Devices[devkey]
                if dev == usb_dev:
                    # found
                    if refcount > 1:
                        # another interface is open, decrement
                        cls.Devices[devkey][1] -= 1
                    else:
                        # last interface in use, release
                        usb.util.dispose_resources(cls.Devices[devkey][0])
                        del cls.Devices[devkey]
                    break
        finally:
            cls.Lock.release()

    @classmethod
    def _find_devices(cls, vendor, product, nocache=False):
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
        cls.Lock.acquire()
        try:
            backend = None
            candidates = ('libusb1', 'libusb10', 'libusb0', 'libusb01',
                          'openusb')
            um = __import__('usb.backend', globals(), locals(),
                            candidates, 0)
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
            vp = (vendor, product)
            if nocache or (vp not in cls.UsbDevices):
                # not freed until Python runtime completion
                # enumerate_devices returns a generator, so back up the
                # generated device into a list. To save memory, we only
                # back up the supported devices
                devs = set()
                vpdict = {}
                vpdict.setdefault(vendor, [])
                vpdict[vendor].append(product)
                for dev in backend.enumerate_devices():
                    device = usb.core.Device(dev, backend)
                    vendor = device.idVendor
                    product = device.idProduct
                    if vendor in vpdict:
                        products = vpdict[vendor]
                        if products and (product not in products):
                            continue
                        devs.add(device)
                cls.UsbDevices[vp] = devs
            return cls.UsbDevices[vp]
        finally:
            cls.Lock.release()

    @staticmethod
    def parse_url(urlstr, devclass, scheme, vdict, pdict, default_vendor):
        urlparts = urlsplit(urlstr)
        if scheme != urlparts.scheme:
            raise UsbToolsError("Invalid URL: %s" % urlstr)
        plcomps = urlparts.netloc.split(':') + [''] * 2
        try:
            plcomps[0] = vdict.get(plcomps[0], plcomps[0])
            if plcomps[0]:
                vendor = to_int(plcomps[0])
            else:
                vendor = None
            product_ids = pdict.get(vendor, None)
            if not product_ids:
                product_ids = pdict[default_vendor]
            plcomps[1] = product_ids.get(plcomps[1], plcomps[1])
            if plcomps[1]:
                product = to_int(plcomps[1])
            else:
                product = None
            if not urlparts.path:
                raise UsbToolsError('URL string is missing device port')
            path = urlparts.path.strip('/')
            if path == '?' or (not path and urlstr.endswith('?')):
                show_devices = True
            else:
                interface = to_int(path)
                show_devices = False
        except (IndexError, ValueError):
            raise UsbToolsError('Invalid device URL: %s' % urlstr)
        sernum = None
        idx = 0
        if plcomps[2]:
            try:
                idx = to_int(plcomps[2])
                if idx > 255:
                    idx = 0
                    raise ValueError
                if idx:
                    idx -= 1
            except ValueError:
                sernum = plcomps[2]
        if not vendor or not product or sernum or idx:
            # Need to enumerate USB devices to find a matching device
            vendors = vendor and [vendor] or \
                set(vdict.values())
            vps = set()
            for v in vendors:
                products = pdict.get(v, [])
                for p in products:
                    vps.add((v, products[p]))
            devices = devclass.find_all(vps)
            candidates = []
            if sernum:
                if sernum not in [dev[2] for dev in devices]:
                    raise UsbToolsError("No USB device with S/N %s" % sernum)
                for v, p, s, i, d in devices:
                    if s != sernum:
                        continue
                    if vendor and vendor != v:
                        continue
                    if product and product != p:
                        continue
                    candidates.append((v, p, s, i, d))
            else:
                for v, p, s, i, d in devices:
                    if vendor and vendor != v:
                        continue
                    if product and product != p:
                        continue
                    candidates.append((v, p, s, i, d))
                if not show_devices:
                    try:
                        vendor, product, ifport, ifcount, description = \
                            candidates[idx]
                    except IndexError:
                        raise UsbToolsError("No USB device #%d" % idx)
        if show_devices:
            UsbTools.show_devices(scheme, vdict, pdict, candidates)
            raise SystemExit(candidates and
                             'Please specify the USB device' or
                             'No USB-Serial device has been detected')
        if vendor not in pdict:
            raise UsbToolsError('Vendor ID 0x%04x not supported' % vendor)
        if product not in pdict[vendor].values():
            raise UsbToolsError('Product ID 0x%04x not supported' % product)
        return vendor, product, interface, sernum, idx

    @staticmethod
    def show_devices(scheme, vdict, pdict, candidates, out=None):
        """Show supported devices"""
        from string import printable as printablechars
        if not out:
            import sys
            out = sys.stdout
        indices = {}
        interfaces = []
        for (v, p, s, i, d) in candidates:
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
            if not s:
                s = ''
            if [c for c in s if c not in printablechars or c == '?']:
                serial = '%d' % indices[ikey]
            else:
                serial = s
            # Now print out the prettiest URL syntax
            for j in range(1, i+1):
                # On most configurations, low interfaces are used for MPSSE,
                # high interfaces are dedicated to UARTs
                interfaces.append((scheme, vendor, product, serial, j, d))
        if interfaces:
            print_("Available interfaces:", file=out)
            for scheme, vendor, product, serial, j, d in interfaces:
                fmt = '  %s://%s/%d  '
                parts = [vendor, product]
                if serial:
                    parts.append(serial)
                desc = d and '  (%s)' % d
                # the description may contain characters that cannot be
                # emitted in the output stream encoding format
                try:
                    print_(fmt % (scheme, ':'.join(parts), j), end='',
                           file=out)
                except Exception:
                    print_(fmt % (scheme, ':'.join([vendor, product, '???']),
                                  j), end='', file=out)
                try:
                    print_(desc or '', file=out)
                except Exception:
                    print_('', file=out)
            print_('', file=out)

    @classmethod
    def get_string(cls, device, strname):
        """Retrieve a string from the USB device, dealing with PyUSB API breaks
        """
        if cls.UsbApi is None:
            import inspect
            args, varargs, varkw, defaults = \
                inspect.getargspec(usb.util.get_string)
            if (len(args) >= 3) and args[1] == 'length':
                cls.UsbApi = 1
            else:
                cls.UsbApi = 2
        if cls.UsbApi == 2:
            return usb.util.get_string(device, strname)
        else:
            return usb.util.get_string(device, 64, strname)
