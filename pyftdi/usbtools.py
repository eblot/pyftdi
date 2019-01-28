# Copyright (C) 2010-2019 Emmanuel Blot <emmanuel.blot@free.fr>
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
from string import printable as printablechars
from sys import platform, stdout
from urllib.parse import urlsplit
from .misc import to_int


class UsbToolsError(Exception):
    """UsbTools error"""


class UsbTools:
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
        """Find all devices that match the specified vendor/product pairs.

           :param vps: a sequence of 2-tuple (vid, pid) pairs
           :type vps: tuple(int, int)
           :param bool nocache: bypass cache to re-enumerate USB devices on
                                the host
           :return: a list of 5-tuple (vid, pid, sernum, iface, description)
                    device descriptors
           :rtype: list(tuple(int,int,str,int,str))
        """
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
        """Flush the FTDI device cache.

           It is highly recommanded to call this method a FTDI device is
           unplugged/plugged back since the last enumeration, as the device
           may appear on a different USB location each time it is plugged
           in.

           Failing to clear out the cache may lead to USB Error 19:
           ``Device may have been disconnected``.
        """
        cls.Lock.acquire()
        cls.UsbDevices = {}
        cls.Lock.release()

    @classmethod
    def get_device(cls, vendor, product, index=0, serial=None,
                   description=None):
        """Find a previously open device with the same vendor/product
           or initialize a new one, and return it.

           If several FTDI devices of the same kind (vid, pid) are connected
           to the host, either index or serial argument should be used to
           discriminate the FTDI device.

           index argument is not a reliable solution as the host may enumerate
           the USB device in random order. serial argument is more reliable
           selector and should always be prefered.

           Some FTDI devices support several interfaces/ports (such as FT2232H
           and FT4232H). The interface argument selects the FTDI port to use,
           starting from 1 (not 0).

           :param int vendor: USB vendor id
           :param int product: USB product id
           :param int index: optional selector, specified the n-th matching
                             FTDI enumerated USB device on the host
           :param str serial: optional selector, specified the FTDI device
                              by its serial number
           :param str interface: FTDI interface/port
           :return: PyUSB device instance
           :rtype: usb.core.Device
        """
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
                if isinstance(devs, set):
                    # there is no guarantee the same index with lead to the
                    # same device. Indexing should be reworked
                    devs = list(devs)
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
                # only change the active configuration if the active one is
                # not the first. This allows other libusb sessions running
                # with the same device to run seamlessly.
                try:
                    config = dev.get_active_configuration()
                    setconf = config.bConfigurationValue != 1
                except usb.core.USBError:
                    setconf = True
                if setconf:
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
        """Release a previously open device, if it not used anymore.

           :param usb_dev: a previously instanciated Usb device instance
           :type usb_deb: usb.core.Device
        """
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
        """Find a USB device and return it.

           This code re-implements the usb.core.find() method using a local
           cache to avoid calling several times the underlying LibUSB and the
           system USB calls to enumerate the available USB devices. As these
           calls are time-hungry (about 1 second/call), the enumerated devices
           are cached. It consumes a bit more memory but dramatically improves
           start-up time.
           Hopefully, this kludge is temporary and replaced with a better
           implementation from PyUSB at some point.

           :param int vendor: USB vendor id
           :param int product: USB product id
           :param bool nocache: bypass cache to re-enumerate USB devices on
                                the host
           :return: a set of USB device matching the vendor/product identifier
                    pair
           :rtype: set(usb.core.Device)

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
                    if device.idVendor in vpdict:
                        products = vpdict[device.idVendor]
                        if products and (device.idProduct not in products):
                            continue
                        devs.add(device)
                if platform == 'win32':
                    # ugly kludge for a boring OS:
                    # on Windows, the USB stack may enumerate the very same
                    # devices several times: a real device with N interface
                    # appears also as N device with as single interface.
                    # We only keep the "device" that declares the most
                    # interface count and discard the "virtual" ones.
                    filtered_devs = dict()
                    for dev in devs:
                        vid = dev.idVendor
                        pid = dev.idProduct
                        ifc = max([cfg.bNumInterfaces for cfg in dev])
                        sn = UsbTools.get_string(dev, dev.iSerialNumber)
                        k = (vid, pid, sn)
                        if k not in filtered_devs:
                            filtered_devs[k] = dev
                        else:
                            fdev = filtered_devs[k]
                            fifc = max([cfg.bNumInterfaces for cfg in fdev])
                            if fifc < ifc:
                                filtered_devs[k] = dev
                    devs = set(filtered_devs.values())
                cls.UsbDevices[vp] = devs
            return cls.UsbDevices[vp]
        finally:
            cls.Lock.release()

    @staticmethod
    def parse_url(urlstr, devclass, scheme, vdict, pdict, default_vendor):
        """Parse a device specifier URL.

           :param str url: the URL to parse
           :param devclass: class that implements the scheme
           :param scheme: scheme to match in the URL string (scheme://...)
           :param dict vdict: vendor name map of USB vendor ids
           :param dict pdict: vendor id map of product name map of product ids
           :param int default_vendor: default vendor id
           :return: a list of 5-tuple (vid, pid, sernum, iface, description)
                    device descriptors
           :rtype: list(tuple(int,int,str,int,str))
        """
        urlparts = urlsplit(urlstr)
        if scheme != urlparts.scheme:
            raise UsbToolsError("Invalid URL: %s" % urlstr)
        # general syntax: protocol://vendor:product[:index|:serial]/interface
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
                try:
                    product = to_int(plcomps[1])
                except ValueError:
                    raise UsbToolsError('Product %s is not referenced' %
                                        plcomps[1])
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
        idx = None
        if plcomps[2]:
            try:
                devidx = to_int(plcomps[2])
                if devidx > 255:
                    raise ValueError()
                idx = devidx
                if idx:
                    idx = devidx-1
            except ValueError:
                sernum = plcomps[2]
        candidates = []
        vendors = vendor and [vendor] or set(vdict.values())
        vps = set()
        for v in vendors:
            products = pdict.get(v, [])
            for p in products:
                vps.add((v, products[p]))
        devices = devclass.find_all(vps)
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
        if show_devices:
            UsbTools.show_devices(scheme, vdict, pdict, candidates)
            raise SystemExit(candidates and
                             'Please specify the USB device' or
                             'No USB-Serial device has been detected')
        if idx is None:
            if len(candidates) > 1:
                raise UsbToolsError('%d USB devices match URL' %
                                    len(candidates))
            idx = 0
        try:
            vendor, product, ifport, ifcount, description = \
                candidates[idx]
        except IndexError:
            raise UsbToolsError('No USB device matches URL %s' %
                                urlstr)
        if not vendor:
            cvendors = set([candidate[0] for candidate in candidates])
            if len(cvendors) == 1:
                vendor = cvendors.pop()
        if vendor not in pdict:
            raise UsbToolsError('Vendor ID %s not supported' %
                                (vendor and '0x%04x' % vendor))
        if not product:
            cproducts = set([candidate[1] for candidate in candidates
                            if candidate[0] == vendor])
            if len(cproducts) == 1:
                product = cproducts.pop()
        if product not in pdict[vendor].values():
            raise UsbToolsError('Product ID %s not supported' %
                                (product and '0x%04x' % product))
        return vendor, product, idx or 0, sernum, interface

    @staticmethod
    def show_devices(scheme, vdict, pdict, candidates, out=None):
        """Show supported devices. When the joker url ``scheme://*/?`` is
           specified as an URL, it generates a list of connected USB devices
           that match the supported USB devices. It can be used to provide the
           end-user with a list of valid URL schemes.

           :param dict vdict: vendor name map of USB vendor ids
           :param dict pdict: vendor id map of product name map of product ids
           :param candidates: candidate devices
           :type candidates: list(tuple(int,int,str,int,str))
           :param out: output stream, none for stdout
           :type out: file object or None
        """
        if not out:
            out = stdout
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
            print("Available interfaces:", file=out)
            serial_ifaces = []
            max_url_len = 0
            for scheme, vendor, product, serial, j, d in interfaces:
                fmt = '%s://%s/%d'
                parts = [vendor, product]
                if serial:
                    parts.append(serial)
                desc = d and '(%s)' % d
                # the description may contain characters that cannot be
                # emitted in the output stream encoding format
                try:
                    serial_url = fmt % (scheme, ':'.join(parts), j)
                except Exception:
                    serial_url = fmt % (scheme,
                                        ':'.join([vendor, product, '???']), j)
                try:
                    serial_desc = desc or ''
                except Exception:
                    serial_desc = ''
                max_url_len = max(max_url_len, len(serial_url))
                serial_ifaces.append((serial_url, serial_desc))
            for iface in serial_ifaces:
                print(('  %%-%ds   %%s' % max_url_len) % iface, file=out)
            print('', file=out)

    @classmethod
    def get_string(cls, device, strname):
        """Retrieve a string from the USB device, dealing with PyUSB API breaks

           :param device: USB device instance
           :type device: usb.core.Device
           :param str strname: the string identifier
           :return: the string read from the USB device
           :rtype: str
        """
        if cls.UsbApi is None:
            import inspect
            args, varargs, varkw, defaults = \
                inspect.signature(usb.core.Device.read).parameters
            if (len(args) >= 3) and args[1] == 'length':
                cls.UsbApi = 1
            else:
                cls.UsbApi = 2
        if cls.UsbApi == 2:
            return usb.util.get_string(device, strname)
        else:
            return usb.util.get_string(device, 64, strname)
