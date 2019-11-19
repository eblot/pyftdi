# Copyright (c) 2014-2019, Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2016, Emmanuel Bouaziz <ebouaziz@free.fr>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Neotion nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL NEOTION BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""USB Helpers"""

from string import printable as printablechars
from sys import platform, stdout
from threading import RLock
from typing import (IO, List, Mapping, NamedTuple, Optional, Sequence, Set,
                    Tuple)
from urllib.parse import urlsplit
from usb.core import Device as UsbDevice, USBError
from usb.util import dispose_resources, get_string as usb_get_string
from .misc import to_int

#pylint: disable-msg=broad-except
#pylint: disable-msg=too-many-locals,too-many-branches,too-many-statements
#pylint: disable-msg=too-many-arguments, too-many-nested-blocks

UsbDeviceDescriptor = NamedTuple('UsbDeviceDescriptor',
                                 (('vid', int),
                                  ('pid', int),
                                  ('bus', Optional[int]),
                                  ('address', Optional[int]),
                                  ('sn', Optional[str]),
                                  ('index', Optional[int]),
                                  ('description', Optional[str])))
"""USB Device descriptor are used to report known information about a FTDI
   compatible device, and as a device selection filter

   * vid: vendor identifier, 16-bit integer
   * pid: product identifier, 16-bit integer
   * bus: USB bus identifier, host dependent integer
   * address: USB address identifier on a USB bus, host dependent integer
   * sn: serial number, string
   * index: integer, can be used to descriminate similar devices
   * description: device description, as a string

   To select a device, use None for unknown fields

   .. note::

     * Always prefer serial number to other identification methods if available
     * Prefer bus/address selector over index
"""

class UsbToolsError(Exception):
    """UsbTools error."""


class UsbTools:
    """Helpers to obtain information about connected USB devices."""

    # Need to maintain a list of reference USB devices, to circumvent a
    # limitation in pyusb that prevents from opening several times the same
    # USB device. The following dictionary used bus/address/vendor/product keys
    # to track (device, refcount) pairs
    Devices = {}
    Lock = RLock()
    UsbDevices = {}
    UsbApi = None

    @staticmethod
    def find_all(vps: Sequence[Tuple[int, int]], nocache: bool = False) -> \
            List[Tuple[UsbDeviceDescriptor, int]]:
        """Find all devices that match the specified vendor/product pairs.

           :param vps: a sequence of 2-tuple (vid, pid) pairs
           :param bool nocache: bypass cache to re-enumerate USB devices on
                                the host
           :return: a list of 2-tuple (UsbDeviceDescriptor, interface count)
        """
        devs = set()
        for vid, pid in vps:
            devs.update(UsbTools._find_devices(vid, pid, nocache))
        devices = set()
        for dev in devs:
            ifcount = max([cfg.bNumInterfaces for cfg in dev])
            sernum = UsbTools.get_string(dev, dev.iSerialNumber)
            description = UsbTools.get_string(dev, dev.iProduct)
            descriptor = UsbDeviceDescriptor(dev.idVendor, dev.idProduct,
                                             dev.bus, dev.address,
                                             sernum, None, description)
            devices.add((descriptor, ifcount))
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
    def get_device(cls, devdesc: UsbDeviceDescriptor) -> UsbDevice:
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

           :param devdesc: Device descriptor that identifies the device by
                           constraints.
           :return: PyUSB device instance
        """
        cls.Lock.acquire()
        try:
            if devdesc.index or devdesc.sn or devdesc.description:
                dev = None
                if not devdesc.vid:
                    raise ValueError('Vendor identifier is required')
                devs = cls._find_devices(devdesc.vid, devdesc.pid)
                if devdesc.description:
                    devs = [dev for dev in devs if
                            UsbTools.get_string(dev, dev.iProduct) ==
                            devdesc.description]
                if devdesc.sn:
                    devs = [dev for dev in devs if
                            UsbTools.get_string(dev, dev.iSerialNumber) ==
                            devdesc.sn]
                if devdesc.bus is not None and devdesc.address is not None:
                    devs = [dev for dev in devs if
                            (devdesc.bus == dev.bus and
                             devdesc.address == dev.address)]
                if isinstance(devs, set):
                    # there is no guarantee the same index with lead to the
                    # same device. Indexing should be reworked
                    devs = list(devs)
                try:
                    dev = devs[devdesc.index]
                except IndexError:
                    raise IOError("No such device")
            else:
                devs = cls._find_devices(devdesc.vid, devdesc.pid)
                dev = list(devs)[0] if devs else None
            if not dev:
                raise IOError('Device not found')
            try:
                devkey = (dev.bus, dev.address, devdesc.vid, devdesc.pid)
                if None in devkey[0:2]:
                    raise AttributeError('USB backend does not support bus '
                                         'enumeration')
            except AttributeError:
                devkey = (devdesc.vid, devdesc.pid)
            if devkey not in cls.Devices:
                # only change the active configuration if the active one is
                # not the first. This allows other libusb sessions running
                # with the same device to run seamlessly.
                try:
                    config = dev.get_active_configuration()
                    setconf = config.bConfigurationValue != 1
                except USBError:
                    setconf = True
                if setconf:
                    try:
                        dev.set_configuration()
                    except USBError:
                        pass
                cls.Devices[devkey] = [dev, 1]
            else:
                cls.Devices[devkey][1] += 1
            return cls.Devices[devkey][0]
        finally:
            cls.Lock.release()

    @classmethod
    def release_device(cls, usb_dev: UsbDevice):
        """Release a previously open device, if it not used anymore.

           :param usb_dev: a previously instanciated USB device instance
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
                        dispose_resources(cls.Devices[devkey][0])
                        del cls.Devices[devkey]
                    break
        finally:
            cls.Lock.release()

    @classmethod
    def _find_devices(cls, vendor: int, product: int,
                      nocache: bool = False) -> Set[UsbDevice]:
        """Find a USB device and return it.

           This code re-implements the usb.core.find() method using a local
           cache to avoid calling several times the underlying LibUSB and the
           system USB calls to enumerate the available USB devices. As these
           calls are time-hungry (about 1 second/call), the enumerated devices
           are cached. It consumes a bit more memory but dramatically improves
           start-up time.
           Hopefully, this kludge is temporary and replaced with a better
           implementation from PyUSB at some point.

           :param vendor: USB vendor id
           :param product: USB product id
           :param bool nocache: bypass cache to re-enumerate USB devices on
                                the host
           :return: a set of USB device matching the vendor/product identifier
                    pair
        """
        cls.Lock.acquire()
        try:
            backend = None
            candidates = ('libusb1', 'libusb10', 'libusb0', 'libusb01',
                          'openusb')
            usbmod = __import__('usb.backend', globals(), locals(),
                                candidates, 0)
            for candidate in candidates:
                try:
                    mod = getattr(usbmod, candidate)
                except AttributeError:
                    continue
                backend = mod.get_backend()
                if backend is not None:
                    break
            else:
                raise ValueError('No backend available')
            vidpid = (vendor, product)
            if nocache or (vidpid not in cls.UsbDevices):
                # not freed until Python runtime completion
                # enumerate_devices returns a generator, so back up the
                # generated device into a list. To save memory, we only
                # back up the supported devices
                devs = set()
                vpdict = {}
                vpdict.setdefault(vendor, [])
                vpdict[vendor].append(product)
                for dev in backend.enumerate_devices():
                    device = UsbDevice(dev, backend)
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
                        sernum = UsbTools.get_string(dev, dev.iSerialNumber)
                        k = (vid, pid, sernum)
                        if k not in filtered_devs:
                            filtered_devs[k] = dev
                        else:
                            fdev = filtered_devs[k]
                            fifc = max([cfg.bNumInterfaces for cfg in fdev])
                            if fifc < ifc:
                                filtered_devs[k] = dev
                    devs = set(filtered_devs.values())
                cls.UsbDevices[vidpid] = devs
            return cls.UsbDevices[vidpid]
        finally:
            cls.Lock.release()

    @classmethod
    def parse_url(cls, urlstr: str, scheme: str,
                  vdict: Mapping[str, int],
                  pdict: Mapping[int, Mapping[str, int]],
                  default_vendor: int) -> Tuple[UsbDeviceDescriptor, int]:
        """Parse a device specifier URL.

           :param url: the URL to parse
           :param scheme: scheme to match in the URL string (scheme://...)
           :param vdict: vendor name map of USB vendor ids
           :param pdict: vendor id map of product name map of product ids
           :param default_vendor: default vendor id
           :return: UsbDeviceDescriptor, interface)
        """
        urlparts = urlsplit(urlstr)
        if scheme != urlparts.scheme:
            raise UsbToolsError("Invalid URL: %s" % urlstr)
        # general syntax:
        #   protocol://vendor:product[:serial|:index|:bus:addr]/interface
        specifiers = urlparts.netloc.split(':')
        plcomps = specifiers + [''] * 2
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
        bus = None
        address = None
        locators = specifiers[2:]
        if len(locators) > 1:
            try:
                bus = int(locators[0], 16)
                address = int(locators[1], 16)
            except ValueError:
                raise UsbToolsError('Invalid bus/address: %s' %
                                    ':'.join(locators))
        else:
            if locators and locators[0]:
                try:
                    devidx = to_int(locators[0])
                    if devidx > 255:
                        raise ValueError()
                    idx = devidx
                    if idx:
                        idx = devidx-1
                except ValueError:
                    sernum = locators[0]
        candidates = []
        vendors = [vendor] if vendor else set(vdict.values())
        vps = set()
        for vid in vendors:
            products = pdict.get(vid, [])
            for pid in products:
                vps.add((vid, products[pid]))
        devices = cls.find_all(vps)
        if sernum:
            if sernum not in [dev.sn for dev, _ in devices]:
                raise UsbToolsError("No USB device with S/N %s" % sernum)
        for desc, ifcount in devices:
            if vendor and vendor != desc.vid:
                continue
            if product and product != desc.pid:
                continue
            if sernum and sernum != desc.sn:
                continue
            if bus is not None:
                if bus != desc.bus or address != desc.address:
                    continue
            candidates.append((desc, ifcount))
        if show_devices:
            UsbTools.show_devices(scheme, vdict, pdict, candidates)
            raise SystemExit(candidates and
                             'Please specify the USB device' or
                             'No USB-Serial device has been detected')
        if idx is None:
            if len(candidates) > 1:
                raise UsbToolsError("%d USB devices match URL '%s'" %
                                    (len(candidates), urlstr))
            idx = 0
        try:
            desc, _ = candidates[idx]
            vendor, product = desc[:2]
        except IndexError:
            raise UsbToolsError('No USB device matches URL %s' %
                                urlstr)
        if not vendor:
            cvendors = {candidate[0] for candidate in candidates}
            if len(cvendors) == 1:
                vendor = cvendors.pop()
        if vendor not in pdict:
            raise UsbToolsError('Vendor ID %s not supported' %
                                (vendor and '0x%04x' % vendor))
        if not product:
            cproducts = {candidate[1] for candidate in candidates
                         if candidate[0] == vendor}
            if len(cproducts) == 1:
                product = cproducts.pop()
        if product not in pdict[vendor].values():
            raise UsbToolsError('Product ID %s not supported' %
                                (product and '0x%04x' % product))
        devdesc = UsbDeviceDescriptor(vendor, product, desc.bus, desc.address,
                                      desc.sn, idx, desc.description)
        return devdesc, interface

    @classmethod
    def show_devices(cls, scheme: str,
                     vdict: Mapping[str, int],
                     pdict: Mapping[int, Mapping[str, int]],
                     candidates: Sequence[Tuple[UsbDeviceDescriptor, int]],
                     out: Optional[IO] = None):
        """Show supported devices. When the joker url ``scheme://*/?`` is
           specified as an URL, it generates a list of connected USB devices
           that match the supported USB devices. It can be used to provide the
           end-user with a list of valid URL schemes.

           :param scheme: scheme to match in the URL string (scheme://...)
           :param vdict: vendor name map of USB vendor ids
           :param pdict: vendor id map of product name map of product ids
           :param candidates: candidate devices
           :param out: output stream, none for stdout
        """
        if not candidates:
            return
        if not out:
            out = stdout
        indices = {}
        print("Available interfaces:", file=out)
        serial_ifaces = []
        max_url_len = 0
        for desc, ifcount in sorted(candidates):
            ikey = (desc.vid, desc.pid)
            indices[ikey] = indices.get(ikey, 0) + 1
            # try to find a matching string for the current vendor
            vendors = []
            # fallback if no matching string for the current vendor is found
            vendor = '%04x' % desc.vid
            for vidc in vdict:
                if vdict[vidc] == desc.vid:
                    vendors.append(vidc)
            if vendors:
                vendors.sort(key=len)
                vendor = vendors[0]
            # try to find a matching string for the current vendor
            # fallback if no matching string for the current product is found
            product = '%04x' % desc.pid
            try:
                products = []
                productids = pdict[desc.vid]
                for prdc in productids:
                    if productids[prdc] == desc.pid:
                        products.append(prdc)
                if products:
                    products.sort(key=len)
                    product = products[0]
            except KeyError:
                pass
            for port in range(1, ifcount+1):
                fmt = '%s://%s/%d'
                parts = [vendor, product]
                sernum = desc.sn
                if not sernum:
                    sernum = ''
                if [c for c in sernum if c not in printablechars or c == '?']:
                    serial = '%d' % indices[ikey]
                else:
                    serial = sernum
                if serial:
                    parts.append(serial)
                elif desc.bus is not None and desc.address is not None:
                    parts.append('%x' % desc.bus)
                    parts.append('%x' % desc.address)
                # the description may contain characters that cannot be
                # emitted in the output stream encoding format
                try:
                    serial_url = fmt % (scheme, ':'.join(parts), port)
                except Exception:
                    serial_url = fmt % (scheme,
                                        ':'.join([vendor, product, '???']),
                                        port)
                try:
                    serial_desc = '(%s)' % desc.description \
                        if desc.description else ''
                except Exception:
                    serial_desc = ''
                max_url_len = max(max_url_len, len(serial_url))
                serial_ifaces.append((serial_url, serial_desc))
        for iface in serial_ifaces:
            print(('  %%-%ds   %%s' % max_url_len) % iface, file=out)
        print('', file=out)

    @classmethod
    def get_string(cls, device: UsbDevice, strname: str) -> str:
        """Retrieve a string from the USB device, dealing with PyUSB API breaks

           :param device: USB device instance
           :param strname: the string identifier
           :return: the string read from the USB device
        """
        if cls.UsbApi is None:
            import inspect
            args, _, _, _ = \
                inspect.signature(UsbDevice.read).parameters
            if (len(args) >= 3) and args[1] == 'length':
                cls.UsbApi = 1
            else:
                cls.UsbApi = 2
        if cls.UsbApi == 2:
            return usb_get_string(device, strname)
        return usb_get_string(device, 64, strname)
