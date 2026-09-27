"""Hold every RealSense camera away from macOS's UVC driver for the life of the process.

librealsense powers each sensor on/off several times during init. On macOS every power-off
releases the libusb capture, the camera re-enumerates, and UVCAssistant races to grab it back
-> "failed to set power state". Capturing each camera once up front, through the *same* libusb
image librealsense loaded, and padding libusb's per-device capture count (every power-off
decrements it once per released interface: 3 for depth, 2 for color) keeps it above zero, so
librealsense's own releases never re-enumerate. Needs root. Same fix as
viam-modules/viam-camera-realsense#182 (libusb 1.0.26 there; the bundled one here is 1.0.29).

Used by myaloha.cameras and setup/check_cameras.py. Call capture_realsense() after `import pyrealsense2` and before creating any rs.context().
"""
import ctypes
import os
import sys

import pyrealsense2

INTEL_VID = 0x8086
# Capture-count padding per camera: ~20k power cycles. Once captured, each extra detach is pure
# bookkeeping (no USB traffic). Viam uses 1e6; 1e5 keeps Python startup to about a second.
BUDGET = 100_000


class _DeviceDescriptor(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8),
        ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16),
        ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8),
        ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8),
        ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16),
        ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8),
        ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8),
        ("bNumConfigurations", ctypes.c_uint8),
    ]


_usb = None  # the libusb image librealsense loaded, and one context for the life of the process
_ctx = ctypes.c_void_p()
_held: dict = {}  # (bus, address) -> open handle of each captured camera; a replug gets a new address


def _lib():
    global _usb
    if _usb is None:
        path = os.path.join(os.path.dirname(pyrealsense2.__file__), ".dylibs", "libusb-1.0.0.dylib")
        usb = ctypes.CDLL(path)  # already loaded by librealsense -> same image, same capture counts
        usb.libusb_error_name.restype = ctypes.c_char_p
        if (rc := usb.libusb_init(ctypes.byref(_ctx))) != 0:
            raise RuntimeError(f"libusb_init: {usb.libusb_error_name(rc).decode()}")
        _usb = usb
    return _usb


def _intel_devices(visit) -> None:
    """Call visit(usb, device, descriptor, (bus, address)) for every Intel USB device attached."""
    usb = _lib()
    devs = ctypes.POINTER(ctypes.c_void_p)()
    n = usb.libusb_get_device_list(_ctx, ctypes.byref(devs))
    try:
        for i in range(n):
            dev = ctypes.c_void_p(devs[i])
            desc = _DeviceDescriptor()
            usb.libusb_get_device_descriptor(dev, ctypes.byref(desc))
            if desc.idVendor == INTEL_VID:
                visit(usb, dev, desc, (usb.libusb_get_bus_number(dev), usb.libusb_get_device_address(dev)))
    finally:
        usb.libusb_free_device_list(devs, 0)   # keep the references: open handles use them


def capture_realsense() -> int:
    """Capture all attached RealSense cameras not captured yet (call again after a replug), and
    forget cameras that are gone. Returns how many were captured now."""
    if sys.platform != "darwin":
        return 0
    present, captured = set(), []

    def visit(usb, dev, desc, where):
        present.add(where)
        if where in _held:
            return
        handle = ctypes.c_void_p()
        if (rc := usb.libusb_open(dev, ctypes.byref(handle))) != 0:
            print(f"realsense_macos: open {desc.idProduct:#06x} failed: {usb.libusb_error_name(rc).decode()}")
            return
        if (rc := usb.libusb_detach_kernel_driver(handle, 0)) != 0:
            print(f"realsense_macos: capture {desc.idProduct:#06x} failed: {usb.libusb_error_name(rc).decode()}")
            usb.libusb_close(handle)
            return
        for _ in range(BUDGET - 1):
            usb.libusb_detach_kernel_driver(handle, 0)
        _held[where] = handle
        captured.append(where)

    _intel_devices(visit)
    for where in [w for w in _held if w not in present]:     # unplugged: its address may be reused
        _lib().libusb_close(_held.pop(where))
    return len(captured)


def usb_addresses() -> frozenset:
    """(bus, address) of every RealSense camera on USB, without opening any: cheap enough to poll.
    A replugged camera shows up at a new address."""
    if sys.platform != "darwin":
        return frozenset()
    out = set()
    _intel_devices(lambda usb, dev, desc, where: out.add(where))
    return frozenset(out)
