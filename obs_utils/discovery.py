import pyudev
from typing import Sequence

def show_current_vidpid() -> None:
    """Show VID:PID of connected USB devices."""
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        parent = device.find_parent("usb", "usb_device")
        if parent is None:
            continue
        if parent.get("ID_VENDOR_ID") and parent.get("ID_MODEL_ID"):
            print(
                f"Device: {device.device_node}, VID:PID = {parent.get('ID_VENDOR_ID')}:{parent.get('ID_MODEL_ID')}"
            )

def show_current_serials() -> None:
    """Show serial numbers of connected USB devices."""
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        parent = device.find_parent("usb", "usb_device")
        if parent is None:
            continue
        if parent.get("ID_SERIAL_SHORT"):
            print(
                f"Device: {device.device_node}, Serial Number = {parent.get('ID_SERIAL_SHORT')}"
            )


def port_by_vidpid(vidpid: str) -> str | None:
    """Return /dev/tty* port for given USB VID:PID."""
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        parent = device.find_parent("usb", "usb_device")
        if parent is None:
            continue
        if parent.get("ID_VENDOR_ID") and parent.get("ID_MODEL_ID"):
            if f"{parent.get('ID_VENDOR_ID')}:{parent.get('ID_MODEL_ID')}" == vidpid:
                return device.device_node.split("/")[-1]
    return None

def port_by_serial(serial: str) -> str | None:
    """Return /dev/tty* port for given USB serial number."""
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        parent = device.find_parent("usb", "usb_device")
        if parent is None:
            continue
        if parent.get("ID_SERIAL_SHORT"):
            if parent.get("ID_SERIAL_SHORT") == serial:
                return device.device_node.split("/")[-1]
    return None

def port_serial_search(serials: Sequence[str]) -> str | None:
    """ Search for a port from a tuple of serial numbers."""
    for serial in serials:
        port = port_by_serial(serial)
        if port is not None:
            return port
    return None

def print_search():
    print(show_current_vidpid())
    print(show_current_serials())

if __name__ == "__main__":
    print_search()
